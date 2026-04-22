"""
Voice agent supervisor.

Polls the orchestrator at localhost:5050/status. When state becomes AT_TABLE,
spawns `python agent.py console` inside a PTY (so rich's Live/readkey don't
choke on a missing TTY) and streams its output. When state leaves AT_TABLE, or
the subprocess exits on its own (e.g. finalize_order → /order_complete →
orchestrator transitions to NAVIGATING_HOME), the subprocess is torn down and
the supervisor waits for the next dispatch.

This is an alternative to the LiveKit-bridge path (`agent.py dev` +
`local_participant.py`). Both paths remain available — pick one at launch:
the bridge path uses LiveKit / WebRTC, this path uses console mode (direct
sounddevice audio) and has no WebRTC layer.

Usage:
    conda activate fyp
    python run_on_at_table.py
"""

import json
import logging
import os
import pty
import signal
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from pathlib import Path

ORCHESTRATOR_URL = "http://localhost:5050/status"
POLL_INTERVAL = 0.3
AGENT_SCRIPT = Path(__file__).resolve().parent / "agent.py"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("supervisor")


def poll_status() -> dict | None:
    try:
        with urllib.request.urlopen(ORCHESTRATOR_URL, timeout=3) as resp:
            return json.loads(resp.read())
    except Exception:
        return None


def spawn_agent(
    mode: str = "order",
    extra_env: dict[str, str] | None = None,
) -> tuple[subprocess.Popen, int]:
    """Launch `agent.py console` in a PTY. Returns (process, master_fd).

    `mode` is passed to the child as AGENT_MODE env var ("order" or "followup").
    `extra_env` is merged in after AGENT_MODE — used e.g. to carry
    FOLLOWUP_ORDER_ID into the follow-up agent.
    """
    master, slave = pty.openpty()
    env = os.environ.copy()
    env["AGENT_MODE"] = mode
    if extra_env:
        env.update(extra_env)
    proc = subprocess.Popen(
        [sys.executable, str(AGENT_SCRIPT), "console"],
        cwd=str(AGENT_SCRIPT.parent),
        env=env,
        stdin=slave,
        stdout=slave,
        stderr=slave,
        preexec_fn=os.setsid,
        close_fds=True,
    )
    os.close(slave)

    def _drain() -> None:
        try:
            while True:
                chunk = os.read(master, 4096)
                if not chunk:
                    return
                sys.stdout.buffer.write(chunk)
                sys.stdout.flush()
        except OSError:
            return

    threading.Thread(target=_drain, daemon=True).start()
    return proc, master


def terminate(proc: subprocess.Popen, master_fd: int) -> None:
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=3)
    try:
        os.close(master_fd)
    except OSError:
        pass


def main() -> None:
    if not AGENT_SCRIPT.exists():
        log.error(f"agent.py not found at {AGENT_SCRIPT}")
        sys.exit(1)

    log.info("Watching orchestrator. Agent will launch on AT_TABLE.")
    proc: subprocess.Popen | None = None
    master_fd: int | None = None
    armed: bool = False  # True once we've written the arm sentinel file
    arm_file: str = ""   # Path to sentinel file for the current pre-warmed proc

    try:
        while True:
            status = poll_status()
            state = (status or {}).get("state")
            mode = (status or {}).get("mode")  # "delivery" or "order"

            if proc is not None and proc.poll() is not None:
                # Subprocess exited on its own (normal after finalize_order).
                log.info(f"Agent exited with code {proc.returncode}")
                try:
                    if master_fd is not None:
                        os.close(master_fd)
                except OSError:
                    pass
                if arm_file:
                    try:
                        os.remove(arm_file)
                    except OSError:
                        pass
                proc = None
                master_fd = None
                armed = False
                arm_file = ""

            # Spawn rules:
            #   - NAVIGATING_TO_TABLE + no tray (order) -> pre-warm agent while
            #     the robot drives so it's ready on arrival (saves ~5-7s).
            #     Spawned UNARMED; supervisor touches arm_file on AT_TABLE.
            #   - AT_TABLE + no tray (order)            -> keep agent running.
            #   - AT_TABLE + tray (delivery)            -> stay silent during
            #     nav and tray cycle (customer only hears agent on followup).
            #   - AT_TABLE_FOLLOWUP                     -> spawn in followup mode.
            should_run = (
                (state == "NAVIGATING_TO_TABLE" and mode == "order")
                or (state == "AT_TABLE" and mode == "order")
                or state == "AT_TABLE_FOLLOWUP"
            )

            if not should_run and proc is not None:
                log.info(f"State={state} mode={mode} — tearing down voice agent")
                terminate(proc, master_fd)
                proc = None
                master_fd = None
                armed = False
                if arm_file:
                    try:
                        os.remove(arm_file)
                    except OSError:
                        pass
                arm_file = ""

            if should_run and proc is None:
                table = (status or {}).get("current_table")
                agent_mode = "followup" if state == "AT_TABLE_FOLLOWUP" else "order"
                extra_env: dict[str, str] = {}
                if agent_mode == "followup":
                    oid = (status or {}).get("order_id")
                    if oid:
                        extra_env["FOLLOWUP_ORDER_ID"] = oid
                # Pre-warm path: only when we spawn *during* nav. If we're
                # entering directly at AT_TABLE (race), spawn already armed.
                prewarm = state == "NAVIGATING_TO_TABLE"
                extra_env["AGENT_ARMED"] = "0" if prewarm else "1"
                if prewarm:
                    # Reserve a temp path (don't create yet — agent polls for its appearance).
                    arm_fd, arm_file = tempfile.mkstemp(prefix="agent_arm_")
                    os.close(arm_fd)
                    os.remove(arm_file)
                    extra_env["AGENT_ARM_FILE"] = arm_file
                else:
                    arm_file = ""
                log.info(
                    f"State={state} ({table}) — spawning voice agent "
                    f"(AGENT_MODE={agent_mode}, armed={not prewarm}"
                    + (f", FOLLOWUP_ORDER_ID={extra_env['FOLLOWUP_ORDER_ID']}"
                       if "FOLLOWUP_ORDER_ID" in extra_env else "")
                    + ")"
                )
                proc, master_fd = spawn_agent(agent_mode, extra_env=extra_env)
                armed = not prewarm

            # If the pre-warmed agent is waiting, touch the sentinel file to
            # unblock its session.start() + greeting.
            if proc is not None and not armed and state == "AT_TABLE" and mode == "order":
                try:
                    open(arm_file, "w").close()
                    log.info("State=AT_TABLE — wrote arm sentinel to unblock voice agent.")
                    armed = True
                except OSError as e:
                    log.warning(f"Failed to write arm sentinel: {e}")

            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        log.info("Shutting down.")
        if proc is not None and master_fd is not None:
            terminate(proc, master_fd)


if __name__ == "__main__":
    main()
