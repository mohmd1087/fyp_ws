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
import threading
import time
import urllib.request
from pathlib import Path

ORCHESTRATOR_URL = "http://localhost:5050/status"
POLL_INTERVAL = 2.0
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


def spawn_agent() -> tuple[subprocess.Popen, int]:
    """Launch `agent.py console` in a PTY. Returns (process, master_fd)."""
    master, slave = pty.openpty()
    proc = subprocess.Popen(
        [sys.executable, str(AGENT_SCRIPT), "console"],
        cwd=str(AGENT_SCRIPT.parent),
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

    try:
        while True:
            status = poll_status()
            state = (status or {}).get("state")

            if proc is not None and proc.poll() is not None:
                # Subprocess exited on its own (normal after finalize_order).
                log.info(f"Agent exited with code {proc.returncode}")
                try:
                    if master_fd is not None:
                        os.close(master_fd)
                except OSError:
                    pass
                proc = None
                master_fd = None

            if state == "AT_TABLE" and proc is None:
                table = (status or {}).get("current_table")
                log.info(f"AT_TABLE ({table}) — spawning voice agent in console mode")
                proc, master_fd = spawn_agent()
            elif state != "AT_TABLE" and proc is not None:
                log.info(f"State={state} — tearing down voice agent")
                terminate(proc, master_fd)
                proc = None
                master_fd = None

            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        log.info("Shutting down.")
        if proc is not None and master_fd is not None:
            terminate(proc, master_fd)


if __name__ == "__main__":
    main()
