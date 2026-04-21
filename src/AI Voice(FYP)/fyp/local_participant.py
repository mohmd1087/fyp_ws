"""
Local participant that bridges the laptop's mic/speaker into the LiveKit room.

Polls the waiter orchestrator at localhost:5050/status. When the robot is
AT_TABLE, auto-joins the LiveKit room, publishes mic audio, and plays the
agent's audio through the speakers. Disconnects when the robot leaves.

Usage:
    conda activate fyp
    python local_participant.py
"""

import asyncio
import json
import logging
import os
import queue
import threading
import urllib.request

import numpy as np
import sounddevice as sd
from dotenv import load_dotenv
from livekit import api as livekit_api
from livekit import rtc

load_dotenv(".env.local")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("local_participant")

# Audio constants
SAMPLE_RATE = 48000
NUM_CHANNELS = 1
FRAME_DURATION_MS = 20
FRAME_SIZE = SAMPLE_RATE * FRAME_DURATION_MS // 1000  # 960 samples

# Polling
ORCHESTRATOR_URL = "http://localhost:5050/status"
POLL_INTERVAL = 2.0

# LiveKit credentials (from .env.local)
LIVEKIT_URL = os.getenv("LIVEKIT_URL", "")
LIVEKIT_API_KEY = os.getenv("LIVEKIT_API_KEY", "")
LIVEKIT_API_SECRET = os.getenv("LIVEKIT_API_SECRET", "")

PARTICIPANT_IDENTITY = "robot-customer"


def poll_status() -> dict | None:
    """GET the orchestrator status. Returns parsed JSON or None on failure."""
    try:
        with urllib.request.urlopen(ORCHESTRATOR_URL, timeout=3) as resp:
            return json.loads(resp.read())
    except Exception:
        return None


def generate_token(room_name: str) -> str:
    """Generate a LiveKit access token for the given room."""
    token = (
        livekit_api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
        .with_identity(PARTICIPANT_IDENTITY)
        .with_grants(livekit_api.VideoGrants(room_join=True, room=room_name))
    )
    return token.to_jwt()


async def run_session(room_name: str):
    """Connect to a LiveKit room, bridge mic/speaker, monitor orchestrator state."""
    token = generate_token(room_name)
    room = rtc.Room()

    log.info(f"Connecting to room '{room_name}'...")
    await room.connect(LIVEKIT_URL, token)
    log.info(f"Connected to room '{room_name}' as '{PARTICIPANT_IDENTITY}'")

    # --- Mic → Room (publish) ---
    audio_source = rtc.AudioSource(SAMPLE_RATE, NUM_CHANNELS)
    mic_track = rtc.LocalAudioTrack.create_audio_track("mic", audio_source)

    publish_opts = rtc.TrackPublishOptions()
    publish_opts.source = rtc.TrackSource.Value("SOURCE_MICROPHONE")
    await room.local_participant.publish_track(mic_track, publish_opts)
    log.info("Mic track published")

    # Queue to bridge sounddevice callback thread → async loop
    mic_queue: queue.Queue[bytes] = queue.Queue(maxsize=200)

    def mic_callback(indata, frames, time_info, status):
        if status:
            log.warning(f"Mic status: {status}")
        mic_queue.put(bytes(indata))

    mic_stream = sd.RawInputStream(
        samplerate=SAMPLE_RATE,
        channels=NUM_CHANNELS,
        dtype="int16",
        blocksize=FRAME_SIZE,
        callback=mic_callback,
    )
    mic_stream.start()

    # --- Room → Speaker (subscribe) ---
    # Callback-driven output with a jitter-tolerant depth target. PortAudio's
    # own thread pulls int16 samples from `speaker_buffer` at the card's rate,
    # independent of the asyncio loop. We gate playback on a minimum depth
    # (JITTER_FLOOR) so a brief network stall doesn't drain the buffer and
    # cause the choppy "cut off, come back" pattern. Playback only starts once
    # ~400 ms has buffered; if the depth later falls below ~150 ms we hold by
    # emitting silence until it refills, rather than stuttering.
    BYTES_PER_SEC = SAMPLE_RATE * 2 * NUM_CHANNELS  # int16 mono @ 48kHz
    TARGET_DEPTH_BYTES = int(BYTES_PER_SEC * 0.75)   # ~750 ms — generous cushion for jittery links
    JITTER_FLOOR_BYTES = int(BYTES_PER_SEC * 0.30)   # ~300 ms hold threshold

    speaker_buffer = bytearray()
    speaker_lock = threading.Lock()
    speaker_state = {"playing": False, "last_log": 0.0}

    # Pre-roll silence so the stream starts cleanly; real audio must still
    # accumulate to TARGET_DEPTH_BYTES before the gate opens.
    speaker_buffer.extend(b"\x00" * TARGET_DEPTH_BYTES)

    def speaker_callback(outdata, frames, time_info, status):
        if status:
            log.debug(f"Speaker status: {status}")
        needed = frames * 2 * NUM_CHANNELS
        with speaker_lock:
            have = len(speaker_buffer)

            # Gate closed → holding to rebuild depth. Play silence.
            if not speaker_state["playing"]:
                if have >= TARGET_DEPTH_BYTES:
                    speaker_state["playing"] = True
                else:
                    outdata[:] = b"\x00" * needed
                    return

            # Gate open: normal playback, but if we drop below the floor,
            # pause and wait for depth to rebuild rather than stuttering.
            if have < JITTER_FLOOR_BYTES:
                speaker_state["playing"] = False
                outdata[:] = b"\x00" * needed
                return

            if have >= needed:
                outdata[:] = bytes(speaker_buffer[:needed])
                del speaker_buffer[:needed]
            else:
                outdata[:have] = bytes(speaker_buffer[:have])
                outdata[have:] = b"\x00" * (needed - have)
                speaker_buffer.clear()

    speaker_stream = sd.RawOutputStream(
        samplerate=SAMPLE_RATE,
        channels=NUM_CHANNELS,
        dtype="int16",
        blocksize=0,
        latency="high",
        callback=speaker_callback,
    )
    speaker_stream.start()

    async def play_remote_audio(track: rtc.AudioTrack):
        """Feed remote audio frames into the speaker ring buffer. Non-blocking.
        Also logs buffer depth every 2s so jitter events are visible."""
        import time as _time
        audio_stream = rtc.AudioStream(track, sample_rate=SAMPLE_RATE, num_channels=NUM_CHANNELS)
        async for event in audio_stream:
            frame_bytes = bytes(event.frame.data)
            with speaker_lock:
                speaker_buffer.extend(frame_bytes)
                depth = len(speaker_buffer)
                now = _time.monotonic()
                if now - speaker_state["last_log"] >= 2.0:
                    depth_ms = int(depth * 1000 / BYTES_PER_SEC)
                    state = "PLAY" if speaker_state["playing"] else "HOLD"
                    log.info(f"[spk] depth={depth_ms}ms state={state}")
                    speaker_state["last_log"] = now

    speaker_tasks: list[asyncio.Task] = []

    @room.on("track_subscribed")
    def on_track_subscribed(
        track: rtc.Track,
        publication: rtc.TrackPublication,
        participant: rtc.RemoteParticipant,
    ):
        if track.kind == rtc.TrackKind.KIND_AUDIO:
            log.info(f"Agent audio track received from '{participant.identity}'")
            task = asyncio.create_task(play_remote_audio(track))
            speaker_tasks.append(task)

    # Also handle tracks that were already subscribed before our handler was registered
    for participant in room.remote_participants.values():
        for pub in participant.track_publications.values():
            if pub.track and pub.track.kind == rtc.TrackKind.KIND_AUDIO:
                log.info(f"Agent audio track already present from '{participant.identity}'")
                task = asyncio.create_task(play_remote_audio(pub.track))
                speaker_tasks.append(task)

    # --- Concurrent tasks: mic pump + orchestrator watchdog ---
    # Each runs at its own cadence so mic frames can't starve waiting on HTTP,
    # and HTTP can't starve waiting on audio. The speaker side is already
    # independent (PortAudio callback pulls directly from its buffer).
    disconnect_event = asyncio.Event()

    async def mic_pump():
        # Blocking Queue.get runs off the event loop, so frames flow the moment
        # sounddevice's callback enqueues them — no batching, no polling.
        frame_count = 0
        import time as _time
        last_log = _time.monotonic()
        while True:
            data = await asyncio.to_thread(mic_queue.get)
            if data is None:
                return
            frame = rtc.AudioFrame(
                data=data,
                sample_rate=SAMPLE_RATE,
                num_channels=NUM_CHANNELS,
                samples_per_channel=FRAME_SIZE,
            )
            await audio_source.capture_frame(frame)
            frame_count += 1
            now = _time.monotonic()
            if now - last_log >= 2.0:
                samples = np.frombuffer(data, dtype=np.int16)
                rms = int(np.sqrt(np.mean(samples.astype(np.float32) ** 2))) if len(samples) else 0
                log.info(f"[mic] {frame_count} frames pushed in last {now-last_log:.1f}s, last frame RMS={rms}")
                frame_count = 0
                last_log = now

    async def watch_orchestrator():
        while not disconnect_event.is_set():
            status = await asyncio.to_thread(poll_status)
            if status is None:
                log.warning("Orchestrator unreachable, disconnecting...")
                disconnect_event.set()
                return
            if status.get("state") != "AT_TABLE" or status.get("current_room") != room_name:
                log.info(f"State changed to '{status.get('state')}', disconnecting...")
                disconnect_event.set()
                return
            await asyncio.sleep(POLL_INTERVAL)

    mic_task = asyncio.create_task(mic_pump())
    watch_task = asyncio.create_task(watch_orchestrator())
    try:
        await disconnect_event.wait()
    finally:
        mic_stream.stop()
        mic_stream.close()
        speaker_stream.stop()
        speaker_stream.close()
        mic_queue.put(None)  # unblock mic_pump's blocking Queue.get
        mic_task.cancel()
        watch_task.cancel()
        for t in speaker_tasks:
            t.cancel()
        await room.disconnect()
        log.info(f"Disconnected from room '{room_name}'")


async def main():
    if not LIVEKIT_URL or not LIVEKIT_API_KEY or not LIVEKIT_API_SECRET:
        log.error("Missing LIVEKIT_URL / LIVEKIT_API_KEY / LIVEKIT_API_SECRET in .env.local")
        return

    log.info("Local participant started. Waiting for robot to reach a table...")

    while True:
        status = await asyncio.to_thread(poll_status)

        if status and status.get("state") == "AT_TABLE" and status.get("current_room"):
            room_name = status["current_room"]
            try:
                await run_session(room_name)
            except Exception as e:
                log.error(f"Session error: {e}")
            log.info("Waiting for next table dispatch...")
        else:
            if status:
                log.debug(f"Orchestrator state: {status.get('state', 'unknown')}")

        await asyncio.sleep(POLL_INTERVAL)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log.info("Shutting down.")
