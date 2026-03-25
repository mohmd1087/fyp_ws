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
    mic_queue: queue.Queue[bytes] = queue.Queue(maxsize=50)

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
    speaker_stream = sd.RawOutputStream(
        samplerate=SAMPLE_RATE,
        channels=NUM_CHANNELS,
        dtype="int16",
        blocksize=FRAME_SIZE,
    )
    speaker_stream.start()

    async def play_remote_audio(track: rtc.AudioTrack):
        """Async iterator that plays incoming remote audio frames to speakers."""
        audio_stream = rtc.AudioStream(track, sample_rate=SAMPLE_RATE, num_channels=NUM_CHANNELS)
        async for event in audio_stream:
            frame = event.frame
            audio_data = np.frombuffer(frame.data, dtype=np.int16)
            try:
                speaker_stream.write(audio_data)
            except sd.PortAudioError:
                break

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

    # --- Main loop: push mic frames + poll orchestrator state ---
    try:
        while True:
            # Push any available mic frames to LiveKit
            frames_pushed = 0
            while not mic_queue.empty() and frames_pushed < 10:
                data = mic_queue.get_nowait()
                frame = rtc.AudioFrame(
                    data=data,
                    sample_rate=SAMPLE_RATE,
                    num_channels=NUM_CHANNELS,
                    samples_per_channel=FRAME_SIZE,
                )
                await audio_source.capture_frame(frame)
                frames_pushed += 1

            # Check orchestrator state periodically
            status = await asyncio.to_thread(poll_status)
            if status is None:
                log.warning("Orchestrator unreachable, disconnecting...")
                break
            if status.get("state") != "AT_TABLE" or status.get("current_room") != room_name:
                log.info(f"State changed to '{status.get('state')}', disconnecting...")
                break

            await asyncio.sleep(POLL_INTERVAL)
    finally:
        mic_stream.stop()
        mic_stream.close()
        speaker_stream.stop()
        speaker_stream.close()
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
