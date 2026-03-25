#!/usr/bin/env python3
"""
Lightweight test script to verify that dashboard dispatch events
reach this machine via Pusher. No ROS2 required.

Usage:
    pip install pysher
    python3 test_pusher_dispatch.py

Then dispatch from the dashboard and watch for output here.
"""

import json
import pysher

PUSHER_KEY = "65f267cd383ad90fde17"
PUSHER_CLUSTER = "ap2"


def on_dispatch(data):
    payload = json.loads(data) if isinstance(data, str) else data
    print(f"\n[DISPATCH RECEIVED] {json.dumps(payload, indent=2)}\n")


def on_connected(data):
    channel = pusher.subscribe("robot")
    channel.bind("robot.dispatch", on_dispatch)
    print("[CONNECTED] Subscribed to Pusher 'robot' channel.")
    print("[WAITING]   Dispatch from the dashboard now...\n")


pusher = pysher.Pusher(PUSHER_KEY, cluster=PUSHER_CLUSTER)
pusher.connection.bind("pusher:connection_established", on_connected)
pusher.connect()

print("Connecting to Pusher...")

try:
    import time
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopped.")
    pusher.disconnect()
