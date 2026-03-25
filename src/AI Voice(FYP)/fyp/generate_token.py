"""Generate a LiveKit access token for testing the voice agent."""

from dotenv import load_dotenv
import os

load_dotenv(".env.local")

from livekit import api

token = api.AccessToken(
    os.getenv("LIVEKIT_API_KEY"),
    os.getenv("LIVEKIT_API_SECRET"),
).with_identity("test-user") \
 .with_grants(api.VideoGrants(
    room_join=True,
    room="table-1",
 ))

print("\n=== LiveKit Token ===")
print(token.to_jwt())
print(f"\nRoom: table-1")
print(f"URL:  {os.getenv('LIVEKIT_URL')}")
print("\nPaste this token into the playground at https://agents-playground.livekit.io")
print("Set the LiveKit URL to:", os.getenv("LIVEKIT_URL"))
