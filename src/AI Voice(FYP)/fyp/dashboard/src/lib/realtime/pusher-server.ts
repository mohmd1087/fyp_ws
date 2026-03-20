import Pusher from "pusher";
import type { RealtimeBus } from "./types";

let instance: Pusher | null = null;

function getPusherServer(): Pusher {
  if (!instance) {
    instance = new Pusher({
      appId: process.env.PUSHER_APP_ID!,
      key: process.env.NEXT_PUBLIC_PUSHER_KEY!,
      secret: process.env.PUSHER_SECRET!,
      cluster: process.env.NEXT_PUBLIC_PUSHER_CLUSTER!,
      useTLS: true,
    });
  }
  return instance;
}

export class PusherRealtimeBus implements RealtimeBus {
  async publish(
    channel: string,
    event: string,
    data: unknown
  ): Promise<void> {
    try {
      await getPusherServer().trigger(channel, event, data);
    } catch (err) {
      // Log but never throw — realtime failure must not block order persistence
      console.error("[RealtimeBus] Pusher publish failed:", err);
    }
  }
}
