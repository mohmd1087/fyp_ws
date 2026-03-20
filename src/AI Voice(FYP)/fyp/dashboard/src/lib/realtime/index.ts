import type { RealtimeBus } from "./types";
import { PusherRealtimeBus } from "./pusher-server";

// To swap providers, change this factory only.
// Example: import { AblyRealtimeBus } from "./ably-server"
let bus: RealtimeBus | null = null;

export function getRealtimeBus(): RealtimeBus {
  if (!bus) {
    bus = new PusherRealtimeBus();
  }
  return bus;
}
