"use client";

import { useEffect, useRef } from "react";
import PusherClient from "pusher-js";

let pusherClient: PusherClient | null = null;

function getClient(): PusherClient {
  if (!pusherClient) {
    pusherClient = new PusherClient(
      process.env.NEXT_PUBLIC_PUSHER_KEY!,
      { cluster: process.env.NEXT_PUBLIC_PUSHER_CLUSTER! }
    );
  }
  return pusherClient;
}

/**
 * Subscribe to a Pusher channel and bind event handlers.
 * Handlers are stored in a ref so re-renders don't cause re-subscriptions.
 */
export function useRealtimeChannel(
  channel: string,
  events: Record<string, (data: unknown) => void>
) {
  const eventsRef = useRef(events);
  eventsRef.current = events;

  useEffect(() => {
    const client = getClient();
    const ch = client.subscribe(channel);

    const entries = Object.entries(eventsRef.current);
    for (const [event, handler] of entries) {
      ch.bind(event, handler);
    }

    return () => {
      for (const [event] of entries) {
        ch.unbind(event);
      }
      client.unsubscribe(channel);
    };
  }, [channel]);
}
