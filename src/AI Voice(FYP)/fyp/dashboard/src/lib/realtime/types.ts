/**
 * Abstract realtime bus interface.
 *
 * Why not host WebSockets in Next.js on Vercel?
 * Vercel Functions are serverless — they spin up per-request and cannot maintain
 * long-lived WebSocket connections. We use an external managed provider (Pusher)
 * for realtime pub/sub. This interface makes swapping providers easy: just change
 * the factory in ./index.ts.
 */
export interface RealtimeBus {
  publish(channel: string, event: string, data: unknown): Promise<void>;
}

export interface OrderCreatedEvent {
  id: string;
  status: string;
  orderType: string;
  customer: { name: string | null; phone: string | null };
  totals: { total: number; currency: string };
  createdAt: string;
}

export interface OrderUpdatedEvent {
  id: string;
  status: string;
  updatedAt: string;
}
