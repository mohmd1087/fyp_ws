/**
 * Simple in-memory sliding window rate limiter.
 * For production with multiple Vercel instances, replace with Upstash Redis:
 *   import { Ratelimit } from "@upstash/ratelimit"
 */

const store = new Map<string, { count: number; reset: number }>();

/**
 * Returns `true` if the key has exceeded the limit (i.e., request is blocked).
 * @param key    - unique identifier (e.g. "login:<ip>")
 * @param limit  - max requests allowed in window
 * @param windowSec - window duration in seconds
 */
export function rateLimit(
  key: string,
  limit: number,
  windowSec: number
): boolean {
  const now = Math.floor(Date.now() / 1000);
  const entry = store.get(key);

  if (!entry || entry.reset < now) {
    store.set(key, { count: 1, reset: now + windowSec });
    return false;
  }

  entry.count += 1;
  return entry.count > limit;
}
