import { NextRequest, NextResponse } from "next/server";
import { prisma } from "@/lib/prisma";
import { verifyAuth } from "@/lib/auth";
import {
  CreateOrderSchema,
  GetOrdersQuerySchema,
} from "@/lib/zod-schemas";
import { getRealtimeBus } from "@/lib/realtime";
import { rateLimit } from "@/lib/rate-limit";

/** GET /api/orders — list orders (staff only, JWT auth) */
export async function GET(req: NextRequest) {
  const auth = await verifyAuth(req);
  if (!auth) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  const { searchParams } = new URL(req.url);
  const query = GetOrdersQuerySchema.safeParse(
    Object.fromEntries(searchParams)
  );
  if (!query.success) {
    return NextResponse.json({ error: "Bad query" }, { status: 400 });
  }

  const { status, search, limit } = query.data;

  const orders = await prisma.order.findMany({
    where: {
      ...(status ? { status } : {}),
      ...(search
        ? {
            OR: [
              { id: { contains: search, mode: "insensitive" as const } },
              { tableNumber: { contains: search, mode: "insensitive" as const } },
            ],
          }
        : {}),
    },
    orderBy: { createdAt: "desc" },
    take: limit,
  });

  return NextResponse.json({ orders });
}

/** POST /api/orders — create order (voice agent, API key auth) */
export async function POST(req: NextRequest) {
  // Agent key auth — NOT cookie auth
  const agentKey = req.headers.get("x-agent-key");
  if (!agentKey || agentKey !== process.env.AGENT_API_KEY) {
    return NextResponse.json({ error: "Forbidden" }, { status: 403 });
  }

  // Rate limit: 60 orders per minute
  if (rateLimit("agent:post", 60, 60)) {
    return NextResponse.json({ error: "Rate limited" }, { status: 429 });
  }

  const body = await req.json().catch(() => null);
  const parsed = CreateOrderSchema.safeParse(body);
  if (!parsed.success) {
    return NextResponse.json(
      { error: "Validation failed", details: parsed.error.flatten() },
      { status: 422 }
    );
  }

  const data = parsed.data;

  // Upsert for idempotency — agent might retry on timeout
  const order = await prisma.order.upsert({
    where: { id: data.order_id },
    update: {}, // ignore duplicate POSTs
    create: {
      id: data.order_id,
      createdAt: new Date(data.created_at_unix * 1000),
      tableNumber: data.dine_in?.table_number ?? null,
      orderType: data.order_type,
      status: "PENDING",
      customer: data.customer as object,
      dineIn: (data.dine_in as object) ?? undefined,
      preferences: data.preferences as object,
      items: data.items as object[],
      totals: data.totals as object,
      rawPayload: data as object,
    },
  });

  // Publish realtime event (best-effort, never blocks)
  const bus = getRealtimeBus();
  await bus.publish("orders", "order.created", {
    id: order.id,
    status: order.status,
    orderType: order.orderType,
    customer: order.customer,
    totals: order.totals,
    createdAt: order.createdAt.toISOString(),
  });

  return NextResponse.json({ ok: true, order_id: order.id }, { status: 201 });
}
