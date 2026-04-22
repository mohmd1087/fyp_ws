import { NextRequest, NextResponse } from "next/server";
import { prisma } from "@/lib/prisma";
import { AppendItemsSchema } from "@/lib/zod-schemas";
import { getRealtimeBus } from "@/lib/realtime";
import { rateLimit } from "@/lib/rate-limit";

type Totals = {
  subtotal: number;
  service_charge: number;
  service_rate: number;
  total: number;
  currency: string;
};

type Item = {
  item_id: string;
  name: string;
  unit_price: number;
  qty: number;
  modifications: string | null;
  allergens: string[];
  category: string;
  line_total: number;
};

function recalcTotals(items: Item[], existing: Totals): Totals {
  const subtotal = items.reduce((acc, it) => acc + it.unit_price * it.qty, 0);
  const service_rate = existing.service_rate;
  const service_charge = Math.round(subtotal * service_rate);
  return {
    subtotal,
    service_charge,
    service_rate,
    total: subtotal + service_charge,
    currency: existing.currency,
  };
}

/** POST /api/orders/[id]/append-items — append items + recompute totals (voice agent, API key auth) */
export async function POST(
  req: NextRequest,
  { params }: { params: Promise<{ id: string }> }
) {
  const agentKey = req.headers.get("x-agent-key");
  if (!agentKey || agentKey !== process.env.AGENT_API_KEY) {
    return NextResponse.json({ error: "Forbidden" }, { status: 403 });
  }

  if (rateLimit("agent:append", 60, 60)) {
    return NextResponse.json({ error: "Rate limited" }, { status: 429 });
  }

  const { id } = await params;

  const body = await req.json().catch(() => null);
  const parsed = AppendItemsSchema.safeParse(body);
  if (!parsed.success) {
    return NextResponse.json(
      { error: "Validation failed", details: parsed.error.flatten() },
      { status: 422 }
    );
  }

  const existing = await prisma.order.findUnique({ where: { id } });
  if (!existing) {
    return NextResponse.json({ error: "Order not found" }, { status: 404 });
  }

  const existingItems = (existing.items as unknown as Item[]) ?? [];
  const mergedItems: Item[] = [...existingItems, ...parsed.data.items];
  const newTotals = recalcTotals(
    mergedItems,
    existing.totals as unknown as Totals
  );

  const rawPayload =
    (existing.rawPayload as Record<string, unknown> | null) ?? {};
  const updatedRawPayload = {
    ...rawPayload,
    items: mergedItems,
    totals: newTotals,
  };

  const updated = await prisma.order.update({
    where: { id },
    data: {
      items: mergedItems as object[],
      totals: newTotals as object,
      rawPayload: updatedRawPayload as object,
    },
  });

  const bus = getRealtimeBus();
  await bus.publish("orders", "order.updated", {
    id: updated.id,
    status: updated.status,
    totals: newTotals,
    updatedAt: updated.updatedAt.toISOString(),
  });

  return NextResponse.json(
    { ok: true, order_id: updated.id, totals: newTotals, appended: parsed.data.items.length },
    { status: 200 }
  );
}
