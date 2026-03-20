import { NextRequest, NextResponse } from "next/server";
import { prisma } from "@/lib/prisma";
import { verifyAuth } from "@/lib/auth";
import { UpdateOrderSchema } from "@/lib/zod-schemas";
import { getRealtimeBus } from "@/lib/realtime";

/** GET /api/orders/[id] — get single order */
export async function GET(
  req: NextRequest,
  { params }: { params: Promise<{ id: string }> }
) {
  const auth = await verifyAuth(req);
  if (!auth) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  const { id } = await params;
  const order = await prisma.order.findUnique({ where: { id } });
  if (!order) {
    return NextResponse.json({ error: "Not found" }, { status: 404 });
  }

  return NextResponse.json({ order });
}

/** PATCH /api/orders/[id] — update order status */
export async function PATCH(
  req: NextRequest,
  { params }: { params: Promise<{ id: string }> }
) {
  const auth = await verifyAuth(req);
  if (!auth) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  const body = await req.json().catch(() => null);
  const parsed = UpdateOrderSchema.safeParse(body);
  if (!parsed.success) {
    return NextResponse.json({ error: "Invalid body" }, { status: 400 });
  }

  const { id } = await params;

  try {
    const order = await prisma.order.update({
      where: { id },
      data: { status: parsed.data.status },
    });

    // Publish realtime update
    const bus = getRealtimeBus();
    await bus.publish("orders", "order.updated", {
      id: order.id,
      status: order.status,
      updatedAt: order.updatedAt.toISOString(),
    });

    return NextResponse.json({ order });
  } catch {
    return NextResponse.json({ error: "Order not found" }, { status: 404 });
  }
}
