import { NextRequest, NextResponse } from "next/server";
import { verifyAuth } from "@/lib/auth";
import { getRealtimeBus } from "@/lib/realtime";

/** POST /api/robot/dispatch — publish dispatch command to Pusher (staff only) */
export async function POST(req: NextRequest) {
  const auth = await verifyAuth(req);
  if (!auth) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  const body = await req.json().catch(() => null);
  const tableId = body?.table_id;
  if (!tableId || typeof tableId !== "string") {
    return NextResponse.json({ error: "table_id required" }, { status: 400 });
  }

  const bus = getRealtimeBus();
  await bus.publish("robot", "robot.dispatch", {
    table_id: tableId,
    dispatched_at: new Date().toISOString(),
  });

  return NextResponse.json({ ok: true, table_id: tableId });
}
