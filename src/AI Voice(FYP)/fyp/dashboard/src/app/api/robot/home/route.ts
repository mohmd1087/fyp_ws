import { NextRequest, NextResponse } from "next/server";
import { verifyAuth } from "@/lib/auth";
import { getRealtimeBus } from "@/lib/realtime";

/** POST /api/robot/home — publish go-home command to Pusher (staff only) */
export async function POST(req: NextRequest) {
  const auth = await verifyAuth(req);
  if (!auth) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  const bus = getRealtimeBus();
  await bus.publish("robot", "robot.go_home", {
    dispatched_at: new Date().toISOString(),
  });

  return NextResponse.json({ ok: true });
}
