import { NextRequest, NextResponse } from "next/server";
import { getRealtimeBus } from "@/lib/realtime";

/** POST /api/robot/status — receive status from orchestrator, republish to Pusher */
export async function POST(req: NextRequest) {
  const agentKey = req.headers.get("x-agent-key");
  if (!agentKey || agentKey !== process.env.AGENT_API_KEY) {
    return NextResponse.json({ error: "Forbidden" }, { status: 403 });
  }

  const body = await req.json().catch(() => null);
  if (!body?.state) {
    return NextResponse.json({ error: "state required" }, { status: 400 });
  }

  const bus = getRealtimeBus();
  await bus.publish("robot", "robot.status", {
    state: body.state,
    current_table: body.current_table ?? null,
    timestamp: new Date().toISOString(),
  });

  return NextResponse.json({ ok: true });
}
