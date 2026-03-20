"use client";

import { useState, useCallback } from "react";
import { toast } from "sonner";
import { useRealtimeChannel } from "@/hooks/use-realtime";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";

type RobotState =
  | "IDLE"
  | "NAVIGATING_TO_TABLE"
  | "AT_TABLE"
  | "NAVIGATING_HOME";

const STATE_LABELS: Record<
  RobotState,
  { label: string; variant: "default" | "secondary" | "destructive" | "outline" }
> = {
  IDLE: { label: "Idle", variant: "secondary" },
  NAVIGATING_TO_TABLE: { label: "Navigating", variant: "default" },
  AT_TABLE: { label: "At Table", variant: "outline" },
  NAVIGATING_HOME: { label: "Returning", variant: "secondary" },
};

const TABLES = ["table-1", "table-2"];

export function RobotDispatch() {
  const [robotState, setRobotState] = useState<RobotState>("IDLE");
  const [currentTable, setCurrentTable] = useState<string | null>(null);
  const [dispatching, setDispatching] = useState(false);

  const handleStatus = useCallback((data: unknown) => {
    const event = data as { state: RobotState; current_table: string | null };
    setRobotState(event.state);
    setCurrentTable(event.current_table);
  }, []);

  useRealtimeChannel("robot", {
    "robot.status": handleStatus,
  });

  async function dispatch(tableId: string) {
    setDispatching(true);
    try {
      const res = await fetch("/api/robot/dispatch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ table_id: tableId }),
      });
      if (!res.ok) {
        const err = await res.json();
        toast.error(err.error ?? "Dispatch failed");
      } else {
        toast.success(`Robot dispatched to ${tableId}`);
      }
    } catch {
      toast.error("Network error");
    } finally {
      setDispatching(false);
    }
  }

  const busy = robotState !== "IDLE";
  const stateInfo = STATE_LABELS[robotState] ?? STATE_LABELS.IDLE;

  return (
    <div className="flex items-center gap-3">
      <Badge variant={stateInfo.variant}>
        {stateInfo.label}
        {currentTable ? ` (${currentTable})` : ""}
      </Badge>
      {TABLES.map((t) => (
        <Button
          key={t}
          variant="outline"
          size="sm"
          disabled={busy || dispatching}
          onClick={() => dispatch(t)}
        >
          {t}
        </Button>
      ))}
    </div>
  );
}
