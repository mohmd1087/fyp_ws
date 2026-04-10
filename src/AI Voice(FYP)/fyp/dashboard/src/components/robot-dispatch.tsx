"use client";

import { useState, useCallback } from "react";
import { toast } from "sonner";
import { useRealtimeChannel } from "@/hooks/use-realtime";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";

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
  const [selectedTray, setSelectedTray] = useState<Record<string, string>>({
    "table-1": "1",
    "table-2": "1",
  });

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

  async function dispatchWithTray(tableId: string, tray: number) {
    setDispatching(true);
    try {
      const res = await fetch("/api/robot/dispatch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ table_id: tableId, tray }),
      });
      if (!res.ok) {
        const err = await res.json();
        toast.error(err.error ?? "Dispatch failed");
      } else {
        toast.success(`Robot dispatched to ${tableId} (Tray ${tray})`);
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
    <div className="flex flex-col gap-3">
      {/* Original dispatch buttons — no tray */}
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

      {/* Tray dispatch buttons — each paired with a tray selector */}
      <div className="flex items-center gap-3">
        {TABLES.map((t) => (
          <div key={`${t}-tray`} className="flex items-center gap-1.5">
            <Select
              value={selectedTray[t]}
              onValueChange={(v) =>
                setSelectedTray((prev) => ({ ...prev, [t]: v }))
              }
              disabled={busy || dispatching}
            >
              <SelectTrigger className="h-8 w-[90px]">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="1">Tray 1</SelectItem>
                <SelectItem value="2">Tray 2</SelectItem>
              </SelectContent>
            </Select>
            <Button
              variant="outline"
              size="sm"
              disabled={busy || dispatching}
              onClick={() => dispatchWithTray(t, Number(selectedTray[t]))}
            >
              {t}
            </Button>
          </div>
        ))}
      </div>
    </div>
  );
}
