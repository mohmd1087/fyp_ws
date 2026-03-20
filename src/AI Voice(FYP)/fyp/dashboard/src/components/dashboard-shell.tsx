"use client";

import { useState, useCallback } from "react";
import { toast } from "sonner";
import { useRealtimeChannel } from "@/hooks/use-realtime";
import { OrderBoard } from "./order-board";
import { RobotDispatch } from "./robot-dispatch";
import { Button } from "@/components/ui/button";

type OrderStatus = "PENDING" | "COMPLETED";

interface OrderData {
  id: string;
  tableNumber: string | null;
  orderType: string;
  status: OrderStatus;
  customer: { name?: string | null; phone?: string | null };
  dineIn: { party_size?: number | null; table_number?: string | null } | null;
  preferences: {
    spice_level?: string;
    allergies?: string | null;
    notes?: string | null;
  };
  items: Array<{
    name: string;
    qty: number;
    modifications?: string | null;
    line_total: number;
  }>;
  totals: { total: number; currency: string };
  createdAt: string;
}

interface Props {
  initialOrders: OrderData[];
}

export function DashboardShell({ initialOrders }: Props) {
  const [orders, setOrders] = useState<OrderData[]>(initialOrders);

  const handleOrderCreated = useCallback((data: unknown) => {
    const event = data as { id: string; customer: { name: string | null } };
    toast.success(
      `New order from ${event.customer?.name ?? "guest"} (#${event.id})`
    );
    // Fetch the full order and prepend to list
    fetch(`/api/orders/${event.id}`)
      .then((r) => r.json())
      .then(({ order }) => {
        if (order) {
          setOrders((prev) => [
            order,
            ...prev.filter((o) => o.id !== order.id),
          ]);
        }
      })
      .catch(() => {
        // Fallback: just refetch all
        fetch("/api/orders")
          .then((r) => r.json())
          .then(({ orders: all }) => {
            if (all) setOrders(all);
          });
      });
  }, []);

  const handleOrderUpdated = useCallback((data: unknown) => {
    const event = data as { id: string; status: OrderStatus };
    setOrders((prev) =>
      prev.map((o) =>
        o.id === event.id ? { ...o, status: event.status } : o
      )
    );
  }, []);

  useRealtimeChannel("orders", {
    "order.created": handleOrderCreated,
    "order.updated": handleOrderUpdated,
  });

  async function handleStatusChange(id: string, status: OrderStatus) {
    const res = await fetch(`/api/orders/${id}`, {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ status }),
    });

    if (!res.ok) {
      toast.error("Failed to update order status");
      return;
    }

    // Optimistic update (also gets confirmed via Pusher event)
    setOrders((prev) =>
      prev.map((o) => (o.id === id ? { ...o, status } : o))
    );
  }

  async function handleLogout() {
    await fetch("/api/auth/logout", { method: "POST" });
    window.location.href = "/login";
  }

  return (
    <div className="min-h-screen bg-gray-50">
      <header className="bg-white border-b px-6 py-4 flex items-center justify-between">
        <h1 className="text-xl font-semibold">
          Saffron Garden — Orders
        </h1>
        <div className="flex items-center gap-4">
          <RobotDispatch />
          <Button variant="outline" size="sm" onClick={handleLogout}>
            Logout
          </Button>
        </div>
      </header>
      <main className="p-6">
        <OrderBoard orders={orders} onStatusChange={handleStatusChange} />
      </main>
    </div>
  );
}
