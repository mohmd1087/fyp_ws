"use client";

import { useState, useMemo } from "react";
import {
  Tabs,
  TabsContent,
  TabsList,
  TabsTrigger,
} from "@/components/ui/tabs";
import { Input } from "@/components/ui/input";
import { OrderTicket } from "./order-ticket";

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
  orders: OrderData[];
  onStatusChange: (id: string, status: OrderStatus) => void;
}

export function OrderBoard({ orders, onStatusChange }: Props) {
  const [search, setSearch] = useState("");

  const filtered = useMemo(() => {
    const q = search.toLowerCase();
    if (!q) return orders;
    return orders.filter((o) => {
      const customerName = o.customer?.name ?? "";
      const tableNum = o.dineIn?.table_number ?? o.tableNumber ?? "";
      return (
        o.id.toLowerCase().includes(q) ||
        customerName.toLowerCase().includes(q) ||
        tableNum.toLowerCase().includes(q)
      );
    });
  }, [orders, search]);

  const pending = filtered.filter((o) => o.status === "PENDING");
  const completed = filtered.filter((o) => o.status === "COMPLETED");

  return (
    <div className="space-y-4">
      <Input
        placeholder="Search by order ID, customer name, or table number..."
        value={search}
        onChange={(e) => setSearch(e.target.value)}
        className="max-w-md"
      />
      <Tabs defaultValue="pending">
        <TabsList>
          <TabsTrigger value="pending">
            Pending ({pending.length})
          </TabsTrigger>
          <TabsTrigger value="completed">
            Completed ({completed.length})
          </TabsTrigger>
        </TabsList>
        <TabsContent value="pending">
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4 mt-4">
            {pending.map((o) => (
              <OrderTicket
                key={o.id}
                order={o}
                onStatusChange={onStatusChange}
              />
            ))}
            {pending.length === 0 && (
              <p className="text-muted-foreground col-span-full text-center py-10">
                No pending orders
              </p>
            )}
          </div>
        </TabsContent>
        <TabsContent value="completed">
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4 mt-4">
            {completed.map((o) => (
              <OrderTicket
                key={o.id}
                order={o}
                onStatusChange={onStatusChange}
              />
            ))}
            {completed.length === 0 && (
              <p className="text-muted-foreground col-span-full text-center py-10">
                No completed orders
              </p>
            )}
          </div>
        </TabsContent>
      </Tabs>
    </div>
  );
}
