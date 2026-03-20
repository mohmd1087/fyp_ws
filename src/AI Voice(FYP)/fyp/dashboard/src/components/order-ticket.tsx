"use client";

import {
  Card,
  CardContent,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
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

const STATUS_COLORS: Record<string, string> = {
  PENDING: "bg-yellow-100 text-yellow-800",
  COMPLETED: "bg-green-100 text-green-800",
};

interface Props {
  order: OrderData;
  onStatusChange: (id: string, status: OrderStatus) => void;
}

export function OrderTicket({ order, onStatusChange }: Props) {
  const { customer, dineIn, preferences: prefs, items, totals } = order;
  const time = new Date(order.createdAt).toLocaleTimeString([], {
    hour: "2-digit",
    minute: "2-digit",
  });

  return (
    <Card className="flex flex-col">
      <CardHeader className="pb-2">
        <div className="flex items-start justify-between gap-2">
          <CardTitle className="text-sm font-mono">{order.id}</CardTitle>
          <span
            className={`text-xs font-medium px-2 py-0.5 rounded-full whitespace-nowrap ${STATUS_COLORS[order.status] ?? "bg-gray-100 text-gray-800"}`}
          >
            {order.status}
          </span>
        </div>
        <div className="text-sm text-muted-foreground space-y-0.5">
          <p>
            {customer?.name ?? "Guest"}
            {customer?.phone ? ` \u00B7 ${customer.phone}` : ""}
          </p>
          <p className="capitalize">
            {order.orderType.replace("_", " ")}
            {dineIn?.table_number
              ? ` \u00B7 Table ${dineIn.table_number}`
              : ""}
            {dineIn?.party_size ? ` \u00B7 ${dineIn.party_size} pax` : ""}
          </p>
          <p className="text-xs">{time}</p>
          {prefs?.allergies && (
            <p className="text-red-600 font-medium text-xs">
              Allergy: {prefs.allergies}
            </p>
          )}
          {prefs?.spice_level && prefs.spice_level !== "not_specified" && (
            <p className="text-xs capitalize">
              Spice: {prefs.spice_level}
            </p>
          )}
        </div>
      </CardHeader>
      <CardContent className="flex-1 space-y-3">
        <ul className="text-sm space-y-1">
          {items.map((item, i) => (
            <li key={i} className="flex justify-between">
              <span>
                {item.qty}x {item.name}
                {item.modifications && (
                  <span className="text-muted-foreground text-xs ml-1">
                    ({item.modifications})
                  </span>
                )}
              </span>
              <span className="font-medium">
                {item.line_total.toLocaleString()}
              </span>
            </li>
          ))}
        </ul>
        <div className="border-t pt-2 flex justify-between font-semibold text-sm">
          <span>Total</span>
          <span>
            {totals.currency} {totals.total.toLocaleString()}
          </span>
        </div>
        {prefs?.notes && (
          <p className="text-xs text-muted-foreground italic">
            Note: {prefs.notes}
          </p>
        )}

        {/* Action buttons */}
        <div className="flex gap-2">
          {order.status === "PENDING" && (
            <Button
              size="sm"
              className="w-full"
              onClick={() => onStatusChange(order.id, "COMPLETED")}
            >
              Mark Complete
            </Button>
          )}
          {order.status === "COMPLETED" && (
            <Button
              size="sm"
              variant="outline"
              className="w-full"
              onClick={() => onStatusChange(order.id, "PENDING")}
            >
              Reopen
            </Button>
          )}
        </div>
      </CardContent>
    </Card>
  );
}
