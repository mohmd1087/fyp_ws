import { prisma } from "@/lib/prisma";
import { DashboardShell } from "@/components/dashboard-shell";

export const dynamic = "force-dynamic";

export default async function DashboardPage() {
  const orders = await prisma.order.findMany({
    orderBy: { createdAt: "desc" },
    take: 100,
  });

  // Serialize for client: convert dates to strings and cast Json fields
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const serialized = orders.map((o) => ({
    id: o.id,
    tableNumber: o.tableNumber,
    orderType: o.orderType,
    status: o.status as "PENDING" | "COMPLETED",
    customer: o.customer as { name?: string | null; phone?: string | null },
    dineIn: o.dineIn as {
      party_size?: number | null;
      table_number?: string | null;
    } | null,
    preferences: o.preferences as {
      spice_level?: string;
      allergies?: string | null;
      notes?: string | null;
    },
    items: o.items as Array<{
      name: string;
      qty: number;
      modifications?: string | null;
      line_total: number;
    }>,
    totals: o.totals as { total: number; currency: string },
    createdAt: o.createdAt.toISOString(),
  }));

  return <DashboardShell initialOrders={serialized} />;
}
