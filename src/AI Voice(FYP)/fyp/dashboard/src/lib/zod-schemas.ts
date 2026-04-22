import { z } from "zod";

// --- Sub-schemas matching the voice agent JSON structure ---

const RestaurantSchema = z.object({
  name: z.string(),
  location: z.string(),
});

const CustomerSchema = z.object({
  name: z.string().nullable(),
  phone: z.string().nullable(),
});

const DineInSchema = z
  .object({
    party_size: z.number().int().nullable(),
    table_number: z.string().nullable(),
  })
  .nullable();

const PreferencesSchema = z.object({
  spice_level: z.enum(["mild", "medium", "spicy", "not_specified"]),
  allergies: z.string().nullable(),
  notes: z.string().nullable(),
});

const OrderItemSchema = z.object({
  item_id: z.string(),
  name: z.string(),
  unit_price: z.number(),
  qty: z.number().int().min(1),
  modifications: z.string().nullable(),
  allergens: z.array(z.string()),
  category: z.string(),
  line_total: z.number(),
});

const TotalsSchema = z.object({
  subtotal: z.number(),
  service_charge: z.number(),
  service_rate: z.number(),
  total: z.number(),
  currency: z.string(),
});

// --- Full agent order payload (POST /api/orders) ---

export const CreateOrderSchema = z.object({
  order_id: z.string().min(1),
  created_at_unix: z.number().int(),
  restaurant: RestaurantSchema,
  customer: CustomerSchema,
  order_type: z.enum(["dine_in", "takeaway", "delivery"]),
  dine_in: DineInSchema,
  preferences: PreferencesSchema,
  items: z.array(OrderItemSchema).min(1),
  totals: TotalsSchema,
  status: z.string(), // "confirmed" from agent
});

export type CreateOrderInput = z.infer<typeof CreateOrderSchema>;

// --- Query params for GET /api/orders ---

export const GetOrdersQuerySchema = z.object({
  status: z.enum(["PENDING", "COMPLETED"]).optional(),
  search: z.string().max(100).optional(),
  limit: z.coerce.number().int().min(1).max(100).default(50),
});

// --- PATCH /api/orders/[id] ---

export const UpdateOrderSchema = z.object({
  status: z.enum(["PENDING", "COMPLETED"]),
});

// --- POST /api/orders/[id]/append-items (follow-up append flow) ---

export const AppendItemsSchema = z.object({
  items: z.array(OrderItemSchema).min(1),
});

export type AppendItemsInput = z.infer<typeof AppendItemsSchema>;
