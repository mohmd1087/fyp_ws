import { NextRequest, NextResponse } from "next/server";
import { z } from "zod";
import bcrypt from "bcryptjs";
import { prisma } from "@/lib/prisma";
import { signToken, setAuthCookie } from "@/lib/auth";
import { rateLimit } from "@/lib/rate-limit";

const LoginSchema = z.object({
  email: z.string().email(),
  password: z.string().min(1),
});

export async function POST(req: NextRequest) {
  // Rate limit: 10 attempts per minute per IP
  const ip = req.headers.get("x-forwarded-for") ?? "unknown";
  if (rateLimit(`login:${ip}`, 10, 60)) {
    return NextResponse.json({ error: "Too many requests" }, { status: 429 });
  }

  const body = await req.json().catch(() => null);
  const parsed = LoginSchema.safeParse(body);
  if (!parsed.success) {
    return NextResponse.json({ error: "Invalid input" }, { status: 400 });
  }

  const { email, password } = parsed.data;

  const staff = await prisma.staff.findUnique({ where: { email } });

  // Constant-time comparison: always run bcrypt even if user not found
  const dummyHash =
    "$2a$12$000000000000000000000000000000000000000000000000000000";
  const valid = await bcrypt.compare(
    password,
    staff?.passwordHash ?? dummyHash
  );

  if (!staff || !valid) {
    return NextResponse.json({ error: "Invalid credentials" }, { status: 401 });
  }

  const token = await signToken({
    sub: staff.id,
    email: staff.email,
    role: staff.role,
  });

  const res = NextResponse.json({
    ok: true,
    name: staff.name,
    role: staff.role,
  });
  setAuthCookie(res, token);
  return res;
}
