import { PrismaClient } from "@prisma/client";
import bcrypt from "bcryptjs";

const prisma = new PrismaClient();

async function main() {
  const hash = await bcrypt.hash("admin123", 12);
  await prisma.staff.upsert({
    where: { email: "admin@saffron.com" },
    update: {},
    create: {
      email: "admin@saffron.com",
      passwordHash: hash,
      name: "Admin",
      role: "ADMIN",
    },
  });
  console.log("Seeded admin user: admin@saffron.com / admin123");
}

main()
  .catch((e) => {
    console.error(e);
    process.exit(1);
  })
  .finally(() => prisma.$disconnect());
