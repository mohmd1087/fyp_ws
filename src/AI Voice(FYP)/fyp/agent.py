import asyncio
import json
import os
import time
import uuid
from difflib import get_close_matches
from typing import Any, Dict, List, Optional

import httpx
from dotenv import load_dotenv

from livekit import agents
from livekit.agents import Agent, AgentServer, AgentSession, RunContext, function_tool, room_io
from livekit.plugins import google, noise_cancellation

# Load env vars from .env.local (LIVEKIT_... and GOOGLE_API_KEY)
load_dotenv(".env.local")

# Agent behavior mode, set by run_on_at_table.py when spawning:
#   "order"    — fresh guest, greet and take an order (default)
#   "followup" — food was just delivered, ask if they need anything else
AGENT_MODE = os.environ.get("AGENT_MODE", "order").lower()

# In follow-up mode the supervisor injects the delivered order's id so the
# agent can append items to the same order instead of creating a new one.
FOLLOWUP_ORDER_ID = os.environ.get("FOLLOWUP_ORDER_ID") or None

# Pre-warm gate. When the supervisor spawns the agent during
# NAVIGATING_TO_TABLE it sets AGENT_ARMED=0 and AGENT_ARM_FILE to a temp path.
# The agent polls for that file's existence; the supervisor creates it when
# the robot reaches AT_TABLE. Avoids any signal API (which requires main thread).
AGENT_ARMED = os.environ.get("AGENT_ARMED", "1") != "0"
AGENT_ARM_FILE = os.environ.get("AGENT_ARM_FILE", "")

RESTAURANT_INFO = {
    "name": "Saffron Garden",
    "location": "Main Boulevard, Lahore",
    "hours": {
        "mon_thu": "12:00 PM – 11:00 PM",
        "fri_sat": "12:00 PM – 12:00 AM",
        "sun": "12:00 PM – 10:30 PM",
    },
    "cuisine": ["Pakistani", "BBQ", "Continental"],
    "policies": {
        "allergens": "Please tell me about allergies (nuts, dairy, gluten). We can guide you.",
        "spice_levels": "Mild / Medium / Spicy",
        "customization": "We can adjust spice and remove onions/garlic on request.",
    },
}

MENU: Dict[str, Dict[str, Any]] = {
    "starters": {
        "chicken_corn_soup": {
            "name": "Chicken Corn Soup",
            "price": 450,
            "description": "Classic chicken corn soup with mild seasoning.",
            "tags": ["popular"],
            "allergens": ["egg (optional)"],
        },
        "dynamite_shrimp": {
            "name": "Dynamite Shrimp",
            "price": 1190,
            "description": "Crispy shrimp tossed in a creamy spicy sauce.",
            "tags": ["spicy"],
            "allergens": ["shellfish", "dairy"],
        },
    },
    "mains": {
        "chicken_karahi_half": {
            "name": "Chicken Karahi (Half)",
            "price": 1850,
            "description": "Traditional karahi with fresh tomatoes, ginger, and green chilies.",
            "tags": ["signature", "spicy_optional"],
            "allergens": [],
        },
        "beef_burger": {
            "name": "Beef Burger",
            "price": 990,
            "description": "Grilled beef patty with cheese, lettuce, and house sauce.",
            "tags": ["kids_friendly_optional"],
            "allergens": ["gluten", "dairy"],
        },
        "alfredo_pasta": {
            "name": "Creamy Alfredo Pasta",
            "price": 1090,
            "description": "Creamy white sauce pasta with garlic and parmesan.",
            "tags": ["mild"],
            "allergens": ["dairy", "gluten"],
        },
    },
    "bbq": {
        "chicken_tikka": {
            "name": "Chicken Tikka",
            "price": 720,
            "description": "Charcoal grilled chicken tikka (1 pc).",
            "tags": ["bbq", "spicy_optional"],
            "allergens": [],
        },
        "seekh_kebab": {
            "name": "Seekh Kebab",
            "price": 690,
            "description": "Juicy minced meat kebabs (2 pcs).",
            "tags": ["bbq"],
            "allergens": [],
        },
    },
    "drinks": {
        "mint_margarita": {
            "name": "Mint Margarita",
            "price": 390,
            "description": "Refreshing mint lemon drink.",
            "tags": ["non_alcoholic"],
            "allergens": [],
        },
        "cold_coffee": {
            "name": "Cold Coffee",
            "price": 520,
            "description": "Iced coffee with milk and chocolate notes.",
            "tags": ["popular"],
            "allergens": ["dairy"],
        },
    },
    "desserts": {
        "kunafa": {
            "name": "Kunafa",
            "price": 890,
            "description": "Warm kunafa with cheese and sweet syrup.",
            "tags": ["popular"],
            "allergens": ["dairy", "gluten"],
        }
    },
}


def _flatten_menu() -> Dict[str, Dict[str, Any]]:
    flat: Dict[str, Dict[str, Any]] = {}
    for category, items in MENU.items():
        for item_id, item in items.items():
            flat[item_id] = {**item, "category": category, "item_id": item_id}
    return flat


FLAT_MENU = _flatten_menu()


def _calc_totals(items: List[Dict[str, Any]]) -> Dict[str, Any]:
    subtotal = 0
    for it in items:
        item_id = it["item_id"]
        qty = int(it.get("qty", 1))
        price = int(FLAT_MENU[item_id]["price"])
        subtotal += price * qty

    service_rate = 0.10
    service_charge = int(round(subtotal * service_rate))
    total = subtotal + service_charge

    return {
        "subtotal": subtotal,
        "service_charge": service_charge,
        "service_rate": service_rate,
        "total": total,
        "currency": "PKR",
    }


def _none_if_string_none(x: Any) -> Any:
    if x is None:
        return None
    if isinstance(x, str) and x.strip().lower() in {"none", "null", ""}:
        return None
    return x


def _normalize_order_type(s: Any) -> str:
    if not isinstance(s, str):
        return "dine_in"
    s2 = s.strip().lower().replace("-", "_").replace(" ", "_")
    # common variants from models:
    mapping = {
        "dinein": "dine_in",
        "dine_in": "dine_in",
        "dine": "dine_in",
        "dine-in": "dine_in",
        "takeaway": "takeaway",
        "take_away": "takeaway",
        "pickup": "takeaway",
        "delivery": "delivery",
    }
    return mapping.get(s2, "dine_in")


def _normalize_spice_level(s: Any) -> str:
    if not isinstance(s, str):
        return "not_specified"
    s2 = s.strip().lower()
    if s2 in {"mild", "medium", "spicy"}:
        return s2
    if s2 in {"none", "null", "", "not_specified", "not specified"}:
        return "not_specified"
    return "not_specified"


def _normalize_items(
    items_in: List[Any],
) -> tuple[List[Dict[str, Any]], Optional[Dict[str, Any]]]:
    """Turn the raw `items` field from a tool call into enriched line-items.

    Accepts either strings (menu item names) or objects ({item_id, qty,
    modifications}). Returns (enriched_items, error_or_None).
    On error returns ([], {"error": ...}) so callers can propagate directly.
    Shared between finalize_order (new order) and append_to_order (follow-up).
    """
    clean_items: List[Dict[str, Any]] = []
    for it in items_in:
        if isinstance(it, str):
            item_id = _find_item_id_by_name(it)
            if not item_id:
                return [], {
                    "error": f"'{it}' is not on our menu. Please choose from the menu items.",
                    "hint_top_items": [
                        {"item_id": k, "name": v["name"]}
                        for k, v in list(FLAT_MENU.items())[:6]
                    ],
                }
            clean_items.append({"item_id": item_id, "qty": 1, "modifications": None})
        elif isinstance(it, dict):
            # Gemini occasionally emits keys wrapped in extra quotes.
            it = {k.strip("'\""): v for k, v in it.items()}
            item_id = str(it.get("item_id", "")).strip()
            if item_id not in FLAT_MENU:
                return [], {
                    "error": f"Invalid item_id '{item_id}'. Use get_menu to pick valid items."
                }
            qty = int(it.get("qty", 1) or 1)
            if qty < 1:
                return [], {"error": f"Quantity must be >= 1 for item_id '{item_id}'."}
            mods = _none_if_string_none(it.get("modifications"))
            clean_items.append({"item_id": item_id, "qty": qty, "modifications": mods})
        else:
            return [], {"error": "Invalid items format. Items must be objects or strings."}

    enriched: List[Dict[str, Any]] = []
    for it in clean_items:
        item_id = it["item_id"]
        qty = int(it["qty"])
        unit_price = int(FLAT_MENU[item_id]["price"])
        enriched.append(
            {
                "item_id": item_id,
                "name": FLAT_MENU[item_id]["name"],
                "unit_price": unit_price,
                "qty": qty,
                "modifications": it.get("modifications"),
                "allergens": FLAT_MENU[item_id].get("allergens", []),
                "category": FLAT_MENU[item_id]["category"],
                "line_total": unit_price * qty,
            }
        )
    return enriched, None


def _find_item_id_by_name(name: str) -> Optional[str]:
    # Exact + fuzzy match against menu names
    name_l = name.strip().lower()
    name_to_id = {v["name"].lower(): k for k, v in FLAT_MENU.items()}
    if name_l in name_to_id:
        return name_to_id[name_l]

    close = get_close_matches(name_l, list(name_to_id.keys()), n=1, cutoff=0.72)
    if close:
        return name_to_id[close[0]]
    return None


# Shared item-array shape (accept objects or bare name strings)
_ITEMS_ARRAY_SCHEMA = {
    "type": "array",
    "items": {
        "anyOf": [
            {
                "type": "object",
                "properties": {
                    "item_id": {"type": "string"},
                    "qty": {"type": "integer", "minimum": 1, "default": 1},
                    "modifications": {"type": ["string", "null"]},
                },
                "required": ["item_id"],
            },
            {"type": "string"},
        ]
    },
}


APPEND_TO_ORDER_SCHEMA = {
    "name": "append_to_order",
    "description": (
        "FOLLOW-UP ONLY. Use this after the customer's food has been delivered "
        "and they ask for additional items. Appends items to the SAME order_id "
        "that was just delivered. Do NOT call finalize_order in follow-up mode.\n"
        "TOOL INPUT RULE:\n"
        "- items MUST be a list of objects like: "
        "[{\"item_id\":\"mint_margarita\",\"qty\":1,\"modifications\":null}]\n"
        "- Always use item_id from get_menu, never pass bare item names."
    ),
    "parameters": {
        "type": "object",
        "properties": {"items": _ITEMS_ARRAY_SCHEMA},
        "required": ["items"],
    },
}


# Raw schema tool to avoid Pydantic crashes when model sends wrong shapes
FINALIZE_ORDER_SCHEMA = {
    "name": "finalize_order",
    "description": (
        "CONFIRMATION RULE:\n"
        "- If the guest says anything that clearly confirms the order (e.g., 'confirm', 'yes', 'go ahead', "
        "'place the order', 'that's correct'), you MUST call finalize_order immediately.\n"
        "- Do NOT ask more questions at confirmation unless a required field is missing.\n"
        "TOOL INPUT RULE:\n"
        "- finalize_order.items MUST be a list of objects like: "
        "[{\"item_id\":\"chicken_karahi_half\",\"qty\":2,\"modifications\":\"medium spice\"}]\n"
        "- Never pass item names in finalize_order.items.\n"
        "- order_type MUST be one of: dine_in, takeaway, delivery.\n"
        "- spice_level MUST be one of: mild, medium, spicy, not_specified.\n"
    ),
    "parameters": {
        "type": "object",
        "properties": {
            "customer_name": {"type": ["string", "null"]},
            "order_type": {"type": "string", "enum": ["dine_in", "takeaway", "delivery"]},
            "party_size": {"type": ["integer", "null"]},
            "table_number": {"type": ["string", "null"]},
            "phone": {"type": ["string", "null"]},
            "notes": {"type": ["string", "null"]},
            "allergies": {"type": ["string", "null"]},
            "spice_level": {"type": "string", "enum": ["mild", "medium", "spicy", "not_specified"]},
            "items": {
                "type": "array",
                "items": {
                    "anyOf": [
                        {
                            "type": "object",
                            "properties": {
                                "item_id": {"type": "string"},
                                "qty": {"type": "integer", "minimum": 1, "default": 1},
                                "modifications": {"type": ["string", "null"]},
                            },
                            "required": ["item_id"],
                        },
                        {"type": "string"},
                    ]
                },
            },
        },
        "required": ["order_type", "spice_level", "items"],
    },
}


ORDER_PROMPT = (
    "Your name is AMORA and you are a highly professional restaurant waiter taking voice orders.\n"
    "Style: warm, polite, efficient, and confident.\n"
    "OPENING: As soon as the session starts, greet the guest warmly as AMORA, a waiter at "
    "Saffron Garden. Do NOT wait for them to speak first — start the conversation yourself. "
    "Welcome them, ask if it's dine-in, takeaway, or delivery, ask about any allergies, "
    "and then ask what they'd like to order.\n"
    "Goals:\n"
    "1) Answer questions about the restaurant and menu and answer in the language you are talked to in.\n"
    "2) Take the order conversationally and collect: order_type (dine_in/takeaway/delivery), "
    "party_size (if dine_in), table_number (if dine_in), allergies, spice_level.\n"
    "3) Never invent menu items. Only use items returned by get_menu/get_item_details.\n"
    "4) Before calling finalize_order, repeat the order (item names + qty + modifications) "
    "and ask for confirmation.\n"
    "5) When calling finalize_order, ALWAYS use item_id from the menu (not item names).\n"
    "If the user asks for something not on the menu, apologize and offer close alternatives.\n"
    "Keep spoken output short and natural.\n"
)

FOLLOWUP_PROMPT = (
    "Your name is AMORA, a waiter at Saffron Garden. The customer's food has "
    "just been delivered to their table.\n"
    "OPENING: Greet them briefly by name of the restaurant and ask if they need "
    "anything else — answer in the language they use.\n"
    "Decision:\n"
    "- If they say no / they're fine / they decline, call decline_followup() "
    "  immediately, then end the call warmly (e.g. 'Enjoy your meal!').\n"
    "- If they want something else, take the additional items using "
    "  get_menu/get_item_details, confirm them, then call append_to_order "
    "  with the new items (ALWAYS use item_id from the menu). This adds the "
    "  items to the SAME order that was just delivered.\n"
    "IMPORTANT: In follow-up mode you must NEVER call finalize_order. That "
    "would create a separate order. Use append_to_order for new items.\n"
    "Keep it short and friendly — they already have their food in front of them.\n"
)


class RestaurantWaiter(Agent):
    def __init__(self) -> None:
        instructions = FOLLOWUP_PROMPT if AGENT_MODE == "followup" else ORDER_PROMPT
        super().__init__(instructions=instructions)

    @function_tool(name="get_restaurant_info", description="Get restaurant location, hours, cuisine, and policies.")
    async def get_restaurant_info(self, context: RunContext) -> Dict[str, Any]:
        return RESTAURANT_INFO

    @function_tool(
        name="get_menu",
        description=(
            "Get the menu. By default returns only item_id, name, price, and category "
            "for every item (a compact summary). "
            "Optional: provide 'category' to restrict to one category. "
            "For full details (description, allergens, tags) of a specific item, call get_item_details."
        ),
    )
    async def get_menu(
        self, context: RunContext, category: Optional[str] = None
    ) -> Dict[str, Any]:
        def _summary(item_id: str, item: Dict[str, Any], cat: str) -> Dict[str, Any]:
            return {
                "item_id": item_id,
                "name": item["name"],
                "price": item["price"],
                "category": cat,
            }

        if category:
            key = category.strip().lower()
            if key not in MENU:
                return {"error": f"Unknown category '{category}'. Valid: {list(MENU.keys())}"}
            return {"items": [_summary(iid, it, key) for iid, it in MENU[key].items()]}

        items = [
            _summary(iid, it, cat)
            for cat, cat_items in MENU.items()
            for iid, it in cat_items.items()
        ]
        return {"items": items, "categories": list(MENU.keys())}

    @function_tool(
        name="get_item_details",
        description="Get details for a specific menu item by item_id (name, price, description, allergens, tags).",
    )
    async def get_item_details(self, context: RunContext, item_id: str) -> Dict[str, Any]:
        item_id = item_id.strip()
        if item_id not in FLAT_MENU:
            return {"error": f"Unknown item_id '{item_id}'. Ask for get_menu and choose a valid item_id."}
        return FLAT_MENU[item_id]

    async def _notify_orchestrator_order_complete(self, context: RunContext, order_id: str) -> None:
        """POST to the orchestrator's /order_complete endpoint so the robot can
        leave AT_TABLE and navigate home. Fail-soft: any failure is logged and
        swallowed — the voice agent must never crash due to orchestrator issues."""
        base_url = os.getenv("ORCHESTRATOR_CALLBACK_URL")
        if not base_url:
            print("[orchestrator] ORCHESTRATOR_CALLBACK_URL not set, skipping release signal.")
            return

        room_name = ""
        try:
            room_name = context.session.room.name or ""
        except Exception as exc:
            print(f"[orchestrator] Could not resolve room name: {exc}")

        payload = {"room_name": room_name, "order_id": order_id}
        try:
            async with httpx.AsyncClient(timeout=5.0) as client:
                resp = await client.post(f"{base_url}/order_complete", json=payload)
                if 200 <= resp.status_code < 300:
                    print(f"[orchestrator] Released AT_TABLE for room '{room_name}' (order {order_id})")
                else:
                    print(f"[orchestrator] /order_complete failed: {resp.status_code} {resp.text[:200]}")
        except Exception as exc:
            print(f"[orchestrator] Could not reach orchestrator: {exc}")

    @function_tool(
        name="decline_followup",
        description=(
            "Call this when the customer confirms they do NOT need anything "
            "else after their food delivery. This ends the voice session and "
            "sends the robot back to its home position. Use only in follow-up "
            "mode, after the food has already been delivered."
        ),
    )
    async def decline_followup(self, context: RunContext) -> Dict[str, Any]:
        base_url = os.getenv("ORCHESTRATOR_CALLBACK_URL")
        room_name = ""
        try:
            room_name = context.session.room.name or ""
        except Exception as exc:
            print(f"[orchestrator] Could not resolve room name: {exc}")

        if base_url:
            try:
                async with httpx.AsyncClient(timeout=5.0) as client:
                    resp = await client.post(
                        f"{base_url}/decline_followup",
                        json={"room_name": room_name},
                    )
                    if 200 <= resp.status_code < 300:
                        print(f"[orchestrator] Declined follow-up for room '{room_name}'")
                    else:
                        print(f"[orchestrator] /decline_followup failed: {resp.status_code} {resp.text[:200]}")
            except Exception as exc:
                print(f"[orchestrator] Could not reach orchestrator: {exc}")
        else:
            print("[orchestrator] ORCHESTRATOR_CALLBACK_URL not set, skipping decline signal.")

        return {"ok": True, "message": "Have a lovely meal!"}

    @function_tool(raw_schema=FINALIZE_ORDER_SCHEMA)
    async def finalize_order(self, raw_arguments: Dict[str, Any], context: RunContext) -> Dict[str, Any]:
        # Normalize top-level fields
        customer_name = _none_if_string_none(raw_arguments.get("customer_name"))
        order_type = _normalize_order_type(raw_arguments.get("order_type"))
        party_size = raw_arguments.get("party_size")
        table_number = _none_if_string_none(raw_arguments.get("table_number"))
        phone = _none_if_string_none(raw_arguments.get("phone"))
        notes = _none_if_string_none(raw_arguments.get("notes"))
        allergies = _none_if_string_none(raw_arguments.get("allergies"))
        spice_level = _normalize_spice_level(raw_arguments.get("spice_level"))

        enriched_items, err = _normalize_items(raw_arguments.get("items") or [])
        if err:
            return err

        totals = _calc_totals(enriched_items)

        order_json = {
            "order_id": f"LK-{uuid.uuid4().hex[:10].upper()}",
            "created_at_unix": int(time.time()),
            "restaurant": {"name": RESTAURANT_INFO["name"], "location": RESTAURANT_INFO["location"]},
            "customer": {"name": customer_name, "phone": phone},
            "order_type": order_type,
            "dine_in": {"party_size": party_size, "table_number": table_number} if order_type == "dine_in" else None,
            "preferences": {"spice_level": spice_level, "allergies": allergies, "notes": notes},
            "items": enriched_items,
            "totals": totals,
            "status": "confirmed",
        }

        # Store full JSON in userdata (not available in console mode, so guard it)
        try:
            context.userdata["last_order"] = order_json
        except (ValueError, AttributeError):
            pass
        # ALSO: print and save to disk so you can see it in console demos
        pretty = json.dumps(order_json, indent=2, ensure_ascii=False)
        print("\n===== FINAL ORDER JSON =====\n" + pretty + "\n============================\n")

        with open("last_order.json", "w", encoding="utf-8") as f:
            f.write(pretty)

        # POST to dashboard API (if configured)
        dashboard_url = os.getenv("DASHBOARD_API_URL")
        agent_key = os.getenv("DASHBOARD_AGENT_KEY")
        if dashboard_url and agent_key:
            try:
                async with httpx.AsyncClient(timeout=5.0) as client:
                    resp = await client.post(
                        f"{dashboard_url}/api/orders",
                        json=order_json,
                        headers={"X-Agent-Key": agent_key},
                    )
                    if resp.status_code == 201:
                        print(f"[dashboard] Order {order_json['order_id']} sent OK")
                    else:
                        print(f"[dashboard] POST failed: {resp.status_code} {resp.text[:200]}")
            except Exception as exc:
                # Never crash the voice agent due to dashboard failure
                print(f"[dashboard] Could not reach dashboard: {exc}")
        else:
            print("[dashboard] DASHBOARD_API_URL or DASHBOARD_AGENT_KEY not set, skipping.")

        # Release the robot from AT_TABLE so it navigates home. The orchestrator
        # HTTP endpoint lives on the robot at ORCHESTRATOR_CALLBACK_URL (e.g.
        # http://<robot-ip>:5050).
        await self._notify_orchestrator_order_complete(context, order_json["order_id"])

        # Keep tool response SMALL (reduces chance of 1011 around tool responses)
        return {"ok": True, "order_id": order_json["order_id"], "total": totals["total"], "currency": totals["currency"]}

    @function_tool(raw_schema=APPEND_TO_ORDER_SCHEMA)
    async def append_to_order(
        self, raw_arguments: Dict[str, Any], context: RunContext
    ) -> Dict[str, Any]:
        """Follow-up flow: add items to the just-delivered order (same order_id)."""
        order_id = FOLLOWUP_ORDER_ID
        if not order_id:
            return {
                "error": (
                    "No order_id available in follow-up context. I cannot "
                    "append without the original order reference."
                )
            }

        enriched_items, err = _normalize_items(raw_arguments.get("items") or [])
        if err:
            return err
        if not enriched_items:
            return {"error": "No items to append."}

        pretty = json.dumps(enriched_items, indent=2, ensure_ascii=False)
        print(
            f"\n===== APPEND TO ORDER {order_id} =====\n" + pretty +
            "\n========================================\n"
        )

        dashboard_url = os.getenv("DASHBOARD_API_URL")
        agent_key = os.getenv("DASHBOARD_AGENT_KEY")
        if dashboard_url and agent_key:
            try:
                async with httpx.AsyncClient(timeout=5.0) as client:
                    resp = await client.post(
                        f"{dashboard_url}/api/orders/{order_id}/append-items",
                        json={"items": enriched_items},
                        headers={"X-Agent-Key": agent_key},
                    )
                    if 200 <= resp.status_code < 300:
                        print(f"[dashboard] Appended {len(enriched_items)} item(s) to {order_id}")
                    else:
                        print(
                            f"[dashboard] append failed: {resp.status_code} "
                            f"{resp.text[:200]}"
                        )
            except Exception as exc:
                print(f"[dashboard] append unreachable: {exc}")
        else:
            print("[dashboard] DASHBOARD_API_URL or DASHBOARD_AGENT_KEY not set, skipping append.")

        # Release the robot so it navigates home, same as finalize_order.
        await self._notify_orchestrator_order_complete(context, order_id)

        return {"ok": True, "order_id": order_id, "appended": len(enriched_items)}


server = AgentServer()


@server.rtc_session()
async def waiter_agent(ctx: agents.JobContext):
    # Heavy LLM construction happens here — this is the part we want to finish
    # while the robot is still driving. After this line the agent is ready to
    # talk; we just need to wait for the arm file before actually starting.
    session = AgentSession(
        llm=google.beta.realtime.RealtimeModel(
            model="gemini-3.1-flash-live-preview",
            voice="Puck",
            temperature=0.4,
            # proactivity=True,  # optional
            # enable_affective_dialog=True,  # optional
        )
    )

    # Pre-warm gate: poll for sentinel file written by the supervisor.
    if not AGENT_ARMED and AGENT_ARM_FILE:
        print(f"[agent] Pre-warmed. Polling for arm file: {AGENT_ARM_FILE}")
        while not os.path.exists(AGENT_ARM_FILE):
            await asyncio.sleep(0.2)
        print("[agent] Arm file detected — starting session.")
        try:
            os.remove(AGENT_ARM_FILE)
        except OSError:
            pass

    await session.start(
        room=ctx.room,
        agent=RestaurantWaiter(),
        room_options=room_io.RoomOptions(
            audio_input=room_io.AudioInputOptions(noise_cancellation=lambda _: noise_cancellation.BVC())
        ),
    )

    # Ensure the job connects (matches recommended entrypoint flow used in recipes)
    await ctx.connect()

    # Wait for the customer (local_participant) to actually join the room before
    # greeting — otherwise the agent talks into an empty room and the user never
    # hears the opening line.
    async def _wait_for_participant(timeout: float = 120.0) -> bool:
        if ctx.room.remote_participants:
            return True
        joined = asyncio.Event()

        def _on_joined(_p):
            joined.set()

        ctx.room.on("participant_connected", _on_joined)
        try:
            await asyncio.wait_for(joined.wait(), timeout=timeout)
            return True
        except asyncio.TimeoutError:
            return False
        finally:
            ctx.room.off("participant_connected", _on_joined)

    await _wait_for_participant()
    # Small settle delay so the participant's audio subscription is fully wired
    # before the agent speaks (otherwise the first word can get clipped).
    await asyncio.sleep(0.8)
    # Greeting is now baked into the system instructions (OPENING section) —
    # gemini-3.1-flash-live-preview does not support generate_reply() mid-session.
    # The model will greet on its own as soon as it starts speaking.


if __name__ == "__main__":
    agents.cli.run_app(server)