"""
Marker-ID encoding for DICT_4X4_50 (IDs 0-49)
    • Destinations (drop-offs)  : 0 – 4
    • Pick-ups                  : 5 – 49
        pickup.dest_id = pickup_id % 5

Mapping table
-------------
Drop-off : Pick-ups
0 : 5,10,15,20,25,30,35,40,45
1 : 6,11,16,21,26,31,36,41,46
2 : 7,12,17,22,27,32,37,42,47
3 : 8,13,18,23,28,33,38,43,48
4 : 9,14,19,24,29,34,39,44,49
"""

DEST_IDS = set(range(0, 5))
PICKUP_IDS = set(range(5, 50))

DEST_TO_PICKUPS = {
    d: [d + 5 * k for k in range(1, 10) if d + 5 * k <= 49] for d in DEST_IDS
}
PICKUP_TO_DEST = {p: p % 5 for p in PICKUP_IDS}

MARKER_MAP = {
    **{d: {"type": "destination", "dest_id": d, "is_pickup": False} for d in DEST_IDS},
    **{p: {"type": "pickup",      "dest_id": PICKUP_TO_DEST[p], "is_pickup": True}
       for p in PICKUP_IDS},
}

# ─────── helper API ─────────────────────────────────────────
def classify(marker_id: int) -> str:
    """Return 'destination', 'pickup', or 'invalid'."""
    if marker_id in DEST_IDS:
        return "destination"
    if marker_id in PICKUP_IDS:
        return "pickup"
    return "invalid"

def dest_id(marker_id: int) -> int | None:
    """Return the drop-off’s ID for *any* legal marker, else None."""
    if marker_id in DEST_IDS:
        return marker_id
    if marker_id in PICKUP_IDS:
        return PICKUP_TO_DEST[marker_id]
    return None

def pickups_for_dest(dest: int) -> list[int]:
    """List every pick-up that belongs to *dest* (0-4)."""
    return DEST_TO_PICKUPS.get(dest, [])

# ─────── quick self-test ───────────────────────────────────
if __name__ == "__main__":
    for d in DEST_IDS:
        print(f"Dest {d}: pickups {DEST_TO_PICKUPS[d]}")
    print("OK  –  definitions consistent.")
