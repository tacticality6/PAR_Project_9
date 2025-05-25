# IDs for DICT_4X4_50 range from 0 to 49.

MARKER_MAP = {
    # --- Item Alpha ---
    42: {"type": "pickup",   "item_id": "item_alpha", "description": "Pickup Location for Alpha Package"},
    27: {"type": "delivery", "item_id": "item_alpha", "description": "Delivery Location for Alpha Package"},

    # --- Item Bravo ---
    18: {"type": "pickup",   "item_id": "item_bravo", "description": "Pickup Location for Bravo Package"},
    43: {"type": "delivery", "item_id": "item_bravo", "description": "Delivery Location for Bravo Package"},

    # --- Item Charlie ---
    12: {"type": "pickup",   "item_id": "item_charlie", "description": "Pickup Location for Charlie Package"},
    5: {"type": "delivery", "item_id": "item_charlie", "description": "Delivery Location for Charlie Package"},

    # --- Item Delta ---
    6: {"type": "pickup",   "item_id": "item_delta", "description": "Pickup Location for Delta Package"},
    7: {"type": "delivery", "item_id": "item_delta", "description": "Delivery Location for Delta Package"},

    # --- Item Echo ---
    8: {"type": "pickup",   "item_id": "item_echo", "description": "Pickup Location for Echo Package"},
    9: {"type": "delivery", "item_id": "item_echo", "description": "Delivery Location for Echo Package"},

    # --- Special Locations ---
    49: {"type": "home_base", "item_id": None, "description": "Robot's Home Base / Recharging Station"},

    # === Can add more items or special markers below using IDs 10-48 ===
    # For example

    # --- Item Foxtrot ---
    # 10: {"type": "pickup",   "item_id": "item_foxtrot", "description": "Pickup Location for Foxtrot Package"},
    # 11: {"type": "delivery", "item_id": "item_foxtrot", "description": "Delivery Location for Foxtrot Package"},

    # --- Item Golf ---
    # 12: {"type": "pickup",   "item_id": "item_golf", "description": "Pickup Location for Golf Package"},
    # 13: {"type": "delivery", "item_id": "item_golf", "description": "Delivery Location for Golf Package"},

    # Example of other special markers (if your task evolves):
    # 40: {"type": "waypoint", "item_id": None, "description": "General Waypoint A"},
    # 41: {"type": "restricted_area_entry", "item_id": None, "description": "Entry to Restricted Zone"},
}
    

if __name__ == '__main__':
    # This part is just for testing/printing the map if run this file directly
    print("Defined MARKER_MAP:")
    for marker_id, info in MARKER_MAP.items():
        print(f"  ID {marker_id:2d}: Type='{info['type']}', Item='{info['item_id']}', Desc='{info['description']}'")
    
    print(f"\nThis map uses {len(MARKER_MAP)} out of the 50 available markers in DICT_4X4_50 (IDs 0-49).")
    print("Ensure you print and use these specific Aruco marker IDs from the DICT_4X4_50 set.")