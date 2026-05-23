import time
from .parking_config import SLOT_FREE_FRAMES, DEBOUNCE_DURATION

class ParkingSlotManager:
    def __init__(self):
        self.occupancy_map = {}
        self.last_detection_time = {}
        self.current_slot = 1

        self.selected_slot = None
        self.selected_side = None

        self._free_frame_count = 0   # consecutive frames with no car on target side
        self._occupied_slots = set() # slots seen occupied during scan

    def reset(self):
        self.occupancy_map = {}
        self.last_detection_time = {}
        self.current_slot = 1
        self.selected_slot = None
        self.selected_side = None
        self._free_frame_count = 0
        self._occupied_slots = set()

    # ── Primary: vision-based free-slot detection ─────────────────────────
    def update_by_vision(self, car_detections):
        """
        Call every frame while in SCANNING state.
        Returns True the moment a free slot is confirmed.

        Logic:
          - If a car is visible on the RIGHT side  → reset counter (occupied)
          - If no car visible on RIGHT for SLOT_FREE_FRAMES frames → FREE slot found
          - Defaults to RIGHT side; flips to LEFT if right keeps being occupied
            and a car-free gap appears on the left instead.
        """
        right_cars = [c for c in car_detections if c.get("side") == "right"]
        left_cars  = [c for c in car_detections if c.get("side") == "left"]

        # Mark any visible car positions as occupied in the occupancy map
        now = time.time()
        if right_cars:
            slot_key = self.current_slot
            if slot_key not in self.occupancy_map:
                self.occupancy_map[slot_key] = {"left": False, "right": False}
                self.last_detection_time[slot_key] = {"left": 0.0, "right": 0.0}
            if now - self.last_detection_time[slot_key]["right"] > DEBOUNCE_DURATION:
                self.occupancy_map[slot_key]["right"] = True
                self.last_detection_time[slot_key]["right"] = now
            self._occupied_slots.add(self.current_slot)

        # Decide target side: prefer RIGHT; use LEFT if right was always occupied
        # and a clear gap appears on the left.
        if not right_cars:
            # Right side looks clear
            self._free_frame_count += 1
            target_side = "right"
        elif not left_cars and right_cars:
            # Right is occupied but left is clear
            self._free_frame_count += 1
            target_side = "left"
        else:
            # Both sides have cars → definitely occupied, reset
            self._free_frame_count = 0
            self.current_slot += 1
            return False

        if self._free_frame_count >= SLOT_FREE_FRAMES:
            self.selected_side = target_side
            self.selected_slot = max(1, self.current_slot)
            self._free_frame_count = 0
            print(f"[Parking] Free slot found → side={target_side.upper()}, slot≈{self.selected_slot}")
            return True

        return False
