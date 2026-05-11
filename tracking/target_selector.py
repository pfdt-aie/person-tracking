"""
target_selector.py — Stateless YOLO result parser + PersonRegistry bridge.

Extracts the target-selection responsibility from PersonGimbalTracker.
select() is called once per frame after inference and updates the shared
TrackerState with the current detection map, then returns the chosen target.
"""

from __future__ import annotations

import time
from typing import Optional

from tracking.person_registry import PersonRegistry
from tracking.target_detection import TargetDetection
from tracking.tracker_state import TrackerState


class TargetSelector:
    """Parse YOLO/ByteTrack results and choose the follow target.

    Args:
        registry: Shared PersonRegistry for persistent re-identification.
    """

    def __init__(self, registry: PersonRegistry) -> None:
        self._registry = registry

    def select(self, results, state: TrackerState) -> Optional[TargetDetection]:
        """Update state.detected_ids and return the chosen TargetDetection.

        Respects state.lock_id: if set, returns that persistent ID's detection
        (or None if not currently visible).  Otherwise returns the largest box.

        Side-effect: overwrites state.detected_ids every call.
        """
        now = time.time()

        if not results or len(results[0].boxes) == 0:
            state.detected_ids = {}
            self._registry.mark_inactive(set())
            return None

        boxes  = results[0].boxes
        bt_ids = boxes.id
        frame  = results[0].orig_img

        new_ids: dict[int, TargetDetection] = {}
        active_bt: set = set()

        for i in range(len(boxes)):
            c = boxes.xyxy[i].cpu().numpy()
            x1, y1, x2, y2 = float(c[0]), float(c[1]), float(c[2]), float(c[3])
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            conf   = float(boxes.conf[i].cpu().item())
            bt_id  = int(bt_ids[i].item()) if bt_ids is not None else i

            iy1, iy2 = max(0, int(y1)), min(frame.shape[0], int(y2))
            ix1, ix2 = max(0, int(x1)), min(frame.shape[1], int(x2))
            crop = frame[iy1:iy2, ix1:ix2]

            pid = self._registry.update(bt_id, crop, cx, cy, now)
            active_bt.add(bt_id)
            new_ids[pid] = TargetDetection(
                cx=cx, cy=cy, x1=x1, y1=y1, x2=x2, y2=y2,
                conf=conf, track_id=pid,
            )

        state.detected_ids = new_ids
        self._registry.mark_inactive(active_bt)

        if state.lock_id is not None:
            return new_ids.get(state.lock_id)

        best_pid, best_area = None, 0.0
        for pid, d in new_ids.items():
            if d.bbox_area > best_area:
                best_area, best_pid = d.bbox_area, pid
        return new_ids.get(best_pid) if best_pid is not None else None
