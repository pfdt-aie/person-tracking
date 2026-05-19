"""
target_selector.py — Stateless YOLO result parser + PersonRegistry bridge.

Extracts the target-selection responsibility from PersonGimbalTracker.
select() is called once per frame after inference and updates the shared
TrackerState with the current detection map, then returns the chosen target.
"""

from __future__ import annotations

import time
from dataclasses import replace
from typing import Optional

import config as cfg
from tracking.person_registry import PersonRegistry
from tracking.target_detection import TargetDetection
from tracking.tracker_state import TrackerState


class TargetSelector:
    """Parse YOLO/ByteTrack results and choose the follow target.

    Args:
        registry: Shared PersonRegistry for persistent re-identification.
    """

    def __init__(
        self,
        registry: PersonRegistry,
        lock_grace_s: float = cfg.LOCK_TARGET_GRACE_S,
        reacquire_center_ratio: float = cfg.LOCK_REACQUIRE_CENTER_RATIO,
        strict_center_ratio: float = cfg.LOCK_REACQUIRE_STRICT_CENTER_RATIO,
        min_reacquire_iou: float = cfg.LOCK_REACQUIRE_MIN_IOU,
    ) -> None:
        self._registry = registry
        self._lock_grace_s = lock_grace_s
        self._reacquire_center_ratio = reacquire_center_ratio
        self._strict_center_ratio = strict_center_ratio
        self._min_reacquire_iou = min_reacquire_iou
        self._last_locked_id: Optional[int] = None
        self._last_locked_detection: Optional[TargetDetection] = None
        self._last_locked_t: float = 0.0
        self._fallback_track_seq = -1

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
            return self._locked_grace_target(state, now)

        boxes  = results[0].boxes
        bt_ids = boxes.id
        frame  = results[0].orig_img

        new_ids: dict[int, TargetDetection] = {}
        active_bt: set = set()
        assigned_pids: set[int] = set()

        for i in range(len(boxes)):
            c = boxes.xyxy[i].cpu().numpy()
            x1, y1, x2, y2 = float(c[0]), float(c[1]), float(c[2]), float(c[3])
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            conf   = float(boxes.conf[i].cpu().item())
            bt_id = int(bt_ids[i].item()) if bt_ids is not None else self._next_fallback_track_id()

            iy1, iy2 = max(0, int(y1)), min(frame.shape[0], int(y2))
            ix1, ix2 = max(0, int(x1)), min(frame.shape[1], int(x2))
            crop = frame[iy1:iy2, ix1:ix2]

            pid = self._registry.update(bt_id, crop, cx, cy, now, exclude_pids=assigned_pids)
            assigned_pids.add(pid)
            active_bt.add(bt_id)
            new_ids[pid] = TargetDetection(
                cx=cx, cy=cy, x1=x1, y1=y1, x2=x2, y2=y2,
                conf=conf, track_id=pid,
            )

        state.detected_ids = new_ids
        self._registry.mark_inactive(active_bt)

        if state.lock_id is not None:
            locked = new_ids.get(state.lock_id)
            if locked is not None:
                self._remember_locked(state.lock_id, locked, now)
                return locked

            reacquired = self._reacquire_near_last_lock(
                state.lock_id, new_ids, frame.shape[1], frame.shape[0]
            )
            if reacquired is not None:
                old_id = state.lock_id
                state.lock_id = reacquired.track_id
                if state.lock_id != old_id:
                    from utils import terminal as _t
                    _t.event(f"[Target] Lock remapped ID {old_id} → {state.lock_id} "
                             f"(ByteTrack re-ID — same person)")
                self._remember_locked(state.lock_id, reacquired, now)
                return reacquired

            return self._locked_grace_target(state, now)

        best_pid, best_area = None, 0.0
        for pid, d in new_ids.items():
            if d.bbox_area > best_area:
                best_area, best_pid = d.bbox_area, pid
        return new_ids.get(best_pid) if best_pid is not None else None

    def _remember_locked(self, lock_id: int, detection: TargetDetection, now: float) -> None:
        self._last_locked_id = lock_id
        self._last_locked_detection = detection
        self._last_locked_t = now

    def _next_fallback_track_id(self) -> int:
        bt_id = self._fallback_track_seq
        self._fallback_track_seq -= 1
        return bt_id

    def _locked_grace_target(self, state: TrackerState, now: float) -> Optional[TargetDetection]:
        if (
            state.lock_id is not None
            and self._last_locked_id == state.lock_id
            and self._last_locked_detection is not None
            and now - self._last_locked_t <= self._lock_grace_s
        ):
            return replace(self._last_locked_detection, is_fresh=False)
        return None

    def _reacquire_near_last_lock(
        self,
        lock_id: int,
        detections: dict[int, TargetDetection],
        frame_w: int,
        frame_h: int,
    ) -> Optional[TargetDetection]:
        if (
            self._last_locked_id != lock_id
            or self._last_locked_detection is None
            or not detections
        ):
            return None

        frame_diag = (frame_w * frame_w + frame_h * frame_h) ** 0.5
        max_dist = frame_diag * self._reacquire_center_ratio
        strict_dist = frame_diag * self._strict_center_ratio
        last = self._last_locked_detection
        best: Optional[TargetDetection] = None
        best_score = float("-inf")
        for detection in detections.values():
            dist = ((detection.cx - last.cx) ** 2 + (detection.cy - last.cy) ** 2) ** 0.5
            iou = self._bbox_iou(last, detection)
            if dist > max_dist:
                continue
            if iou < self._min_reacquire_iou and dist > strict_dist:
                continue

            norm_dist = dist / max(max_dist, 1.0)
            score = iou - norm_dist
            if score > best_score:
                best = detection
                best_score = score
        return best

    @staticmethod
    def _bbox_iou(a: TargetDetection, b: TargetDetection) -> float:
        ix1 = max(a.x1, b.x1)
        iy1 = max(a.y1, b.y1)
        ix2 = min(a.x2, b.x2)
        iy2 = min(a.y2, b.y2)
        iw = max(0.0, ix2 - ix1)
        ih = max(0.0, iy2 - iy1)
        inter = iw * ih
        union = a.bbox_area + b.bbox_area - inter
        return inter / union if union > 0 else 0.0
