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
        min_reacquire_appearance: float = cfg.LOCK_REACQUIRE_MIN_APPEARANCE,
        ambiguity_margin: float = cfg.LOCK_REACQUIRE_AMBIGUITY_MARGIN,
        visible_max_center_ratio: float = cfg.LOCK_VISIBLE_MAX_CENTER_RATIO,
    ) -> None:
        self._registry = registry
        self._lock_grace_s = lock_grace_s
        self._reacquire_center_ratio = reacquire_center_ratio
        self._strict_center_ratio = strict_center_ratio
        self._min_reacquire_iou = min_reacquire_iou
        self._min_reacquire_appearance = min_reacquire_appearance
        self._ambiguity_margin = ambiguity_margin
        self._visible_max_center_ratio = visible_max_center_ratio
        self._last_locked_id: Optional[int] = None
        self._last_locked_detection: Optional[TargetDetection] = None
        self._last_locked_t: float = 0.0
        self._unlocked_target_id: Optional[int] = None
        self._last_unlocked_t: float = 0.0
        self._fallback_track_seq = -1
        self._last_reacquire_reason = ""

    def select(self, results, state: TrackerState) -> Optional[TargetDetection]:
        """Update state.detected_ids and return the chosen TargetDetection.

        Respects state.lock_id: if set, returns that persistent ID's detection
        (or None if not currently visible). Otherwise keeps the same unlocked
        preview target while visible, falling back to the largest box.

        Side-effect: overwrites state.detected_ids every call.
        """
        now = time.time()

        if not results or len(results[0].boxes) == 0:
            state.detected_ids = {}
            self._registry.mark_inactive(set())
            if state.lock_id is None:
                self._set_target_status(state, "unlocked", "")
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

            lock_sim = self._registry.similarity_to(state.lock_id, crop)
            pid = self._registry.update(bt_id, crop, cx, cy, now, exclude_pids=assigned_pids)
            assigned_pids.add(pid)
            active_bt.add(bt_id)
            new_ids[pid] = TargetDetection(
                cx=cx, cy=cy, x1=x1, y1=y1, x2=x2, y2=y2,
                conf=conf, track_id=pid, appearance_sim_to_lock=lock_sim,
            )

        state.detected_ids = new_ids
        self._registry.mark_inactive(active_bt)

        if state.lock_id is not None:
            locked = new_ids.get(state.lock_id)
            if locked is not None:
                if self._locked_detection_plausible(
                    locked, new_ids, frame.shape[1], frame.shape[0]
                ):
                    self._remember_locked(state.lock_id, locked, now)
                    self._set_target_status(
                        state, "locked_visible",
                        f"ID {state.lock_id} app={locked.appearance_sim_to_lock:.2f}",
                    )
                    return locked
                self._set_target_status(
                    state,
                    "lock_hold",
                    self._last_reacquire_reason or "locked ID failed continuity check",
                )
                return self._locked_grace_target(state, now)

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
                self._set_target_status(
                    state, "lock_reacquired",
                    f"ID {old_id}->{state.lock_id} app={reacquired.appearance_sim_to_lock:.2f}",
                )
                return reacquired

            self._set_target_status(
                state,
                "lock_hold",
                self._last_reacquire_reason or "locked target not confidently visible",
            )
            return self._locked_grace_target(state, now)

        self._set_target_status(state, "unlocked", "")
        return self._select_unlocked_target(new_ids, now)

    def _remember_locked(self, lock_id: int, detection: TargetDetection, now: float) -> None:
        self._last_locked_id = lock_id
        self._last_locked_detection = detection
        self._last_locked_t = now
        self._unlocked_target_id = lock_id
        self._last_unlocked_t = now
        self._last_reacquire_reason = ""

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
            warning = (
                state.target_warning
                if state.target_status == "lock_hold" and state.target_warning
                else "coasting through brief visual dropout"
            )
            self._set_target_status(state, "locked_grace", warning)
            return replace(self._last_locked_detection, is_fresh=False)
        if state.lock_id is not None:
            self._set_target_status(state, "lock_lost", "locked target not visible")
        return None

    def _select_unlocked_target(
        self,
        detections: dict[int, TargetDetection],
        now: float,
    ) -> Optional[TargetDetection]:
        """Keep preview tracking on one person instead of chasing largest box.

        Drone-body follow still requires an explicit operator lock.  This
        only stabilises the gimbal preview before lock so a 2-3 person scene
        does not snap between people whenever someone steps closer.
        """
        if not detections:
            return None

        if self._unlocked_target_id in detections:
            self._last_unlocked_t = now
            return detections[self._unlocked_target_id]  # type: ignore[index]

        if (
            self._unlocked_target_id is not None
            and now - self._last_unlocked_t <= self._lock_grace_s
        ):
            return None

        best_pid, best_area = None, 0.0
        for pid, detection in detections.items():
            if detection.bbox_area > best_area:
                best_area, best_pid = detection.bbox_area, pid
        if best_pid is None:
            return None
        self._unlocked_target_id = best_pid
        self._last_unlocked_t = now
        return detections[best_pid]

    @staticmethod
    def _set_target_status(state: TrackerState, status: str, warning: str = "") -> None:
        state.target_status = status
        state.target_warning = warning

    def _locked_detection_plausible(
        self,
        locked: TargetDetection,
        detections: dict[int, TargetDetection],
        frame_w: int,
        frame_h: int,
    ) -> bool:
        if self._last_locked_detection is None or self._last_locked_id != locked.track_id:
            return True
        frame_diag = (frame_w * frame_w + frame_h * frame_h) ** 0.5
        max_dist = frame_diag * self._visible_max_center_ratio
        locked_score = self._candidate_score(self._last_locked_detection, locked, max_dist)
        if locked_score is None:
            self._last_reacquire_reason = "locked ID jumped too far from last confirmed box"
            return False
        if locked.appearance_sim_to_lock < self._min_reacquire_appearance:
            self._last_reacquire_reason = (
                f"locked ID appearance changed "
                f"({locked.appearance_sim_to_lock:.2f} < {self._min_reacquire_appearance:.2f})"
            )
            return False

        rival_scores = []
        for candidate in detections.values():
            if candidate.track_id == locked.track_id:
                continue
            if candidate.appearance_sim_to_lock < self._min_reacquire_appearance:
                continue
            score = self._candidate_score(self._last_locked_detection, candidate, max_dist)
            if score is not None:
                rival_scores.append(score)
        if rival_scores and locked_score - max(rival_scores) < self._ambiguity_margin:
            self._last_reacquire_reason = "two nearby people both match the lock — holding"
            return False
        return True

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

        self._last_reacquire_reason = ""
        frame_diag = (frame_w * frame_w + frame_h * frame_h) ** 0.5
        max_dist = frame_diag * self._reacquire_center_ratio
        strict_dist = frame_diag * self._strict_center_ratio
        last = self._last_locked_detection
        candidates: list[tuple[float, TargetDetection]] = []
        for detection in detections.values():
            dist = ((detection.cx - last.cx) ** 2 + (detection.cy - last.cy) ** 2) ** 0.5
            iou = self._bbox_iou(last, detection)
            if dist > max_dist:
                continue
            if iou < self._min_reacquire_iou and dist > strict_dist:
                continue
            if detection.appearance_sim_to_lock < self._min_reacquire_appearance:
                continue

            score = self._candidate_score(last, detection, max_dist)
            if score is not None:
                candidates.append((score, detection))
        if not candidates:
            self._last_reacquire_reason = "no candidate matched lock appearance + continuity"
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        if (
            len(candidates) > 1
            and candidates[0][0] - candidates[1][0] < self._ambiguity_margin
        ):
            self._last_reacquire_reason = "ambiguous reacquire between nearby people — holding"
            return None
        return candidates[0][1]

    def _candidate_score(
        self,
        last: TargetDetection,
        detection: TargetDetection,
        max_dist: float,
    ) -> Optional[float]:
        dist = ((detection.cx - last.cx) ** 2 + (detection.cy - last.cy) ** 2) ** 0.5
        iou = self._bbox_iou(last, detection)
        if dist > max_dist:
            return None
        if iou < self._min_reacquire_iou and dist > max(max_dist * 0.35, 1.0):
            return None
        norm_dist = dist / max(max_dist, 1.0)
        center_score = max(0.0, 1.0 - norm_dist)
        appearance = max(0.0, min(1.0, detection.appearance_sim_to_lock))
        return 0.45 * iou + 0.35 * appearance + 0.20 * center_score

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
