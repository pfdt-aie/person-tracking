"""
person_registry.py — Persistent person identity across the full session.

Assigns stable IDs to people even after they leave the frame and ByteTrack
drops their internal track ID.

How it works:
  1. Each person gets a persistent integer ID (P-ID) on first appearance.
  2. Their bounding-box region is converted to an HSV color histogram
     (robust to brightness changes) and stored in a "gallery".
  3. Each frame, ByteTrack may assign new internal track IDs. We compare
     those detections against the gallery. High similarity → same person
     → keep original P-ID. Low similarity → genuinely new person → new P-ID.
  4. Gallery entries inactive for more than REID_GALLERY_TTL seconds are
     pruned to avoid false matches.
"""

import time

import cv2
import numpy as np

import config as cfg


class PersonRegistry:
    """Stable person identity across the full session.

    Uses appearance-based re-identification (HSV color histogram with
    cosine similarity) to reconnect ByteTrack IDs across track drops.
    """

    def __init__(self) -> None:
        self._next_pid       = 1
        self._bt_to_pid: dict = {}   # {bytetrack_id (int): persistent_id (int)}
        self._gallery: dict  = {}    # {persistent_id: {'hist', 'last_seen', 'active'}}

    def update(
        self,
        bytetrack_id: int,
        crop,
        cx: float,
        cy: float,
        now: float,
        exclude_pids: set[int] | None = None,
    ) -> int:
        """Return the persistent ID for this detection.

        Args:
            bytetrack_id: ByteTrack internal ID for this frame.
            crop: BGR numpy array of the person's bounding box region.
            cx, cy: Centre pixel coordinates (unused, reserved for future use).
            now: Current timestamp (time.time()).
            exclude_pids: Persistent IDs already assigned in the current frame.

        Returns:
            Stable persistent ID for this person.
        """
        hist = self._compute_hist(crop)
        exclude_pids = exclude_pids or set()

        # Known ByteTrack ID → fast path (no gallery search)
        if bytetrack_id in self._bt_to_pid:
            pid   = self._bt_to_pid[bytetrack_id]
            entry = self._gallery.get(pid)
            if entry is not None and pid not in exclude_pids:
                hist_norm = np.linalg.norm(hist)
                entry_norm = np.linalg.norm(entry['hist'])
                if hist_norm > 0 and entry_norm > 0:
                    sim = float(np.dot(hist, entry['hist']))
                else:
                    sim = 0.0
                if sim >= cfg.REID_KNOWN_ID_MIN_SIM:
                    self._update_entry(entry, hist, now)
                    return pid
            # ByteTrack IDs can be recycled after occlusion or tracker fallback.
            # Drop the stale mapping and let gallery matching/new-ID allocation
            # below decide where this detection belongs.
            self._bt_to_pid.pop(bytetrack_id, None)

        # New ByteTrack ID → try re-identification via gallery
        best_pid, best_sim = None, 0.0
        for pid, entry in self._gallery.items():
            if pid in exclude_pids:
                continue
            sim = float(np.dot(hist, entry['hist']))   # cosine (both normalised)
            if sim > best_sim:
                best_sim, best_pid = sim, pid

        if best_pid is not None and best_sim >= cfg.REID_SIM_THRESHOLD:
            self._bt_to_pid[bytetrack_id] = best_pid
            entry = self._gallery[best_pid]
            self._update_entry(entry, hist, now)
            return best_pid

        # Genuinely new person
        pid = self._next_pid
        self._next_pid += 1
        self._bt_to_pid[bytetrack_id] = pid
        self._gallery[pid] = {
            'hist':      hist,
            'last_seen': now,
            'active':    True,
        }
        return pid

    def similarity_to(self, pid: int | None, crop) -> float:
        """Return appearance similarity between *crop* and an existing P-ID.

        This is used by TargetSelector before the registry is updated for the
        frame, so lock reacquisition can compare a candidate against the
        previously-confirmed locked person instead of the candidate's freshly
        written gallery entry.
        """
        if pid is None:
            return 0.0
        entry = self._gallery.get(pid)
        if entry is None:
            return 0.0
        hist = self._compute_hist(crop)
        if np.linalg.norm(hist) <= 0 or np.linalg.norm(entry['hist']) <= 0:
            return 0.0
        return float(np.dot(hist, entry['hist']))

    @staticmethod
    def _update_entry(entry: dict, hist: np.ndarray, now: float) -> None:
        entry['hist'] = 0.7 * entry['hist'] + 0.3 * hist
        norm = np.linalg.norm(entry['hist'])
        if norm > 0:
            entry['hist'] /= norm
        entry['last_seen'] = now
        entry['active'] = True

    def mark_inactive(self, active_bytetrack_ids: set) -> None:
        """Mark gallery entries not currently visible as inactive; prune stale ones."""
        active_pids = {self._bt_to_pid[bt] for bt in active_bytetrack_ids
                       if bt in self._bt_to_pid}
        now = time.time()
        to_delete = []
        for pid, entry in self._gallery.items():
            entry['active'] = pid in active_pids
            if not entry['active'] and now - entry['last_seen'] > cfg.REID_GALLERY_TTL:
                to_delete.append(pid)
        for pid in to_delete:
            self._gallery.pop(pid, None)
            self._bt_to_pid = {bt: p for bt, p in self._bt_to_pid.items() if p != pid}

    def reset(self) -> None:
        self._bt_to_pid.clear()
        self._gallery.clear()
        self._next_pid = 1

    @staticmethod
    def _compute_hist(crop) -> np.ndarray:
        """Normalised HSV color histogram (48 bins). Returns zeros on bad crop."""
        if crop is None or crop.size == 0 or crop.shape[0] < 4 or crop.shape[1] < 4:
            return np.zeros(48, dtype=np.float32)
        h, w = crop.shape[:2]
        # Focus on the central body area. Full-box histograms include floor,
        # sky, and nearby people, which makes 2-3 person ground tests much
        # more prone to ID swaps.
        x1, x2 = int(w * 0.15), int(w * 0.85)
        y1, y2 = int(h * 0.08), int(h * 0.92)
        body = crop[y1:y2, x1:x2]
        if body.shape[0] >= 4 and body.shape[1] >= 4:
            crop = body
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hh = cv2.calcHist([hsv], [0], None, [16], [0, 180]).flatten()
        sh = cv2.calcHist([hsv], [1], None, [16], [0, 256]).flatten()
        vh = cv2.calcHist([hsv], [2], None, [16], [0, 256]).flatten()
        hist = np.concatenate([hh, sh, vh]).astype(np.float32)
        norm = np.linalg.norm(hist)
        return hist / norm if norm > 0 else hist
