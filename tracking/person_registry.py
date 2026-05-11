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
    ) -> int:
        """Return the persistent ID for this detection.

        Args:
            bytetrack_id: ByteTrack internal ID for this frame.
            crop: BGR numpy array of the person's bounding box region.
            cx, cy: Centre pixel coordinates (unused, reserved for future use).
            now: Current timestamp (time.time()).

        Returns:
            Stable persistent ID for this person.
        """
        hist = self._compute_hist(crop)

        # Known ByteTrack ID → fast path (no gallery search)
        if bytetrack_id in self._bt_to_pid:
            pid   = self._bt_to_pid[bytetrack_id]
            entry = self._gallery[pid]
            entry['hist'] = 0.7 * entry['hist'] + 0.3 * hist
            norm = np.linalg.norm(entry['hist'])
            if norm > 0:
                entry['hist'] /= norm
            entry['last_seen'] = now
            entry['active']    = True
            return pid

        # New ByteTrack ID → try re-identification via gallery
        best_pid, best_sim = None, 0.0
        for pid, entry in self._gallery.items():
            sim = float(np.dot(hist, entry['hist']))   # cosine (both normalised)
            if sim > best_sim:
                best_sim, best_pid = sim, pid

        if best_pid is not None and best_sim >= cfg.REID_SIM_THRESHOLD:
            self._bt_to_pid[bytetrack_id] = best_pid
            entry = self._gallery[best_pid]
            entry['hist'] = 0.7 * entry['hist'] + 0.3 * hist
            norm = np.linalg.norm(entry['hist'])
            if norm > 0:
                entry['hist'] /= norm
            entry['last_seen'] = now
            entry['active']    = True
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
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hh = cv2.calcHist([hsv], [0], None, [16], [0, 180]).flatten()
        sh = cv2.calcHist([hsv], [1], None, [16], [0, 256]).flatten()
        vh = cv2.calcHist([hsv], [2], None, [16], [0, 256]).flatten()
        hist = np.concatenate([hh, sh, vh]).astype(np.float32)
        norm = np.linalg.norm(hist)
        return hist / norm if norm > 0 else hist
