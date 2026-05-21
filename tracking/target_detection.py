"""
tracking/target_detection.py — Typed container for a single YOLO detection.

Replaces the positional tuple (cx, cy, x1, y1, x2, y2, conf, track_id) that
was previously passed between tracker, drone controller, HUD renderer, and
EKF updater.  All coordinates are in *pixel space* relative to the source
camera frame (not normalised).
"""

from dataclasses import dataclass


@dataclass
class TargetDetection:
    """One detected person, selected and re-identified by PersonRegistry.

    Attributes:
        cx:       Pixel centre x of bounding box.
        cy:       Pixel centre y of bounding box.
        x1:       Pixel left edge of bounding box.
        y1:       Pixel top edge of bounding box.
        x2:       Pixel right edge of bounding box.
        y2:       Pixel bottom edge of bounding box.
        conf:     YOLO detection confidence [0, 1].
        track_id: Persistent person ID assigned by PersonRegistry.
        is_fresh: True when this came from the current detector frame; False
                  when it is a short grace/coast target.
        appearance_sim_to_lock: HSV-gallery similarity to the currently
                  locked person before this frame updates the registry.
    """
    cx:       float
    cy:       float
    x1:       float
    y1:       float
    x2:       float
    y2:       float
    conf:     float
    track_id: int
    is_fresh: bool = True
    appearance_sim_to_lock: float = 0.0

    @property
    def bbox_w(self) -> float:
        return self.x2 - self.x1

    @property
    def bbox_h(self) -> float:
        return self.y2 - self.y1

    @property
    def bbox_area(self) -> float:
        return self.bbox_w * self.bbox_h
