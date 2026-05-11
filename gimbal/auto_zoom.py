"""
auto_zoom.py — Automatic digital zoom controller for SIYI A8 mini.

Adjusts zoom level to keep the tracked person at a target bounding-box
height relative to frame height. Off by default (AUTO_ZOOM_ENABLED=False).
"""

import time

import config as cfg
from gimbal.siyi_controller import SIYIController


class AutoZoomController:
    """Automatic zoom adjustment based on bounding-box size.

    Args:
        ctrl: Shared SIYIController instance.
    """

    def __init__(self, ctrl: SIYIController) -> None:
        self.ctrl           = ctrl
        self.enabled: bool  = cfg.AUTO_ZOOM_ENABLED
        self.target_ratio   = cfg.ZOOM_TARGET_HEIGHT_RATIO
        self.tolerance      = cfg.ZOOM_TOLERANCE
        self._last_time     = 0.0
        self._zooming       = False
        self._zoom_stop_t   = 0.0

    def update(self, bbox_h: float, frame_h: float) -> None:
        """Adjust zoom to keep person at target size.

        Args:
            bbox_h:  Bounding box height in pixels.
            frame_h: Frame height in pixels.
        """
        if not self.enabled or frame_h <= 0 or bbox_h <= 0:
            return
        now = time.time()
        if self._zooming and now >= self._zoom_stop_t:
            self.ctrl.zoom_stop()
            self._zooming = False
        if now - self._last_time < cfg.ZOOM_CMD_INTERVAL:
            return
        ratio = bbox_h / frame_h
        error = ratio - self.target_ratio
        if abs(error) < self.tolerance:
            return
        if error < -self.tolerance and self.ctrl.current_zoom >= cfg.ZOOM_MAX - 0.1:
            return
        if error > self.tolerance and self.ctrl.current_zoom <= 1.1:
            return
        self._last_time = now
        if error < -self.tolerance:
            self.ctrl.zoom_in()
        else:
            self.ctrl.zoom_out()
        self._zooming     = True
        self._zoom_stop_t = now + 0.2

    def stop(self) -> None:
        if self._zooming:
            self.ctrl.zoom_stop()
            self._zooming = False
