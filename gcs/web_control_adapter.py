"""
web_control_adapter.py — Web UI callback handlers extracted from tracker.py.

Handles click-to-track, mode switching, D-pad gimbal control, and zoom
commands arriving over HTTP from the browser.  All mutable state lives
in the shared TrackerState; this class holds no state of its own.
"""

from __future__ import annotations

import logging
import time
from typing import Callable, Optional

import config as cfg
from tracking.tracker_state import TrackerState

_log = logging.getLogger(__name__)


class WebControlAdapter:
    """Translate web UI requests into tracker state changes and gimbal commands.

    Args:
        state:          Shared TrackerState (lock_id, detected_ids, mode, …).
        ctrl:           SIYIController for gimbal stop/zoom commands.
        zoom_ctrl:      AutoZoomController to disable auto-zoom on manual zoom.
        grabber:        FrameGrabber — provides frame_w / frame_h for hit-testing.
        enter_manual:   Callable that switches the tracker to MANUAL mode.
        enter_auto:     Callable that switches the tracker to AUTO mode.
        request_brake:  Optional callable for FCU BRAKE mode.
        request_land:   Optional callable for FCU LAND mode.
        request_rtl:    Optional callable for FCU RTL mode.
        manual_gimbal_speed: Gimbal speed for web D-pad (default: cfg.MANUAL_GIMBAL_SPEED).
    """

    def __init__(
        self,
        state:    TrackerState,
        ctrl,
        zoom_ctrl,
        grabber,
        enter_manual: Callable[[], None],
        enter_auto:   Callable[[], None],
        request_brake: Optional[Callable[[], dict]] = None,
        request_land:  Optional[Callable[[], dict]] = None,
        request_rtl:   Optional[Callable[[], dict]] = None,
        manual_gimbal_speed: int | None = None,
    ) -> None:
        self._state    = state
        self._ctrl     = ctrl
        self._zoom     = zoom_ctrl
        self._grabber  = grabber
        self._enter_manual = enter_manual
        self._enter_auto   = enter_auto
        self._request_brake = request_brake
        self._request_land  = request_land
        self._request_rtl   = request_rtl
        self._spd = manual_gimbal_speed if manual_gimbal_speed is not None else cfg.MANUAL_GIMBAL_SPEED

    # ------------------------------------------------------------------

    def handle_click(self, nx: Optional[float], ny: Optional[float]) -> dict:
        """Handle a browser click on the live stream image.

        nx, ny: normalised 0–1 coordinates.
        Sentinels: nx == -1.0 → unlock;  nx is None → status query (read-only).
        Returns a dict JSON-serialised and sent to the browser.
        """
        state = self._state

        if nx is None:
            snap = dict(state.detected_ids)
            return {
                'lock_id': state.lock_id,
                'ids': list(snap.keys()),
                'target_status': state.target_status,
                'target_warning': state.target_warning,
            }

        if nx == -1.0:
            state.lock_id = None
            state.target_status = "unlocked"
            state.target_warning = ""
            print("[CLICK] Unlocked — auto mode")
            return {'status': 'unlocked', 'lock_id': None}

        fw, fh = self._grabber.frame_w, self._grabber.frame_h
        if fw <= 0 or fh <= 0:
            return {'status': 'error', 'msg': 'stream not ready'}

        orig_x, orig_y = nx * fw, ny * fh

        snap = dict(state.detected_ids)
        if not snap:
            return {'status': 'miss', 'msg': 'No persons detected'}

        best_id, best_area = None, float('inf')
        for tid, data in snap.items():
            x1, y1, x2, y2 = data.x1, data.y1, data.x2, data.y2
            if x1 <= orig_x <= x2 and y1 <= orig_y <= y2:
                area = (x2 - x1) * (y2 - y1)
                if area < best_area:
                    best_area, best_id = area, tid

        if best_id is None:
            if state.lock_id is not None:
                state.lock_id = None
                state.target_status = "unlocked"
                state.target_warning = ""
                print("[CLICK] Unlocked — clicked empty space")
                return {'status': 'unlocked', 'lock_id': None}
            return {'status': 'miss', 'msg': 'No person at that position'}

        if state.lock_id == best_id:
            state.lock_id = None
            state.target_status = "unlocked"
            state.target_warning = ""
            print(f"[CLICK] Unlocked — toggled off ID {best_id}")
            return {'status': 'unlocked', 'lock_id': None}

        state.lock_id = best_id
        state.target_status = "locked_pending"
        state.target_warning = "waiting for next confirmed detection"
        print(f"[CLICK] Locked → ID {best_id}")
        return {'status': 'locked', 'id': best_id, 'lock_id': best_id}

    def handle_mode(self, mode_str: Optional[str] = None) -> dict:
        """Query or set tracking mode.

        Args:
            mode_str: None = query only; 'manual' or 'auto' = set tracker
                mode; 'brake', 'land', or 'rtl' = request FCU safety mode.
        """
        state = self._state
        if mode_str == 'manual' and state.mode != 'MANUAL':
            self._enter_manual()
        elif mode_str == 'auto' and state.mode != 'AUTO':
            self._enter_auto()
        elif mode_str in ('brake', 'land', 'rtl'):
            cb = {
                'brake': self._request_brake,
                'land': self._request_land,
                'rtl': self._request_rtl,
            }[mode_str]
            if cb is None:
                return {
                    'mode': state.mode,
                    'status': 'error',
                    'msg': f'{mode_str.upper()} not wired',
                }
            result = cb()
            result.setdefault('mode', state.mode)
            return result
        return {'mode': state.mode}

    def handle_gimbal(self, direction: str) -> dict:
        """Handle D-pad gimbal command.

        Args:
            direction: 'up' | 'down' | 'left' | 'right' | 'stop'
        """
        state = self._state
        if state.mode != 'MANUAL':
            return {'status': 'error', 'msg': 'Switch to MANUAL mode first'}

        spd = self._spd
        if direction == 'left':
            state.manual_yaw_speed, state.manual_pitch_speed = -spd, 0
        elif direction == 'right':
            state.manual_yaw_speed, state.manual_pitch_speed =  spd, 0
        elif direction == 'up':
            state.manual_yaw_speed, state.manual_pitch_speed = 0,  spd
        elif direction == 'down':
            state.manual_yaw_speed, state.manual_pitch_speed = 0, -spd
        elif direction == 'stop':
            state.manual_yaw_speed   = 0
            state.manual_pitch_speed = 0
            self._ctrl.stop()
        else:
            _log.warning("[Web] Unknown gimbal direction: %r", direction)
            return {'status': 'error', 'msg': f'Unknown direction: {direction!r}'}

        state.manual_key_t = time.time() + 86400.0  # hold until explicit stop
        return {'status': 'ok', 'direction': direction}

    def handle_zoom(self, direction: str) -> None:
        """Handle zoom in/out command from web UI."""
        if direction == 'in':
            self._ctrl.zoom_in()
            self._zoom.enabled = False
            self._state.manual_zoom_stop_at = time.time() + 0.5
            print("[CLICK] Web zoom IN")
        elif direction == 'out':
            self._ctrl.zoom_out()
            self._zoom.enabled = False
            self._state.manual_zoom_stop_at = time.time() + 0.5
            print("[CLICK] Web zoom OUT")
