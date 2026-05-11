"""
mission/hud_renderer.py — HUD overlay drawing extracted from tracker.py.

HudRenderer draws all on-screen telemetry, tracking boxes, search status,
battery bar, and recording indicator onto a frame.  It holds a reference
to the host PersonGimbalTracker so future refactors can progressively inject
explicit dependencies rather than changing the call-site all at once.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Optional

import cv2
import numpy as np

import config as cfg
from tracking.state_machine import State
from tracking.target_detection import TargetDetection

if TYPE_CHECKING:
    from tracker import PersonGimbalTracker


class HudRenderer:
    """Draws telemetry and tracking annotations onto a BGR frame.

    Args:
        tracker: Host PersonGimbalTracker — provides live state for the HUD.
    """

    def __init__(self, tracker: "PersonGimbalTracker") -> None:
        self._t = tracker

    def draw(self, frame: np.ndarray, target_info: Optional[TargetDetection]) -> None:
        """Annotate *frame* in-place with the full HUD overlay."""
        t = self._t
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        # Mode indicator — top-right
        mode_col = (0, 255, 0) if t.mode == "AUTO" else (0, 200, 255)
        cv2.putText(frame, t.mode, (w - 72, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, mode_col, 2)

        # MANUAL mode bottom banner
        if t.mode == "MANUAL":
            cv2.rectangle(frame, (0, h - 28), (w, h), (0, 60, 100), -1)
            cv2.putText(frame,
                "MANUAL MODE  ←↑↓→ pan/tilt  |  m = AUTO  |  web D-pad available",
                (8, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1)

        # Crosshair + dead-zone rect
        cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 255), 1)
        cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 255), 1)
        dz_x = int(cfg.DEAD_ZONE * w / 2)
        dz_y = int(cfg.DEAD_ZONE * h / 2)
        cv2.rectangle(frame, (cx - dz_x, cy - dz_y), (cx + dz_x, cy + dz_y), (0, 255, 255), 1)

        colors = {
            State.TRACKING:          (0, 255, 0),
            State.PREDICTING:        (0, 200, 255),
            State.PRED_FADE:         (0, 140, 255),
            State.SEARCHING:         (255, 255, 0),
            State.EXPANDING_SQUARE:  (0, 128, 255),
            State.LISSAJOUS:         (255, 0, 255),
            State.INITIAL_SCAN:      (255, 255, 255),
            State.WAITING:           (0, 0, 255),
        }
        color = colors.get(t.state, (255, 255, 255))

        fw, fh = t.grabber.frame_w, t.grabber.frame_h
        if fw > 0:
            sx, sy = w / fw, h / fh
            grey   = (160, 160, 160)
            snap   = dict(t._detected_ids)
            tracked_tid = target_info.track_id if target_info is not None else -1
            for tid, pd in snap.items():
                if tid == tracked_tid:
                    continue
                cv2.rectangle(frame, (int(pd.x1 * sx), int(pd.y1 * sy)),
                              (int(pd.x2 * sx), int(pd.y2 * sy)), grey, 1)
                cv2.putText(frame, f"ID:{tid}",
                            (int(pd.cx * sx) - 20, max(int(pd.cy * sy) - 8, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, grey, 1)

        if target_info is not None and fw > 0:
            sx, sy = w / fw, h / fh
            x1, y1, x2, y2 = target_info.x1, target_info.y1, target_info.x2, target_info.y2
            tcx, tcy = target_info.cx, target_info.cy
            tid = target_info.track_id
            cv2.rectangle(frame, (int(x1 * sx), int(y1 * sy)),
                          (int(x2 * sx), int(y2 * sy)), color, 2)
            id_label = f"ID:{tid}" if tid >= 0 else ""
            if t._lock_id is not None and tid == t._lock_id:
                id_label += " [LOCK]"
            if id_label:
                cv2.putText(frame, id_label,
                            (int(x1 * sx), max(int(y1 * sy) - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 2)
            cv2.circle(frame, (int(tcx * sx), int(tcy * sy)), 4, (0, 0, 255), -1)
            cv2.line(frame, (cx, cy), (int(tcx * sx), int(tcy * sy)), color, 1)

        if t.velocity.valid and t.state in (State.TRACKING, State.PREDICTING, State.PRED_FADE):
            ax = int(cx + t.velocity.vel_x * 100)
            ay = int(cy + t.velocity.vel_y * 100)
            arrow_color = (0, 255, 255) if t.state == State.TRACKING else (0, 165, 255)
            cv2.arrowedLine(frame, (cx, cy), (ax, ay), arrow_color, 2, tipLength=0.3)

        y = 16
        state_txt: str = t.state
        if t.state == State.PREDICTING and t.target_lost_time:
            state_txt += f" {time.time() - t.target_lost_time:.1f}s"
        elif t.state == State.PRED_FADE and t.target_lost_time:
            dt   = time.time() - t.target_lost_time - cfg.PREDICT_DURATION
            fade = 1.0 - (1.0 - cfg.PRED_FADE_MIN_FACTOR) * (dt / cfg.PRED_FADE_DURATION)
            state_txt += f" {fade:.0%}"
        elif t.state == State.SEARCHING:
            state_txt += f" {t.search.phase_name}"
        elif t.state == State.INITIAL_SCAN:
            state_txt += f" {t.init_scan.status}"
        elif t.state == State.EXPANDING_SQUARE:
            state_txt += f" {t.expand_search.status}"
        elif t.state == State.LISSAJOUS:
            state_txt += f" {t.lissajous.status}"
        cv2.putText(frame, state_txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        y += 18

        cv2.putText(frame, f"Err X:{t.telem_error_x:+.2f} Y:{t.telem_error_y:+.2f}",
                    (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)
        y += 16
        cv2.putText(frame, f"Cmd Y:{t.telem_yaw_cmd:+3d} P:{t.telem_pitch_cmd:+3d}",
                    (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)
        y += 16
        if t.velocity.valid:
            cv2.putText(frame, f"Vel:{t.velocity.speed:.2f}/s Dir:{t.velocity.direction_deg:.0f}deg",
                        (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 200), 1)
            y += 16
        if t.velocity.edge_exit and t.state in (State.PREDICTING, State.PRED_FADE):
            cv2.putText(frame, f"EDGE: {t.velocity.edge_exit} x{cfg.EDGE_EXIT_BOOST:.1f}",
                        (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 100, 255), 1)
            y += 16
        cv2.putText(frame,
                    f"Kp:{t.telem_adaptive_kp:.0f} Spd:{t.telem_adaptive_speed:.0f} "
                    f"Sz:{t.telem_bbox_ratio:.0%}",
                    (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1)
        y += 16

        cv2.putText(frame, f"P:{t.fps:.0f} G:{t.grabber.grab_fps:.0f}",
                    (w - 120, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1)

        bv = t.ctrl.battery_voltage
        if bv > 0.0:
            pw  = t.ctrl.power_watts
            ima = t.ctrl.current_ma
            bc  = ((0, 255, 0) if bv >= cfg.BATT_WARN_V
                   else (0, 200, 255) if bv >= cfg.BATT_CRIT_V
                   else (0, 0, 255))
            cv2.putText(frame, f"{bv:.2f}V {ima}mA {pw:.1f}W",
                        (w - 145, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.38, bc, 1)
            bx, by, bw2, bh2 = w - 145, 35, 40, 7
            fill = max(0.0, min(1.0, (bv - cfg.BATT_DISPLAY_MIN_V) /
                                     (cfg.BATT_DISPLAY_MAX_V - cfg.BATT_DISPLAY_MIN_V)))
            cv2.rectangle(frame, (bx, by), (bx + bw2, by + bh2), (80, 80, 80), -1)
            cv2.rectangle(frame, (bx, by), (bx + int(bw2 * fill), by + bh2), bc, -1)
            cv2.rectangle(frame, (bx, by), (bx + bw2, by + bh2), (160, 160, 160), 1)
        else:
            cv2.putText(frame, "Batt: --", (w - 80, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 100, 100), 1)

        if t._drone_enabled:
            drone_col = (0, 255, 0) if t.mav.is_connected() else (0, 0, 255)
            cv2.putText(frame, f"DRONE: {'OK' if t.mav.is_connected() else 'NO LINK'}",
                        (w - 145, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.38, drone_col, 1)

        if t.recorder.is_recording:
            cv2.circle(frame, (w - 12, h - 10), 5, (0, 0, 255), -1)
            cv2.putText(frame, "REC", (w - 50, h - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 255), 1)
        cv2.putText(frame, "A8mini v10", (w - 85, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        ind: Optional[str] = None
        ind_color           = (255, 255, 255)
        if t.state == State.SEARCHING:
            if t.search.pitch_scanning:
                ind = "PITCH SCAN ^v"
            else:
                d = ">>>" if t.search.yaw_direction > 0 else "<<<"
                ind = f"{d} {t.search.phase_name} Spd:{t.search.current_speed}"
            ind_color = (255, 255, 0)
        elif t.state == State.INITIAL_SCAN:
            ind = f"INIT-SCAN {t.init_scan.status}"
        elif t.state == State.EXPANDING_SQUARE:
            ind = f"EXP-SQ {t.expand_search.status}"
            ind_color = (0, 128, 255)
        elif t.state == State.LISSAJOUS:
            ind = f"LISSAJOUS {t.lissajous.status}"
            ind_color = (255, 0, 255)
        if ind:
            cv2.putText(frame, ind, (w // 2 - 100, h - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, ind_color, 1)
