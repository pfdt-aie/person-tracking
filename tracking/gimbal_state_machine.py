"""
gimbal_state_machine.py — Gimbal tracking state machine extracted from tracker.py.

Owns the 8-state FSM (TRACKING, PREDICTING, PRED_FADE, SEARCHING,
EXPANDING_SQUARE, LISSAJOUS, INITIAL_SCAN, WAITING), the PID/smoother
dispatch for each state, and the adaptive speed control.

The only shared mutable state is TrackerState; everything else lives here.
"""

from __future__ import annotations

import time
from typing import Optional

import config as cfg
from tracking.state_machine import State
from tracking.target_detection import TargetDetection
from tracking.tracker_state import TrackerState
from utils import terminal


class GimbalStateMachine:
    """Gimbal FSM — computes gimbal commands each frame and drives state transitions.

    Args:
        state:         Shared TrackerState (read: tracking_enabled, search_enabled,
                       state; write: state, telem_*).
        grabber:       FrameGrabber — provides frame_w / frame_h.
        pid_yaw:       Yaw PID controller.
        pid_pitch:     Pitch PID controller.
        smoother:      TargetSmoother for bbox centre smoothing.
        velocity:      VelocityTracker for prediction-phase speed.
        ctrl:          SIYIController — receives set_speed / center / stop commands.
        zoom_ctrl:     AutoZoomController — updated each tracking frame.
        search:        SectorScanSearch.
        init_scan:     InitialScanSearch.
        expand_search: ExpandingSquareSearch.
        lissajous:     LissajousSearch.
        drone_ctrl:    Optional DroneController; if provided, its lightweight
                       visual reacquisition hook is called on re-acquisition.
    """

    def __init__(
        self,
        state:         TrackerState,
        grabber,
        pid_yaw,
        pid_pitch,
        smoother,
        velocity,
        ctrl,
        zoom_ctrl,
        search,
        init_scan,
        expand_search,
        lissajous,
        drone_ctrl     = None,
    ) -> None:
        self._state        = state
        self._grabber      = grabber
        self._pid_yaw      = pid_yaw
        self._pid_pitch    = pid_pitch
        self._smoother     = smoother
        self._velocity     = velocity
        self._ctrl         = ctrl
        self._zoom         = zoom_ctrl
        self._search       = search
        self._init_scan    = init_scan
        self._expand       = expand_search
        self._lissajous    = lissajous
        self._drone_ctrl   = drone_ctrl

        # Internal state not shared with other components
        self._frames_without_person: int   = 0
        self._target_lost_time: Optional[float] = None
        self._search_phase_start: float    = 0.0
        self._last_cmd_yaw_dir:   int      = 1
        self._last_cmd_pitch_dir: int      = 0
        self._lissajous_ground_recover_until: float = 0.0

    # ------------------------------------------------------------------
    #  Public entry points
    # ------------------------------------------------------------------

    def update(self, person_detected: bool, target_info: Optional[TargetDetection]) -> None:
        """Drive the FSM one tick and issue the appropriate gimbal command.

        Called once per frame from the main loop when mode == 'AUTO'.
        Updates state.state and all state.telem_* fields.
        """
        self._update_state(person_detected, target_info)
        self._dispatch(target_info)

    def reset_all(self) -> None:
        """Stop all patterns/PIDs and transition to INITIAL_SCAN or WAITING."""
        st = self._state
        self._ctrl.stop()
        self._zoom.stop()
        self._pid_yaw.reset()
        self._pid_pitch.reset()
        self._smoother.reset()
        self._velocity.reset()
        self._search.stop()
        self._init_scan.stop()
        self._expand.stop()
        self._lissajous.stop()

        self._target_lost_time       = None
        self._frames_without_person  = 0
        self._search_phase_start     = 0.0
        self._lissajous_ground_recover_until = 0.0

        st.telem_error_x = st.telem_error_y = 0.0
        st.telem_yaw_cmd = st.telem_pitch_cmd = 0
        st.telem_bbox_ratio = 0.0

        if st.tracking_enabled and st.search_enabled:
            st.state = State.INITIAL_SCAN
            self._init_scan.start()
        else:
            st.state = State.WAITING

    # ------------------------------------------------------------------
    #  State dispatch
    # ------------------------------------------------------------------

    def _dispatch(self, target_info: Optional[TargetDetection]) -> None:
        st = self._state
        if st.state == State.TRACKING and target_info is not None:
            self._handle_tracking(target_info)
        elif st.state == State.PREDICTING:
            self._handle_predicting()
        elif st.state == State.PRED_FADE:
            self._handle_pred_fade()
        elif st.state == State.SEARCHING:
            self._handle_searching()
        elif st.state == State.INITIAL_SCAN:
            self._handle_initial_scan()
        elif st.state == State.EXPANDING_SQUARE:
            self._handle_expanding_square()
        elif st.state == State.LISSAJOUS:
            self._handle_lissajous()
        # WAITING: no gimbal command

    # ------------------------------------------------------------------
    #  State handlers
    # ------------------------------------------------------------------

    def _handle_tracking(self, target_info: TargetDetection) -> None:
        st  = self._state
        fh  = self._grabber.frame_h
        fw  = self._grabber.frame_w
        st.telem_bbox_ratio = target_info.bbox_h / fh if fh > 0 else 0
        self._adapt_speed(st.telem_bbox_ratio)

        self._velocity.update(
            target_info.cx, target_info.cy,
            target_info.x1, target_info.y1,
            target_info.x2, target_info.y2,
            fw, fh,
        )

        cx, cy = self._smoother.update(target_info.cx, target_info.cy)
        ex, ey = self._compute_error(cx, cy)
        yaw    = self._pid_yaw.update(ex)
        pitch  = self._pid_pitch.update(-ey)
        self._ctrl.set_speed(int(yaw), int(pitch))

        self._zoom.update(target_info.bbox_h, fh)

        st.telem_error_x,   st.telem_error_y   = ex, ey
        st.telem_yaw_cmd,   st.telem_pitch_cmd  = int(yaw), int(pitch)

        if abs(yaw)   > 4: self._last_cmd_yaw_dir   = 1 if yaw   > 0 else -1
        if abs(pitch) > 4: self._last_cmd_pitch_dir = 1 if pitch > 0 else -1

    def _handle_predicting(self) -> None:
        st = self._state
        dt = time.time() - self._target_lost_time  # type: ignore[operator]
        yaw, pitch = self._velocity.predict_command(dt, st.telem_adaptive_kp, st.telem_adaptive_speed)
        self._ctrl.set_speed(yaw, pitch)
        st.telem_yaw_cmd, st.telem_pitch_cmd = yaw, pitch

    def _handle_pred_fade(self) -> None:
        st       = self._state
        dt_total = time.time() - self._target_lost_time  # type: ignore[operator]
        dt_fade  = dt_total - cfg.PREDICT_DURATION
        fade     = 1.0 - (1.0 - cfg.PRED_FADE_MIN_FACTOR) * (dt_fade / cfg.PRED_FADE_DURATION)
        fade     = max(cfg.PRED_FADE_MIN_FACTOR, min(1.0, fade))
        yaw, pitch = self._velocity.predict_command(dt_total, st.telem_adaptive_kp, st.telem_adaptive_speed)
        yaw, pitch = int(yaw * fade), int(pitch * fade)
        self._ctrl.set_speed(yaw, pitch)
        st.telem_yaw_cmd, st.telem_pitch_cmd = yaw, pitch

    def _handle_searching(self) -> None:
        yaw, pitch = self._search.get_command()
        yaw, pitch = self._apply_search_pitch_limits(yaw, pitch)
        self._ctrl.set_speed(yaw, pitch)
        st = self._state
        st.telem_yaw_cmd, st.telem_pitch_cmd = yaw, pitch

    def _handle_initial_scan(self) -> None:
        yaw, pitch = self._init_scan.get_command()
        yaw, pitch = self._apply_search_pitch_limits(yaw, pitch)
        self._ctrl.set_speed(yaw, pitch)
        st = self._state
        st.telem_yaw_cmd, st.telem_pitch_cmd = yaw, pitch

    def _handle_expanding_square(self) -> None:
        yaw, pitch = self._expand.get_command()
        yaw, pitch = self._apply_search_pitch_limits(yaw, pitch)
        self._ctrl.set_speed(yaw, pitch)
        st = self._state
        st.telem_yaw_cmd, st.telem_pitch_cmd = yaw, pitch

    def _handle_lissajous(self) -> None:
        if self._lissajous.needs_center_cmd:
            self._ctrl.center()
            self._lissajous_ground_recover_until = (
                time.time() + cfg.LISSAJOUS_RECENTER_DWELL + cfg.SEARCH_RECENTER_GROUND_TIMEOUT_S
            )
        yaw, pitch = self._lissajous.get_command()
        yaw, pitch = self._apply_lissajous_ground_recovery(yaw, pitch)
        yaw, pitch = self._apply_search_pitch_limits(yaw, pitch)
        self._ctrl.set_speed(yaw, pitch)
        st = self._state
        st.telem_yaw_cmd, st.telem_pitch_cmd = yaw, pitch

    # ------------------------------------------------------------------
    #  State transitions
    # ------------------------------------------------------------------

    def _update_state(self, person_detected: bool, target_info) -> None:
        st  = self._state
        now = time.time()

        if person_detected and st.tracking_enabled:
            self._frames_without_person = 0
            if st.state != State.TRACKING:
                prev = st.state
                self._pid_yaw.reset(); self._pid_pitch.reset(); self._smoother.reset()
                self._search.stop(); self._init_scan.stop()
                self._expand.stop(); self._lissajous.stop()
                st.state = State.TRACKING
                if self._drone_ctrl is not None:
                    if prev in State.SEARCH_STATES:
                        # Long search: reset EKF and smoother so the first
                        # velocity command after re-acquisition is driven by
                        # fresh projection data, not a stale estimate.
                        # Do NOT clear the NED origin — keeping the same GPS
                        # reference frame prevents repeated origin resets that
                        # destabilise EKF position and cause yaw oscillations.
                        self._drone_ctrl.reset()
                    else:
                        # Brief loss (PREDICTING/PRED_FADE): preserve EKF
                        # continuity — only seed the velocity smoother.
                        self._drone_ctrl.on_target_reacquired()
                if prev in (State.PREDICTING, State.PRED_FADE):
                    edge = self._velocity.edge_exit
                    terminal.event(f"[State] Re-acquired from {prev}"
                                   f"{f' (edge:{edge})' if edge else ''}")
                elif prev in State.SEARCH_STATES:
                    self._velocity.reset()
                    terminal.event(f"[State] Found during {prev}")
            self._target_lost_time = None
            return

        self._frames_without_person += 1

        if st.state == State.TRACKING and self._frames_without_person < cfg.LOST_CONFIRM_FRAMES:
            return

        if st.state == State.TRACKING:
            self._target_lost_time = now
            self._smoother.reset(); self._zoom.stop()
            self._pid_yaw.reset();  self._pid_pitch.reset()

            use_vel = self._velocity.valid and self._velocity.speed > cfg.MIN_VELOCITY_PREDICT
            if use_vel:
                st.state = State.PREDICTING
                edge = self._velocity.edge_exit
                terminal.event(f"[State] Lost → PREDICTING (vel={self._velocity.speed:.2f}/s"
                               f"{f', edge:{edge}' if edge else ''})")
            elif st.search_enabled:
                self._search.start(yaw_dir=self._last_cmd_yaw_dir, pitch_dir=self._last_cmd_pitch_dir)
                st.state = State.SEARCHING
                self._search_phase_start = now
                dir_label = 'R' if self._last_cmd_yaw_dir > 0 else 'L'
                terminal.event(f"[State] Lost → SEARCHING (no velocity, dir={dir_label})")
            else:
                st.state = State.WAITING
                self._ctrl.stop()

        elif st.state == State.PREDICTING:
            if now - self._target_lost_time > cfg.PREDICT_DURATION:  # type: ignore[operator]
                st.state = State.PRED_FADE
                terminal.event("[State] PREDICTING → PRED_FADE")

        elif st.state == State.PRED_FADE:
            if now - self._target_lost_time > cfg.PREDICT_DURATION + cfg.PRED_FADE_DURATION:  # type: ignore[operator]
                self._ctrl.stop(); self._pid_yaw.reset(); self._pid_pitch.reset()
                if st.search_enabled:
                    if self._velocity.valid and self._velocity.speed > cfg.MIN_VELOCITY_PREDICT:
                        yaw_dir, pitch_dir = self._velocity.get_search_direction()
                    else:
                        yaw_dir, pitch_dir = self._last_cmd_yaw_dir, self._last_cmd_pitch_dir
                    self._search.start(yaw_dir=yaw_dir, pitch_dir=pitch_dir)
                    st.state = State.SEARCHING
                    self._search_phase_start = now
                    terminal.event("[State] PRED_FADE → SEARCHING")
                else:
                    st.state = State.WAITING

        elif st.state == State.SEARCHING:
            if st.search_enabled and now - self._search_phase_start >= cfg.SECTOR_SEARCH_TIMEOUT:
                self._search.stop()
                self._expand.start(yaw_dir=self._last_cmd_yaw_dir, pitch_dir=self._last_cmd_pitch_dir)
                st.state = State.EXPANDING_SQUARE
                self._search_phase_start = now
                terminal.event("[State] SEARCHING → EXPANDING_SQUARE")

        elif st.state == State.EXPANDING_SQUARE:
            if st.search_enabled and now - self._search_phase_start >= cfg.EXPAND_SEARCH_TIMEOUT:
                self._expand.stop()
                self._lissajous.start()
                st.state = State.LISSAJOUS
                self._search_phase_start = now
                terminal.event("[State] EXPANDING_SQUARE → LISSAJOUS")

    # ------------------------------------------------------------------
    #  Adaptive speed + error helpers
    # ------------------------------------------------------------------

    def _adapt_speed(self, bbox_ratio: float) -> None:
        t     = max(0.0, min(1.0, (bbox_ratio - cfg.ADAPT_LOW) / (cfg.ADAPT_HIGH - cfg.ADAPT_LOW)))
        kp    = cfg.ADAPT_KP_MIN    + t * (cfg.ADAPT_KP_MAX    - cfg.ADAPT_KP_MIN)
        speed = cfg.ADAPT_SPEED_MIN + t * (cfg.ADAPT_SPEED_MAX - cfg.ADAPT_SPEED_MIN)
        self._pid_yaw.kp             = kp
        self._pid_pitch.kp           = kp
        self._pid_yaw.output_limit   = speed
        self._pid_pitch.output_limit = speed
        st = self._state
        st.telem_adaptive_kp    = kp
        st.telem_adaptive_speed = speed

    def _compute_error(self, cx: float, cy: float) -> tuple:
        fw, fh = self._grabber.frame_w, self._grabber.frame_h
        return (
            self._sdz((cx - fw / 2) / (fw / 2), cfg.DEAD_ZONE),
            self._sdz((cy - fh / 2) / (fh / 2), cfg.DEAD_ZONE),
        )

    def _apply_search_pitch_limits(self, yaw: int, pitch: int) -> tuple[int, int]:
        """Clamp search pitch commands to the configured ground-looking envelope."""
        return self._clamp_search_pitch_for_tilt(yaw, pitch, self._ctrl.gimbal_tilt_deg)

    def _apply_lissajous_ground_recovery(self, yaw: int, pitch: int) -> tuple[int, int]:
        """After center(), pitch back down to the shallow search bound."""
        if self._lissajous_ground_recover_until <= 0.0:
            return yaw, pitch

        now = time.time()
        if now >= self._lissajous_ground_recover_until:
            self._lissajous_ground_recover_until = 0.0
            return yaw, pitch

        if self._ctrl.gimbal_tilt_deg <= cfg.SEARCH_PITCH_SHALLOW_DEG:
            self._lissajous_ground_recover_until = 0.0
            return yaw, pitch

        return 0, -cfg.SEARCH_RECENTER_PITCH_SPEED

    @staticmethod
    def _clamp_search_pitch_for_tilt(yaw: int, pitch: int, tilt_deg: float) -> tuple[int, int]:
        if pitch < 0 and tilt_deg <= cfg.SEARCH_PITCH_STEEP_DEG:
            return yaw, 0
        if pitch > 0 and tilt_deg >= cfg.SEARCH_PITCH_SHALLOW_DEG:
            return yaw, 0
        return yaw, pitch

    @staticmethod
    def _sdz(v: float, dz: float) -> float:
        if abs(v) <= dz:
            return 0.0
        return (1 if v > 0 else -1) * (abs(v) - dz) / (1 - dz)
