"""
tracker.py — PersonGimbalTracker: MissionRuntime orchestrator.

Wires all subsystems together and runs the main capture/detect/control loop.
All individual algorithms live in dedicated modules:
  - Detection + re-ID : detection/, tracking/target_selector.py
  - Gimbal FSM        : tracking/gimbal_state_machine.py
  - Operator input    : tracking/operator_input.py
  - Web UI callbacks  : gcs/web_control_adapter.py
  - Shared state      : tracking/tracker_state.py
  - Safety / MAVLink  : safety/, mavlink_client/
"""

import math
import os
import signal
import sys
import threading
import time
from typing import Optional

import cv2
import numpy as np

import config as cfg
from config.settings import Settings, load_settings
from control.drone_controller import DroneController
from control.pid_controller import PIDController, TargetSmoother
from control.search_patterns import (
    ExpandingSquareSearch,
    InitialScanSearch,
    LissajousSearch,
    SectorScanSearch,
)
from detection.detector import Detector
from gcs.stream_server import StreamServer
from gcs.web_control_adapter import WebControlAdapter
from gimbal.auto_zoom import AutoZoomController
from gimbal.siyi_controller import SIYIController
from mavlink_client import MAVLinkClient
from mission.hud_renderer import HudRenderer
from mission.recording_manager import RecordingManager
from mission.stream_adapter import StreamAdapter
from safety import SafetyMonitor
from safety.preflight import PreflightCheck
from tracking.gimbal_state_machine import GimbalStateMachine
from tracking.operator_input import OperatorInputController
from tracking.person_geolocation import CameraGeolocation, PersonEKF
from tracking.person_registry import PersonRegistry
from tracking.state_machine import State
from tracking.target_detection import TargetDetection
from tracking.target_selector import TargetSelector
from tracking.tracker_state import TrackerState
from tracking.velocity_tracker import VelocityTracker
from utils.flight_log import safe_event
from utils.frame_grabber import FrameGrabber
from utils import terminal
from utils.video_recorder import VideoRecorder


def _display_available() -> bool:
    if sys.platform.startswith("linux"):
        return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    return True


class PersonGimbalTracker:
    """Mission-runtime orchestrator — wires subsystems and runs the main loop.

    Args:
        drone_enabled: If True, MAVLink is connected and drone body follows.
        settings:      Injected Settings object (from main.py CLI parsing).
                       Defaults to load_settings() so the class is usable standalone.
    """

    def __init__(self, drone_enabled: bool = False, settings: Settings | None = None) -> None:
        self._drone_enabled = drone_enabled
        self._s = settings or load_settings()

        # --- Shared mutable state (read by all 5 extracted classes + HudRenderer) ---
        self._ts = TrackerState(
            tracking_enabled = True,
            search_enabled   = cfg.SEARCH_ENABLED,
            telem_adaptive_kp    = self._s.pid_yaw_kp,
            telem_adaptive_speed = self._s.max_gimbal_speed,
            running          = True,
        )

        # --- Hardware ---
        self.detector = Detector(self._s.model_path, settings=self._s)

        print("[SIYI] Connecting to A8 mini...")
        self.ctrl      = SIYIController(settings=self._s)
        self.zoom_ctrl = AutoZoomController(self.ctrl)

        self.pid_yaw   = PIDController(self._s.pid_yaw_kp,   self._s.pid_yaw_ki,   self._s.pid_yaw_kd,   output_limit=self._s.max_gimbal_speed)
        self.pid_pitch = PIDController(self._s.pid_pitch_kp, self._s.pid_pitch_ki, self._s.pid_pitch_kd, output_limit=self._s.max_gimbal_speed)
        self.smoother  = TargetSmoother()
        self.velocity  = VelocityTracker()

        self.search        = SectorScanSearch()
        self.init_scan     = InitialScanSearch()
        self.expand_search = ExpandingSquareSearch()
        self.lissajous     = LissajousSearch()

        self.grabber = FrameGrabber(settings=self._s)

        # --- Drone / safety subsystems ---
        self.safety  = SafetyMonitor(settings=self._s)
        self.geo     = CameraGeolocation(self._s.calibration_yaml)
        self.ekf     = PersonEKF()
        self.mav     = MAVLinkClient(safety=self.safety, settings=self._s)

        if drone_enabled:
            print("[Drone] Connecting to Orange Cube+ via MAVLink...")
            if self.mav.connect():
                self.mav.set_mode_guided()
            else:
                print("[Drone] MAVLink connection failed — reverting to gimbal-only mode")
                self._drone_enabled = False

        self.drone_ctrl = DroneController(
            mav=self.mav, safety=self.safety, geo=self.geo, ekf=self.ekf, settings=self._s
        )

        # S1.3 — preflight check provider for the web UI checklist.
        from mavlink_client import ParamVerifier   # local import: keeps gimbal-only paths clean
        self.preflight = PreflightCheck(
            mav=self.mav, safety=self.safety,
            ground_test=getattr(self._s, "ground_test", False),
            param_verifier=ParamVerifier(self.mav),
        )

        # --- Persistent person re-identification ---
        self.registry = PersonRegistry()

        # --- Extracted subsystems ---
        drone_ctrl_for_gsm = self.drone_ctrl if self._drone_enabled else None
        self._gsm = GimbalStateMachine(
            state         = self._ts,
            grabber       = self.grabber,
            pid_yaw       = self.pid_yaw,
            pid_pitch     = self.pid_pitch,
            smoother      = self.smoother,
            velocity      = self.velocity,
            ctrl          = self.ctrl,
            zoom_ctrl     = self.zoom_ctrl,
            search        = self.search,
            init_scan     = self.init_scan,
            expand_search = self.expand_search,
            lissajous     = self.lissajous,
            drone_ctrl    = drone_ctrl_for_gsm,
        )

        self._selector = TargetSelector(self.registry)

        self._web_ctrl = WebControlAdapter(
            state        = self._ts,
            ctrl         = self.ctrl,
            zoom_ctrl    = self.zoom_ctrl,
            grabber      = self.grabber,
            enter_manual = self._enter_manual,
            enter_auto   = self._enter_auto,
            request_brake = self._request_brake,
            request_land  = self._request_land,
            request_rtl   = self._request_rtl,
            manual_gimbal_speed = self._s.manual_gimbal_speed,
        )

        # --- Video recorder ---
        _script_dir    = os.path.dirname(os.path.abspath(__file__))
        self.recorder  = VideoRecorder(os.path.join(_script_dir, "recordings"))
        self._rec_thread: Optional[threading.Thread] = None

        # --- Live stream server ---
        self.stream    = StreamServer(settings=self._s) if self._s.stream_enabled else None
        self._stream_thread: Optional[threading.Thread] = None

        # --- Mission sub-units ---
        self.hud = HudRenderer(self)
        self._rec_mgr = RecordingManager(
            grabber    = self.grabber,
            recorder   = self.recorder,
            draw_fn    = self.hud.draw,
            get_target = lambda: self._latest_target_info,
            is_running = lambda: self._ts.running,
        )
        self._stream_adp = StreamAdapter(
            grabber    = self.grabber,
            stream     = self.stream,
            draw_fn    = self.hud.draw,
            get_target = lambda: self._latest_target_info,
            is_running = lambda: self._ts.running,
        )

        # --- OperatorInputController (stdin + keyboard) ---
        # SSH parity: the four handle_* callbacks expose the same actions
        # the browser hits over HTTP (E-STOP, arm/disarm, preflight,
        # telemetry) so the SSH stdin loop can drive them directly.
        self._op_input = OperatorInputController(
            state        = self._ts,
            ctrl         = self.ctrl,
            zoom_ctrl    = self.zoom_ctrl,
            stream       = self.stream,
            recorder     = self.recorder,
            grabber      = self.grabber,
            enter_manual = self._enter_manual,
            enter_auto   = self._enter_auto,
            request_land = self._request_land,
            request_rtl  = self._request_rtl,
            request_brake = self._request_brake,
            reset_all    = self._gsm.reset_all,
            search       = self.search,
            init_scan    = self.init_scan,
            expand_search = self.expand_search,
            lissajous    = self.lissajous,
            handle_estop     = self._handle_estop,
            handle_follow    = self._handle_follow,
            handle_takeoff   = self._handle_takeoff,
            handle_preflight = self._handle_preflight,
            handle_telemetry = self._handle_telemetry,
        )
        self._cmd_thread = self._op_input.start()

        # --- Display ---
        self.show_display: bool = _display_available()
        if not self.show_display:
            print("[Display] No X11 display — running headless")

        # --- Misc runtime state ---
        self._last_id_report:     float = 0.0
        self._last_id_signature:  tuple = ()    # (frozenset of IDs, lock_id) — dedup [IDs] prints
        self._last_battery_poll:  float = 0.0
        self._last_battery_print: float = 0.0
        self._last_gimbal_warn:   float = 0.0
        self._last_drone_cmd:     float = 0.0
        self._drone_cmd_interval: float = 1.0 / self._s.drone_cmd_rate_hz

        self.frame_count: int   = 0
        self.fps:         float = 0.0
        self._fps_timer:  float = time.time()

        self._latest_target_info = None

    # ------------------------------------------------------------------
    #  Property delegates — HudRenderer and tests read these off the tracker
    # ------------------------------------------------------------------

    @property
    def mode(self) -> str:
        return self._ts.mode

    @property
    def state(self) -> str:
        return self._ts.state

    @property
    def _detected_ids(self) -> dict:
        return self._ts.detected_ids

    @property
    def _lock_id(self) -> Optional[int]:
        return self._ts.lock_id

    @property
    def tracking_enabled(self) -> bool:
        return self._ts.tracking_enabled

    @tracking_enabled.setter
    def tracking_enabled(self, v: bool) -> None:
        self._ts.tracking_enabled = v

    @property
    def search_enabled(self) -> bool:
        return self._ts.search_enabled

    @search_enabled.setter
    def search_enabled(self, v: bool) -> None:
        self._ts.search_enabled = v

    @property
    def target_lost_time(self) -> Optional[float]:
        return self._gsm._target_lost_time

    @property
    def telem_error_x(self) -> float:       return self._ts.telem_error_x
    @property
    def telem_error_y(self) -> float:       return self._ts.telem_error_y
    @property
    def telem_yaw_cmd(self) -> int:         return self._ts.telem_yaw_cmd
    @property
    def telem_pitch_cmd(self) -> int:       return self._ts.telem_pitch_cmd
    @property
    def telem_adaptive_kp(self) -> float:   return self._ts.telem_adaptive_kp
    @property
    def telem_adaptive_speed(self) -> float: return self._ts.telem_adaptive_speed
    @property
    def telem_bbox_ratio(self) -> float:    return self._ts.telem_bbox_ratio

    # ------------------------------------------------------------------
    #  Mode transitions  (called by WebControlAdapter + OperatorInputController)
    # ------------------------------------------------------------------

    def _enter_manual(self) -> None:
        """Switch to MANUAL gimbal-control mode. Safe to call mid-flight."""
        self._stop_drone_body_autonomy("MANUAL")
        if self._drone_enabled and self.mav.is_connected():
            self.mav.send_zero_velocity()
        self._gsm.reset_all()
        self._ts.manual_yaw_speed   = 0
        self._ts.manual_pitch_speed = 0
        self._ts.manual_key_t       = 0.0
        self._ts.state              = State.WAITING
        self._ts.mode               = "MANUAL"
        print()
        print("[Mode] ═══════════════════════════════════════════")
        print("[Mode]  *** MANUAL MODE — Gimbal under operator control ***")
        print("[Mode]  Keyboard : Arrow keys ←↑↓→ to pan/tilt")
        print("[Mode]  Terminal : pan <-100..100>  |  tilt <-100..100>  |  stop")
        print("[Mode]  Web UI   : D-pad buttons (hold to move, release to stop)")
        print("[Mode]  Return   : press 'm'  or  type 'mode auto'")
        print("[Mode]  Drone    : holding position (zero velocity)")
        print("[Mode] ═══════════════════════════════════════════")

    def _enter_auto(self) -> None:
        """Switch to AUTO (autonomous) tracking mode."""
        self._ts.manual_yaw_speed   = 0
        self._ts.manual_pitch_speed = 0
        self._ts.manual_key_t       = 0.0
        self.ctrl.stop()
        self._ts.mode = "AUTO"
        self._gsm.reset_all()
        print()
        print("[Mode] ═══════════════════════════════════════════")
        print("[Mode]  *** AUTO MODE — Autonomous person tracking active ***")
        print(f"[Mode]  Drone following: {'ON' if self._drone_enabled else 'OFF'}")
        if self._ts.lock_id is not None:
            print(f"[Mode]  Target lock: ID {self._ts.lock_id} (type 'unlock' to release)")
        else:
            print("[Mode]  Target lock: NONE — type 'ids' then 'track <id>' to lock")
        print("[Mode] ═══════════════════════════════════════════")

    # ------------------------------------------------------------------
    #  Software E-STOP  (S1.1)
    # ------------------------------------------------------------------

    def _handle_estop(self, action: str) -> dict:
        """Execute the operator-triggered E-STOP.

        action: "brake" on first press; "land" on a double-tap within
                _ESTOP_DOUBLE_TAP_S of a prior press.

        Always stops the autonomous tracker, regardless of MAVLink state,
        so the gimbal stops chasing even if the FCU is unreachable.
        Returns a dict echoed back to the browser.
        """
        safe_event("estop", action=action, drone_enabled=self._drone_enabled)
        try:
            self._ts.tracking_enabled = False
            # B2: emit the unfollow event from here when E-STOP is what
            # actually stops body-follow. A subsequent operator
            # 'unfollow' command therefore won't double-log the same
            # transition (see _handle_follow idempotency below).
            if self._ts.drone_following:
                self._ts.drone_following = False
                safe_event("unfollow", reason="estop", action=action)
            self.drone_ctrl.reset()
            # B3: stop manual gimbal motion. Without this an operator
            # mid-'pan 50' in MANUAL mode would keep panning after the
            # estop fired — surprising and arguably unsafe near a person.
            self._ts.manual_yaw_speed   = 0
            self._ts.manual_pitch_speed = 0
            self._ts.manual_key_t       = 0.0
            try:
                self.ctrl.stop()
            except Exception as exc:
                print(f"[ESTOP] gimbal stop error: {exc}")
        except Exception as exc:
            print(f"[ESTOP] tracker reset error: {exc}")

        if not self._drone_enabled or not self.mav.is_connected():
            msg = "MAVLink not connected — tracker disabled but FCU unreachable"
            print(f"[ESTOP] {msg}")
            return {"status": "error", "action": action, "msg": msg}

        ok = self.mav.send_brake() if action == "brake" else self.mav.send_land()
        if not ok and action == "brake":
            print("[ESTOP] BRAKE not ACKed — falling back to LAND")
            ok = self.mav.send_land()
            action = "land"
        return {
            "status": "ok" if ok else "error",
            "action": action,
            "msg": "" if ok else f"FCU did not ACK {action.upper()}",
        }

    def _stop_drone_body_autonomy(self, reason: str) -> None:
        """Disable body-following until the operator explicitly follows again."""
        if self._ts.drone_following:
            print(f"[Follow] Drone-body following STOPPED by {reason}")
        self._ts.drone_following = False
        try:
            self.drone_ctrl.reset()
        except Exception as exc:
            print(f"[{reason}] tracker reset error: {exc}")

    def _request_safety_mode(self, action: str, send_fn) -> dict:
        """Shared terminal safety-mode handler for BRAKE/LAND/RTL."""
        self._stop_drone_body_autonomy(action)
        if not self._drone_enabled:
            msg = "tracker launched without --drone"
            print(f"[{action}] {msg}")
            return {"status": "error", "action": action.lower(), "msg": msg}

        if not self.mav.is_connected():
            msg = f"MAVLink not connected — {action} not sent"
            print(f"[{action}] {msg}")
            return {"status": "error", "action": action.lower(), "msg": msg}

        self.mav.send_zero_velocity()
        ok = send_fn()
        return {
            "status": "ok" if ok else "error",
            "action": action.lower(),
            "msg": "" if ok else f"FCU did not ACK {action}",
        }

    def _request_brake(self) -> dict:
        """Operator BRAKE request used by the terminal ``mode brake`` command."""
        return self._request_safety_mode("BRAKE", self.mav.send_brake)

    def _request_land(self) -> dict:
        """Operator LAND request used by the terminal ``mode land`` command."""
        return self._request_safety_mode("LAND", self.mav.send_land)

    def _request_rtl(self) -> dict:
        """Operator RTL request used by the terminal ``mode rtl`` command."""
        return self._request_safety_mode("RTL", self.mav.send_rtl)

    # ------------------------------------------------------------------
    #  Preflight + arm  (S1.3)
    # ------------------------------------------------------------------

    def _handle_preflight(self) -> list:
        """Return the live preflight checklist as a list of dicts."""
        return [item.to_dict() for item in self.preflight.run()]

    def _handle_follow(self, on) -> dict:
        """Enable or stop drone-body following of the locked person.

        This is the tracker-side authority gate — distinct from FCU arming
        (which the RC pilot does via the sticks gesture). Renamed from
        `_handle_follow` to remove the FCU-arm collision that confused
        operators in earlier field tests.

        Args:
            on: True to start following, False to stop, None to query current state.

        Returns:
            {"following": bool, "status": "ok"|"error", "msg": str, "items": [...]}.
            Refuses to start following if any preflight item is failing or
            --drone was not specified at launch.
        """
        if on is None:
            return {"following": bool(self._ts.drone_following), "status": "ok", "msg": ""}

        if not on:
            # B2: idempotent unfollow. Only print + log when actually
            # transitioning from following → stopped, so repeated 'unfollow'
            # commands (or an 'unfollow' after an E-STOP that already
            # stopped) don't pollute the flight log with phantom events.
            was_following = self._ts.drone_following
            if was_following:
                self._stop_drone_body_autonomy("operator")
                if self._drone_enabled and self.mav.is_connected():
                    self.mav.send_zero_velocity()
                safe_event("unfollow", source="operator")
                return {"following": False, "status": "ok", "msg": "stopped"}
            self._ts.drone_following = False
            return {"following": False, "status": "ok", "msg": "already stopped"}

        if not self._drone_enabled:
            return {"following": False, "status": "error",
                    "msg": "tracker launched without --drone"}

        # Idempotency: already following.
        # If the RC-override latch is active (pilot switched modes and back),
        # re-typing 'follow' is the operator's explicit intent to resume —
        # clear the latch so the drone controller can send commands again.
        # Web UI polling that hits this path when not latched is harmless.
        if self._ts.drone_following:
            if self.mav.is_rc_override_active() or self.mav.get_mode() != "GUIDED":
                self.mav.clear_rc_override()
                if self.mav.get_mode() != "GUIDED":
                    print(f"[Follow] FCU in {self.mav.get_mode()} — switching to GUIDED")
                    self.mav.set_mode_guided()
                # Reset drone controller completely: clears stale EKF position,
                # smoother state, and origin so fresh GPS is used as reference.
                # Without this, the EKF retains a position from before the LOITER
                # and every new camera projection is rejected as a >10 m jump,
                # causing the drone to fly in one fixed wrong direction.
                self.drone_ctrl.reset()
                self.drone_ctrl.clear_origin()
                print("[Follow] Latch cleared, GUIDED re-entered, EKF reset — resuming")
                return {"following": True, "status": "ok", "msg": "resumed"}
            return {"following": True, "status": "ok", "msg": "already following"}

        items = self.preflight.run()
        # When the FCU is already armed, ArduPilot has already verified GPS
        # lock and HOME position (it will not arm without them).  Treat
        # outdoor_only failures as advisory rather than blocking so the
        # operator does not need to wait for the Jetson's own EKF stream to
        # fill in after the FCU is already flying.
        fcu_armed = self.mav.is_armed()
        bad = [c for c in items if not c.ok and (not c.outdoor_only or not fcu_armed)]
        if bad:
            failing = ", ".join(c.name for c in bad)
            return {"following": False, "status": "error",
                    "msg": f"preflight failing: {failing}",
                    "items": [c.to_dict() for c in items]}

        # Successfully enabling follow clears the RC-override latch (S3.6)
        # so the controller can resume issuing commands.
        self.mav.clear_rc_override()
        self._ts.drone_following = True
        skipped = [c for c in items if not c.ok and c.outdoor_only and fcu_armed]
        if skipped:
            names = ", ".join(c.name for c in skipped)
            print(f"[Follow] Drone-body following ENABLED — GPS/HOME bypassed (FCU armed): {names}")
        else:
            print("[Follow] Drone-body following ENABLED — preflight all green")
        safe_event("follow", checks_passed=len(items) - len(skipped))
        if self._ts.lock_id is None:
            print("[Follow] Hint: no target locked — drone will hover until you type "
                  "'ids' then 'track <id>'")
        return {"following": True, "status": "ok", "msg": "following",
                "items": [c.to_dict() for c in items]}

    def _handle_takeoff(self, altitude_m: Optional[float] = None) -> dict:
        """Command the FCU to take off to ``altitude_m`` AGL.

        Args:
            altitude_m: Target altitude in metres AGL, or None to use
                ``cfg.DEFAULT_TAKEOFF_ALT_M`` (7.0 m).

        Returns:
            ``{"status": "ok"|"error", "msg": str, "altitude_m": float,
            "items": [...]}``. ``items`` is included on preflight
            failure to mirror ``_handle_follow``.

        Refuses unless ``--drone`` was passed, preflight passes, and
        the FCU reports LANDED. Same preflight gate as ``_handle_follow``
        — covers MAVLink link, ARMED, GUIDED, HOME, GPS, sensors,
        battery, RC link, params, and fence. (Preflight intentionally
        skips ARMED/GUIDED in ``--ground-test`` so bench rehearsal
        works.) If the FCU is not in GUIDED, switches to GUIDED first
        and gives the heartbeat a moment to refresh the local mode
        cache before preflight runs.

        Altitudes below ``MIN_ALT_M`` are accepted for hover testing
        but the response includes a warning, since the follow
        controller's SAFETY-CRITICAL floor will climb the drone up to
        ``MIN_ALT_M`` once the tracker is armed.
        """
        alt = float(altitude_m) if altitude_m is not None else cfg.DEFAULT_TAKEOFF_ALT_M

        if not (cfg.MIN_TAKEOFF_ALT_M <= alt <= cfg.MAX_TAKEOFF_ALT_M):
            return {"status": "error", "altitude_m": alt,
                    "msg": f"altitude {alt:.1f} m outside "
                           f"[{cfg.MIN_TAKEOFF_ALT_M}, {cfg.MAX_TAKEOFF_ALT_M}] m"}

        if not self._drone_enabled:
            return {"status": "error", "altitude_m": alt,
                    "msg": "tracker launched without --drone"}

        # Auto-switch to GUIDED BEFORE preflight, since preflight gates
        # on the cached FCU mode. Trust the ACK; sleep briefly to give
        # the heartbeat rx_loop a chance to refresh `_mode` before
        # preflight reads it. No-op when already in GUIDED.
        if self.mav.get_mode() != "GUIDED":
            print(f"[Takeoff] FCU mode is {self.mav.get_mode()} — switching to GUIDED")
            if not self.mav.set_mode_guided():
                return {"status": "error", "altitude_m": alt,
                        "msg": "failed to set GUIDED mode"}
            time.sleep(0.5)   # let heartbeat refresh the local mode cache

        items = self.preflight.run()
        fcu_armed = self.mav.is_armed()
        bad = [c for c in items if not c.ok and (not c.outdoor_only or not fcu_armed)]
        if bad:
            failing = ", ".join(c.name for c in bad)
            return {"status": "error", "altitude_m": alt,
                    "msg": f"preflight failing: {failing}",
                    "items": [c.to_dict() for c in items]}

        # Takeoff-specific gate: must be on the ground. Not part of
        # preflight because in-flight `arm` is legitimate.
        if not self.mav.is_landed():
            return {"status": "error", "altitude_m": alt,
                    "msg": "FCU does not report landed — refusing takeoff in flight"}

        print(f"[Takeoff] Commanding takeoff to {alt:.1f} m AGL")
        if not self.mav.send_takeoff(alt):
            return {"status": "error", "altitude_m": alt,
                    "msg": "FCU did not ACK NAV_TAKEOFF"}

        safe_event("takeoff", altitude_m=alt)

        # Soft warning: alt below the follow controller's hard floor
        # will trigger an immediate climb to MIN_ALT_M on tracker arm.
        warn = ""
        if alt < cfg.MIN_ALT_M:
            warn = (f" — WARNING: {alt:.1f} m is below MIN_ALT_M "
                    f"({cfg.MIN_ALT_M} m); arming the tracker will "
                    f"climb to the floor")
            print(f"[Takeoff]{warn}")
        return {"status": "ok", "altitude_m": alt,
                "msg": f"takeoff to {alt:.1f} m commanded{warn}"}

    # ------------------------------------------------------------------
    #  Live telemetry (S3.1)
    # ------------------------------------------------------------------

    def _handle_telemetry(self) -> dict:
        """Return live failsafe + flight telemetry for the /status payload.

        All fields are best-effort; missing data is returned as None so the
        web UI can render a stable layout regardless of subsystem readiness.
        """
        import time as _t
        out: dict = {
            "drone_enabled": bool(self._drone_enabled),
            "drone_following":   bool(self._ts.drone_following),
            "mode_tracker":  str(self._ts.mode),
        }
        try:
            if self._drone_enabled and self.mav is not None:
                out["mavlink"]     = bool(self.mav.is_connected())
                out["mode_fcu"]    = self.mav.get_mode()
                out["armed_fcu"]   = bool(self.mav.is_armed())
                out["gps_fix"]     = int(self.mav.get_gps_fix())
                out["gps_hdop"]    = round(float(self.mav.get_gps_hdop()), 2)
                out["gps_sats"]    = int(self.mav.get_sat_count())
                out["ekf_var"]     = round(
                    float(self.mav.get_ekf_horizontal_variance()), 3
                )
                out["battery_v"]   = round(float(self.mav.get_battery_voltage()), 2)
                out["fence_breach"] = bool(self.mav.is_fence_breached())
                out["rc_override"] = bool(self.mav.is_rc_override_active())
                out["rc_connected"] = bool(self.mav.is_rc_connected())
                out["rc_rssi"]     = int(self.mav.get_rc_rssi())
                out["home_set"]    = bool(self.mav.is_home_set())
                out["sensors_ok"]  = bool(self.mav.is_sensors_healthy())
                out["ground_test"] = bool(self.mav.is_ground_test())
                out["landed_fcu"]  = bool(self.mav.is_landed())
                out["landed_state"] = self.mav.get_landed_state()
        except Exception as exc:
            out["mavlink_error"] = str(exc)

        try:
            now = _t.monotonic()
            last_det = self.drone_ctrl._last_detection
            out["tracking_loss_s"] = round(now - last_det, 2) if last_det else None
            out["fps"]             = round(self.drone_ctrl.effective_fps(), 1)
            out["body_confirmed"]  = bool(self.drone_ctrl.is_body_confirmed())
            out["person_protection"] = self.drone_ctrl.get_protection_status()
        except Exception:
            pass
        return out

    # ------------------------------------------------------------------
    #  Stream / recorder / overlay delegates
    # ------------------------------------------------------------------

    def _stream_loop(self) -> None:
        self._stream_adp.loop()

    def _recorder_loop(self) -> None:
        self._rec_mgr.loop()

    def _draw_overlay(self, frame: np.ndarray, target_info: Optional[TargetDetection]) -> None:
        self.hud.draw(frame, target_info)

    def _update_drone_outer_loop(
        self,
        now: float,
        person_detected: bool,
        target_info: Optional[TargetDetection],
    ) -> None:
        """Run the 10 Hz drone safety/follow loop.

        The loop must continue even if SIYI attitude telemetry is stale so
        tracking-loss, battery, RC, and geofence failsafes still execute.
        Stale attitude only suppresses fresh EKF measurement updates.
        """
        if not self._drone_enabled or now - self._last_drone_cmd < self._drone_cmd_interval:
            return

        self._last_drone_cmd = now
        ts = self._ts
        fresh_target = target_info if (target_info is not None and target_info.is_fresh) else None
        detected_for_drone = fresh_target is not None and ts.mode == "AUTO"
        # H4: drone body only moves toward the explicitly operator-locked target.
        # Without a lock, the gimbal still tracks whoever it sees, but the drone
        # hovers — prevents following the wrong person before operator confirms.
        locked_target = fresh_target if (detected_for_drone and ts.lock_id is not None) else None
        self.drone_ctrl.notify_detection(locked_target is not None)

        attitude_fresh = self.ctrl.attitude_is_fresh()
        pan_deg = self.ctrl.gimbal_pan_deg
        tilt_deg = self.ctrl.gimbal_tilt_deg
        if attitude_fresh:
            self.drone_ctrl.set_gimbal_angles(
                pan_rad=math.radians(pan_deg),
                tilt_rad=math.radians(tilt_deg),
            )
        else:
            import logging as _log
            _log.debug("[Drone] Gimbal telemetry stale — running failsafes with fallback angles")

        pan_correction = self.drone_ctrl.update(
            gimbal_pan_deg=pan_deg,
            gimbal_tilt_deg=tilt_deg,
            target_info=locked_target if attitude_fresh else None,
            drone_tracking_enabled=(
                ts.tracking_enabled
                and ts.drone_following
                and ts.mode == "AUTO"
            ),
        )
        if pan_correction != 0.0 and ts.state == State.TRACKING:
            correction_speed = int(pan_correction * (180.0 / math.pi))
            self.ctrl.set_speed(
                max(-100, min(100, ts.telem_yaw_cmd + correction_speed)),
                ts.telem_pitch_cmd,
            )

    # ------------------------------------------------------------------
    #  Safe shutdown
    # ------------------------------------------------------------------

    def _return_home(self) -> None:
        print("[Home] Stopping gimbal and returning to home position...")
        try:
            self.ctrl.stop()
            time.sleep(0.05)
            for _ in range(3):
                self.ctrl.center()
                time.sleep(0.08)
            print("[Home] Center command sent")
        except Exception as exc:
            print(f"[Home] Warning: {exc}")

    # ------------------------------------------------------------------
    #  Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Start the tracking loop. Blocks until quit."""
        if not self.grabber.start():
            return

        self.ctrl.test_zoom()
        self.drone_ctrl.set_frame_size(self.grabber.frame_w, self.grabber.frame_h)

        print("=" * 62)
        print("  TRACKER RUNNING — SIYI A8 mini  v10.0-modular")
        drone_label = "ON (MAVLink)" if self._drone_enabled else "OFF (gimbal-only)"
        print(f"  Drone body tracking: {drone_label}")
        print(f"  States: TRACK → PREDICT(3s) → FADE(3s) → SEARCH")
        print(f"  Search: INIT-SCAN → SECTOR(15s) → EXP-SQ(45s) → LISSAJOUS")
        print(f"  Keys: q=Quit  t=Track  r=Center  s=Search  d=Display  i=Scan")
        print(f"        v=Record  l=Stream  +/-=Kp  [/]=Speed  z/x=Zoom  a=AutoZoom")
        print(f"        m=Manual/Auto  (MANUAL: ←↑↓→ arrow keys pan/tilt gimbal)")
        print("=" * 62)

        self._rec_thread = threading.Thread(
            target=self._recorder_loop, daemon=True, name="RecorderThread")
        self._rec_thread.start()

        if self.stream:
            self.stream.start()
            self.stream.set_click_callback(self._web_ctrl.handle_click)
            self.stream.set_zoom_callback(self._web_ctrl.handle_zoom)
            self.stream.set_mode_callback(self._web_ctrl.handle_mode)
            self.stream.set_gimbal_callback(self._web_ctrl.handle_gimbal)
            self.stream.set_estop_callback(self._handle_estop)
            self.stream.set_preflight_callback(self._handle_preflight)
            self.stream.set_follow_callback(self._handle_follow)
            self.stream.set_takeoff_callback(self._handle_takeoff)
            self.stream.set_telemetry_callback(self._handle_telemetry)
        self._stream_thread = threading.Thread(
            target=self._stream_loop, daemon=True, name="StreamThread")
        self._stream_thread.start()

        # Initial acquisition scan
        if self._ts.tracking_enabled:
            self._ts.state = State.INITIAL_SCAN
            self.init_scan.start()

        if cfg.AUTO_RECORD:
            self.recorder.start(self.grabber.frame_w, self.grabber.frame_h, cfg.RECORD_FPS)

        def _sigterm(signum, frame):
            print("\n[Signal] SIGTERM — shutting down cleanly...")
            self._ts.running = False

        signal.signal(signal.SIGTERM, _sigterm)

        try:
            while self._ts.running:
                now = time.time()
                ts  = self._ts

                # Manual zoom auto-stop
                if ts.manual_zoom_stop_at > 0 and now >= ts.manual_zoom_stop_at:
                    self.ctrl.zoom_stop()
                    ts.manual_zoom_stop_at = 0

                # Battery poll (gimbal)
                if now - self._last_battery_poll >= cfg.BATTERY_POLL_INTERVAL:
                    self._last_battery_poll = now
                    self.ctrl.request_battery_status()
                if (now - self._last_battery_print >= cfg.BATTERY_PRINT_INTERVAL
                        and self.ctrl.battery_voltage > 0.0):
                    self._last_battery_print = now
                    v   = self.ctrl.battery_voltage
                    pw  = self.ctrl.power_watts
                    ima = self.ctrl.current_ma
                    print(f"[Power] Voltage: {v:.2f}V  |  Current: {ima}mA  |  Power: {pw:.2f}W")

                # Gimbal UDP watchdog
                if not self.ctrl.is_responding:
                    if now - self._last_gimbal_warn >= cfg.GIMBAL_WATCHDOG_S:
                        self._last_gimbal_warn = now
                        print(f"[SIYI] WARNING: No UDP response from gimbal "
                              f"(>{cfg.GIMBAL_WATCHDOG_S:.0f}s) — check network/cable")

                frame, is_new = self.grabber.get_frame()
                if frame is None:
                    if not self.grabber.is_connected():
                        self._gsm.reset_all()
                        if self._drone_enabled:
                            self.mav.send_zero_velocity()
                    time.sleep(0.01)
                    if self.show_display:
                        raw_key = cv2.waitKey(1)
                        if ts.mode == "MANUAL":
                            self._op_input.handle_manual_key(raw_key)
                        if (raw_key & 0xFF) == ord("q"):
                            break
                    continue

                if not is_new:
                    if not self.grabber.is_connected():
                        self._gsm.reset_all()
                        if self._drone_enabled:
                            self.mav.send_zero_velocity()
                    if self.show_display:
                        raw_key = cv2.waitKey(1)
                        key     = raw_key & 0xFF
                        if ts.mode == "MANUAL":
                            self._op_input.handle_manual_key(raw_key)
                        self._op_input.handle_key(key)
                        if key == ord("d"):
                            self.show_display = not self.show_display
                            if not self.show_display:
                                cv2.destroyAllWindows()
                        if key == ord("q"):
                            break
                    else:
                        time.sleep(0.001)
                    continue

                # --- Detection with PersonRegistry ---
                target_info     = None
                person_detected = False
                if ts.tracking_enabled:
                    results         = self.detector.detect_and_track(
                        frame, cfg.CONF_THRESHOLD, cfg.IMGSZ
                    )
                    target_info     = self._selector.select(results, ts)
                    person_detected = target_info is not None
                    self._latest_target_info = target_info

                # Headless ID report — print only when the ID set or lock state
                # changes, plus a heartbeat every 30s if anything is detected,
                # so logs stay greppable instead of repeating per-frame.
                if not self.show_display:
                    now_r = time.time()
                    id_snap = dict(ts.detected_ids)
                    signature = (frozenset(id_snap.keys()), ts.lock_id)
                    changed = signature != self._last_id_signature
                    heartbeat = (
                        id_snap
                        and now_r - self._last_id_report >= max(30.0, cfg.ID_REPORT_INTERVAL * 15)
                    )
                    if id_snap and (changed or heartbeat):
                        self._last_id_report = now_r
                        self._last_id_signature = signature
                        id_str   = "  ".join(
                            f"ID {t}@({d.cx:.0f},{d.cy:.0f})" for t, d in id_snap.items())
                        lock_str = (f"  [locked={ts.lock_id}]"
                                    if ts.lock_id is not None else "")
                        terminal.status(f"[IDs] {id_str}{lock_str}  | 'track <id>' to lock")
                    elif not id_snap and self._last_id_signature:
                        # IDs went from "something" to nothing — note it once.
                        self._last_id_signature = ()  # falsy → won't re-trigger until next detection
                        terminal.event("[IDs] (none detected)")

                # --- Gimbal state machine (inner loop ~30 Hz) ---
                if ts.mode == "AUTO":
                    self._gsm.update(person_detected, target_info)
                else:
                    # MANUAL: apply operator-commanded speeds with auto-stop
                    if now >= ts.manual_key_t:
                        ts.manual_yaw_speed   = 0
                        ts.manual_pitch_speed = 0
                    self.ctrl.set_speed(ts.manual_yaw_speed, ts.manual_pitch_speed)

                # --- Drone outer loop (10 Hz) ---
                self._update_drone_outer_loop(now, person_detected, target_info)

                # --- FPS ---
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    el       = time.time() - self._fps_timer
                    self.fps = 30 / el if el > 0 else 0
                    self._fps_timer = time.time()

                # --- Local display (HDMI window, optional) ---
                if self.show_display:
                    disp = cv2.resize(frame, (cfg.DISPLAY_WIDTH, cfg.DISPLAY_HEIGHT))
                    self._draw_overlay(disp, target_info)
                    cv2.imshow("A8mini Tracker v10-modular", disp)
                    raw_key = cv2.waitKey(1)
                    key     = raw_key & 0xFF
                    if ts.mode == "MANUAL":
                        self._op_input.handle_manual_key(raw_key)
                    self._op_input.handle_key(key)
                    if key == ord("d"):
                        self.show_display = not self.show_display
                        if not self.show_display:
                            cv2.destroyAllWindows()
                    if key == ord("q"):
                        break

        finally:
            self._ts.running = False
            print("[Cleanup] Shutting down...")
            if self._stream_thread:
                self._stream_thread.join(timeout=2.0)
            if self._rec_thread:
                self._rec_thread.join(timeout=3.0)
            self.recorder.stop()
            if self.stream:
                self.stream.stop()
            self._return_home()
            if self._drone_enabled:
                self.mav.send_zero_velocity()
                self.mav.close()
            self.ctrl.close()
            self.grabber.stop()
            if self.show_display:
                cv2.destroyAllWindows()
            print("[Done]")
