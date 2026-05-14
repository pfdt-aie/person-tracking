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
from utils.frame_grabber import FrameGrabber
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
            handle_arm       = self._handle_arm,
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
        from utils.flight_log import get_flight_log
        get_flight_log().event("estop", action=action,
                               drone_enabled=self._drone_enabled)
        try:
            self._ts.tracking_enabled = False
            self._ts.drone_armed = False
            self.drone_ctrl.reset()
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
        """Disable body-following until the operator explicitly arms again."""
        if self._ts.drone_armed:
            print(f"[Arm] Drone-body tracker DISARMED by {reason}")
        self._ts.drone_armed = False
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

    def _handle_arm(self, on) -> dict:
        """Arm or disarm the drone-body tracker.

        Args:
            on: True to arm, False to disarm, None to query current state.

        Returns:
            {"armed": bool, "status": "ok"|"error", "msg": str, "items": [...]}.
            Refuses to arm if any preflight item is failing or --drone was
            not specified at launch.
        """
        if on is None:
            return {"armed": bool(self._ts.drone_armed), "status": "ok", "msg": ""}

        if not on:
            self._ts.drone_armed = False
            print("[Arm] Drone-body tracker DISARMED by operator")
            from utils.flight_log import get_flight_log
            get_flight_log().event("disarm")
            return {"armed": False, "status": "ok", "msg": "disarmed"}

        if not self._drone_enabled:
            return {"armed": False, "status": "error",
                    "msg": "tracker launched without --drone"}

        items = self.preflight.run()
        if not all(c.ok for c in items):
            failing = ", ".join(c.name for c in items if not c.ok)
            return {"armed": False, "status": "error",
                    "msg": f"preflight failing: {failing}",
                    "items": [c.to_dict() for c in items]}

        # Successful arm clears the RC-override latch (S3.6) so the controller
        # can resume issuing commands.
        self.mav.clear_rc_override()
        self._ts.drone_armed = True
        print("[Arm] Drone-body tracker ARMED — preflight all green")
        from utils.flight_log import get_flight_log
        get_flight_log().event("arm", checks_passed=len(items))
        return {"armed": True, "status": "ok", "msg": "armed",
                "items": [c.to_dict() for c in items]}

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
            "drone_armed":   bool(self._ts.drone_armed),
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
            self.stream.set_arm_callback(self._handle_arm)
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

                # Headless ID report
                if not self.show_display:
                    now_r = time.time()
                    if now_r - self._last_id_report >= cfg.ID_REPORT_INTERVAL:
                        self._last_id_report = now_r
                        id_snap = dict(ts.detected_ids)
                        if id_snap:
                            id_str   = "  ".join(
                                f"ID {t}@({d.cx:.0f},{d.cy:.0f})" for t, d in id_snap.items())
                            lock_str = (f"  [locked={ts.lock_id}]"
                                        if ts.lock_id is not None else "")
                            print(f"[IDs] {id_str}{lock_str}  | 'track <id>' to lock")

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
                if self._drone_enabled and now - self._last_drone_cmd >= self._drone_cmd_interval:
                    self._last_drone_cmd = now
                    detected_for_drone = person_detected and ts.mode == "AUTO"
                    self.drone_ctrl.notify_detection(detected_for_drone)
                    if self.ctrl.attitude_is_fresh():
                        self.drone_ctrl.set_gimbal_angles(
                            pan_rad  = math.radians(self.ctrl.gimbal_pan_deg),
                            tilt_rad = math.radians(self.ctrl.gimbal_tilt_deg),
                        )
                        pan_correction = self.drone_ctrl.update(
                            gimbal_pan_deg         = self.ctrl.gimbal_pan_deg,
                            gimbal_tilt_deg        = self.ctrl.gimbal_tilt_deg,
                            target_info            = target_info,
                            drone_tracking_enabled = (
                                ts.tracking_enabled
                                and ts.drone_armed         # S1.3 preflight gate
                                and ts.mode == "AUTO"
                            ),
                        )
                        if pan_correction != 0.0 and ts.state == State.TRACKING:
                            correction_speed = int(pan_correction * (180.0 / math.pi))
                            self.ctrl.set_speed(
                                max(-100, min(100, ts.telem_yaw_cmd + correction_speed)),
                                ts.telem_pitch_cmd,
                            )
                    else:
                        import logging as _log
                        _log.debug("[Drone] Gimbal telemetry stale — skipping EKF update")

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
