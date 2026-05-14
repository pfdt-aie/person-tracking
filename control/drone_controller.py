"""
drone_controller.py — Hybrid gimbal/drone body control with full failsafe.

Architecture: Two-loop hybrid strategy
  Inner loop (30 Hz, tracker.py):   Gimbal PID centres person in frame.
  Outer loop (10 Hz, this module):  Drone body repositions when needed.

Drone movement is triggered when:
  - Gimbal pan > GIMBAL_PAN_SOFT_DEG  (person drifting off-centre in yaw)
  - Person GPS is known (EKF valid) and standoff distance error > 1 m

Velocity pipeline (per frame at 10 Hz):
  EKF person position → proportional error + feedforward velocity
      → EMA low-pass filter (reduce noise)
      → jerk limiter (reduce mechanical stress)
      → safety.check_velocity() (hard speed cap)
      → mavlink.send_position_velocity_ned()

Failsafe hierarchy (tracking loss):
  0–2s   Use EKF prediction (maintain motion, gimbal searching)
  2–5s   Zero velocity → drone decelerates to hover
  5–15s  LOITER command → drone holds position, GCS alert
  >15s   Stay in LOITER; operator decides. Never auto-RTL on tracking loss.

Battery critical (separate failsafe):
  battery_voltage < CELL_CRITICAL_MV * N_cells → RTL immediately.

Safety notes:
  - This module NEVER sends MAVLink commands if safety checks fail.
  - GPS loss → immediately zero velocity → LOITER.
  - Gimbal pan counter-compensation: when drone yaws to recenter,
    a complementary gimbal correction is computed and returned to tracker.py
    so the person does not jump in frame.
"""

import math
import threading
import time
from collections import deque
from typing import Optional

import numpy as np

from utils.flight_log import get_flight_log

import config as cfg
from config.settings import Settings, load_settings
from mavlink_client import MAVLinkClient
from tracking.person_geolocation import CameraGeolocation, PersonEKF
from tracking.target_detection import TargetDetection
from safety import SafetyMonitor


class DroneController:
    """Hybrid outer-loop drone body controller.

    Args:
        mav:    MAVLinkClient (already connected or not; checked before each send).
        safety: Shared SafetyMonitor instance.
        geo:    CameraGeolocation instance (shared with tracker).
        ekf:    PersonEKF instance (shared with tracker).
    """

    def __init__(
        self,
        mav: MAVLinkClient,
        safety: SafetyMonitor,
        geo: CameraGeolocation,
        ekf: PersonEKF,
        settings: Settings | None = None,
    ) -> None:
        self._s      = settings or load_settings()
        self._mav    = mav
        self._safety = safety
        self._geo    = geo
        self._ekf    = ekf

        # EKF origin (home position in GPS) — set when GPS is first available
        self._origin_lat: float = 0.0
        self._origin_lon: float = 0.0
        self._origin_set: bool  = False

        # EMA filter state
        self._ema_vn: float = 0.0
        self._ema_ve: float = 0.0

        # Jerk limiter state
        self._prev_vn: float = 0.0
        self._prev_ve: float = 0.0
        self._prev_an: float = 0.0
        self._prev_ae: float = 0.0

        # Timing
        self._last_update:     float = 0.0
        self._last_detection:  float = time.monotonic()
        self._loiter_issued:   bool  = False
        self._loiter_t:        float = 0.0
        self._rtl_issued:      bool  = False
        self._alert_issued:    bool  = False

        # S1.5 — body-movement confirmation counter.  Drone body stays at
        # zero velocity until BODY_MOVE_CONFIRM_FRAMES consecutive valid
        # detections.  Gimbal control is independent and tracks immediately.
        self._confirm_count: int = 0

        # S1.4 — retreat latch.  Once horizontal separation drops below
        # MIN_PERSON_DRONE_SEP_M, stay retreating until sep exceeds
        # MIN_PERSON_DRONE_SEP_M + RETREAT_HYSTERESIS_M.  Prevents flutter
        # at the boundary if the subject is walking toward the drone.
        self._retreating: bool = False
        self._last_person_sep_m: float | None = None
        self._last_vertical_clearance_m: float | None = None

        # S2.6 — standoff bearing hysteresis.  _vel_above_t is the monotonic
        # timestamp the person's velocity first crossed STANDOFF_VEL_THRESHOLD_MS
        # (-1.0 sentinel = below threshold). _bearing_rad is the slew-rate-
        # limited bearing used for the actual standoff vector.
        self._vel_above_t: float = -1.0
        self._bearing_rad: float = 0.0
        self._bearing_init: bool = False

        # S3.3 — session-time RTL. Timer starts on first armed-enable;
        # resets when the drone is disarmed (drone_tracking_enabled=False).
        self._session_start_t: float = -1.0
        self._session_rtl_issued: bool = False
        self._rc_loss_loiter_issued: bool = False

        # S3.5 — detection frame timestamps for FPS estimation.
        self._fps_times: deque = deque(maxlen=cfg.FPS_WINDOW_SIZE)
        self._fps_warned: bool = False

        # RTL retry state
        self._rtl_attempts:    int   = 0
        self._rtl_last_t:      float = 0.0
        self._rtl_confirmed:   bool  = False

        # Pre-check warning rate-limiters
        self._mode_warn_t:       float = 0.0
        self._sensor_warn_issued: bool = False
        self._target_fence_warn_t: float = 0.0

        # Initialised when set_gimbal_angles() is first called by tracker
        self._gimbal_pan_rad:  float = 0.0
        self._gimbal_tilt_rad: float = math.radians(cfg.GIMBAL_TILT_DEFAULT_DEG)

        # Thread lock for _last_detection (updated by tracker thread)
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    #  Public interface — called from tracker.py main loop at 10 Hz
    # ------------------------------------------------------------------

    def notify_detection(self, detected: bool) -> None:
        """Called every detection frame to update the tracking-loss timer.

        Args:
            detected: True if the person was detected this frame.
        """
        now = time.monotonic()
        with self._lock:
            self._fps_times.append(now)   # S3.5 — sample for FPS window
            if detected:
                self._last_detection  = now
                self._loiter_issued   = False
                self._alert_issued    = False
                if self._confirm_count < cfg.BODY_MOVE_CONFIRM_FRAMES:
                    self._confirm_count += 1
            else:
                self._confirm_count = 0

    def is_body_confirmed(self) -> bool:
        """True once BODY_MOVE_CONFIRM_FRAMES consecutive detections seen."""
        with self._lock:
            return self._confirm_count >= cfg.BODY_MOVE_CONFIRM_FRAMES

    def effective_fps(self) -> float:
        """Rolling estimate of detection-loop FPS (S3.5).

        Returns 0.0 until at least 2 samples are present so callers can
        decide whether the answer is meaningful yet.
        """
        with self._lock:
            if len(self._fps_times) < 2:
                return 0.0
            window = self._fps_times[-1] - self._fps_times[0]
            n = len(self._fps_times) - 1
        return n / window if window > 0 else 0.0

    def update(
        self,
        gimbal_pan_deg:  float,
        gimbal_tilt_deg: float,
        target_info:     Optional[TargetDetection],
        drone_tracking_enabled: bool,
    ) -> float:
        """Outer-loop update — compute and send drone velocity command.

        Call this at DRONE_CMD_RATE_HZ (10 Hz) regardless of detection status.
        Always sends a command so ArduPilot GUID_TIMEOUT does not trigger.

        Args:
            gimbal_pan_deg:          Current gimbal pan angle (degrees).
            gimbal_tilt_deg:         Current gimbal tilt angle (degrees).
            target_info:             YOLO target tuple from tracker, or None.
            drone_tracking_enabled:  False = tracking disabled (gimbal-only mode).

        Returns:
            gimbal_pan_correction (rad/s): Add to the gimbal PID yaw output
            to compensate for drone body yaw rotation so the person does
            not jump in frame.
        """
        if not drone_tracking_enabled:
            # S3.3 — reset the session timer the moment the operator disarms.
            self._session_start_t   = -1.0
            self._session_rtl_issued = False
            return 0.0

        now  = time.monotonic()
        dt   = now - self._last_update if self._last_update > 0 else 0.1
        self._last_update = now

        # S3.3 — session-time RTL. Caps the autonomous-tracking session at
        # MAX_FLIGHT_TIME_S so a forgotten run can never exceed safe battery.
        if self._session_start_t < 0:
            self._session_start_t = now
        if (now - self._session_start_t) >= cfg.MAX_FLIGHT_TIME_S:
            if not self._session_rtl_issued:
                print(f"[Drone] Session time {cfg.MAX_FLIGHT_TIME_S:.0f}s reached "
                      "— issuing RTL")
                get_flight_log().event(
                    "session_rtl",
                    elapsed_s=now - self._session_start_t,
                    limit_s=cfg.MAX_FLIGHT_TIME_S,
                )
                self._mav.send_rtl()
                self._session_rtl_issued = True
            return 0.0

        # Advance EKF state estimate every tick so velocity feedforward stays fresh
        # between detection updates (predict-only steps incur no measurement cost).
        self._ekf.predict(dt)

        # --- Safety pre-checks ---
        if not self._mav.is_connected():
            return 0.0

        if not self._safety.watchdog_heartbeat(self._mav.get_last_heartbeat_time()):
            print("[Drone] MAVLink heartbeat lost — stopping commands")
            return 0.0

        # --- A1: GUIDED mode gate ---
        if self._mav.get_mode() != "GUIDED":
            if now - self._mode_warn_t >= cfg.MODE_WARN_INTERVAL_S:
                self._mode_warn_t = now
                print(f"[Drone] Not in GUIDED ({self._mav.get_mode()}) — commands suppressed")
            return 0.0   # no send; GUID_TIMEOUT irrelevant outside GUIDED

        # --- S3.6: RC-override latch ---
        if self._mav.is_rc_override_active():
            if now - self._mode_warn_t >= cfg.MODE_WARN_INTERVAL_S:
                self._mode_warn_t = now
                print("[Drone] RC override latched — re-arm tracker to resume")
            return 0.0

        # --- S3.7: Runtime RC-loss gate ---
        # Preflight requires RC before takeoff; this keeps enforcing it after
        # arm. If the pilot link drops, stop autonomy and let ArduPilot hold.
        if not self._mav.is_rc_connected():
            self._mav.send_zero_velocity()
            if not self._rc_loss_loiter_issued:
                print("[Drone] RC link lost — LOITER issued, re-arm required")
                get_flight_log().event("rc_loss", action="loiter")
                self._mav.send_loiter()
                self._rc_loss_loiter_issued = True
            return 0.0
        self._rc_loss_loiter_issued = False

        # --- A2: ARM state gate ---
        if not self._mav.is_armed():
            return 0.0

        # --- A3: Home-position gate ---
        if not self._mav.is_home_set():
            self._mav.send_zero_velocity()   # keep GUID_TIMEOUT alive while waiting
            return 0.0

        # --- C3: Sensor health gate ---
        if not self._mav.is_sensors_healthy():
            if not self._sensor_warn_issued:
                print("[Drone] Critical sensors not healthy — suppressing commands")
                self._sensor_warn_issued = True
            self._mav.send_zero_velocity()
            return 0.0
        self._sensor_warn_issued = False

        # --- E2: Battery critical → RTL with fire-and-verify retry ---
        batt_v = self._mav.get_battery_voltage()
        if self._safety.is_battery_critical(batt_v):
            if not self._rtl_confirmed:
                if self._mav.get_mode() == "RTL":
                    self._rtl_confirmed = True
                    print("[Drone] RTL confirmed by flight controller")
                elif (not self._rtl_issued
                      or now - self._rtl_last_t >= cfg.RTL_RETRY_INTERVAL_S):
                    if self._rtl_attempts < cfg.RTL_MAX_ATTEMPTS:
                        self._mav.send_rtl()
                        self._rtl_issued    = True
                        self._rtl_last_t    = now
                        self._rtl_attempts += 1
                        print(f"[Drone] BATTERY CRITICAL — RTL attempt "
                              f"{self._rtl_attempts}/{cfg.RTL_MAX_ATTEMPTS}")
                    else:
                        print(f"[Drone] ⚠ RTL FAILED after {cfg.RTL_MAX_ATTEMPTS} "
                              "attempts — operator must intervene!")
            return 0.0

        # --- GPS check ---
        if not self._mav.is_gps_ok():
            print("[Drone] GPS lost — zero velocity")
            self._mav.send_zero_velocity()
            return 0.0

        # --- A4: Initialise EKF NED origin from HOME_POSITION (same source as geofence) ---
        if not self._origin_set and self._mav.is_home_set():
            home_lat, home_lon, _ = self._mav.get_home_position()
            self._origin_lat = home_lat
            self._origin_lon = home_lon
            self._origin_set = True

        # --- D1: Geofence check — return toward home instead of just stopping ---
        lat, lon, alt = self._mav.get_gps()
        if alt < cfg.MIN_ALT_M:
            self._mav.send_velocity_ned(0.0, 0.0, -cfg.RETREAT_SPEED_MS)  # vD<0 = climb
            print(f"[Drone] Geofence altitude low ({alt:.1f}m) — climbing")
            return 0.0
        if alt > cfg.MAX_ALT_M:
            self._mav.send_velocity_ned(0.0, 0.0, cfg.RETREAT_SPEED_MS)   # vD>0 = descend
            print(f"[Drone] Geofence altitude high ({alt:.1f}m) — descending")
            return 0.0
        if not self._safety.check_geofence(lat, lon, alt):
            if self._safety.is_home_set:
                fence_lat, fence_lon = self._safety.get_fence_centre()
                dn = math.radians(fence_lat - lat) * 6_371_000.0
                de = (math.radians(fence_lon - lon) * 6_371_000.0
                      * math.cos(math.radians(lat)))
                dist = math.sqrt(dn ** 2 + de ** 2)
                if dist > 1.0:
                    speed = min(cfg.GEOFENCE_RETURN_MAX_MS,
                                dist * cfg.GEOFENCE_RETURN_KP)
                    scale = speed / dist
                    self._mav.send_velocity_ned(dn * scale, de * scale, 0.0)
                    print(f"[Drone] Geofence breach — returning ({dist:.0f}m from home)")
                else:
                    self._mav.send_zero_velocity()
            else:
                self._mav.send_zero_velocity()
            return 0.0

        # --- C1: ArduPilot onboard fence breach ---
        if self._mav.is_fence_breached():
            print("[Drone] ArduPilot fence breach active — zero velocity")
            self._mav.send_zero_velocity()
            return 0.0

        # --- Update EKF with latest detection ---
        if target_info is not None and self._origin_set:
            self._update_ekf_from_detection(target_info)

        # --- Tracking-loss failsafe ---
        with self._lock:
            dt_lost = now - self._last_detection
        pan_correction = self._handle_tracking_loss(dt_lost, now)
        if pan_correction is not None:
            return pan_correction   # failsafe took control

        # --- S3.5: FPS floor ---
        # Once we have enough samples, hold the body if the detector loop
        # is too slow to drive commands safely. Gimbal continues tracking.
        fps = self.effective_fps()
        if fps > 0.0 and fps < cfg.MIN_TRACKING_FPS:
            if not self._fps_warned:
                print(f"[Drone] Detection FPS {fps:.1f} < "
                      f"{cfg.MIN_TRACKING_FPS:.1f} — body holding")
                get_flight_log().event(
                    "fps_floor", fps=fps, threshold=cfg.MIN_TRACKING_FPS,
                    state="hold",
                )
                self._fps_warned = True
            self._mav.send_zero_velocity()
            return 0.0
        elif fps >= cfg.MIN_TRACKING_FPS and self._fps_warned:
            print(f"[Drone] Detection FPS recovered ({fps:.1f}) — body resuming")
            get_flight_log().event(
                "fps_floor", fps=fps, threshold=cfg.MIN_TRACKING_FPS,
                state="resume",
            )
            self._fps_warned = False

        # --- S1.5: Body-movement confirmation gate ---
        # Require BODY_MOVE_CONFIRM_FRAMES consecutive detections before
        # any drone body velocity is sent. Holds hover otherwise. Gimbal
        # tracking is unaffected (handled by tracker.py).
        if not self.is_body_confirmed():
            self._mav.send_zero_velocity()
            return 0.0

        # --- Compute following velocity ---
        vN, vE = self._compute_follow_velocity(dt)
        if vN is None:
            self._mav.send_zero_velocity()
            return 0.0

        # --- Drone yaw correction for gimbal pan ---
        pan_correction_rads = self._compute_yaw_correction(gimbal_pan_deg, dt)

        # --- Target NED position with standoff ---
        if not self._ekf.is_valid or not self._origin_set:
            self._mav.send_zero_velocity()
            return 0.0

        pN, pE             = self._ekf.get_position_ned()        # 2-tuple (N, E)
        drone_pN, drone_pE, _ = self._mav.get_position_ned()

        # S1.4 — Hard separation guard with active retreat + hysteresis.
        sep = math.hypot(pN - drone_pN, pE - drone_pE)
        self._last_person_sep_m = sep
        if self._should_retreat(sep):
            self._send_retreat_velocity(pN, pE, drone_pN, drone_pE, sep)
            return 0.0

        # S1.4 — Vertical separation guard.  Uses drone AGL altitude as the
        # vertical clearance above the subject (assumes flat ground at home
        # altitude; use the higher of configured follow altitude and minimum
        # clearance to preserve margin on uneven ground.
        alt_agl = self._mav.get_altitude_agl()
        self._last_vertical_clearance_m = alt_agl
        min_vertical_clearance = max(cfg.MIN_VERTICAL_SEP_M, cfg.FOLLOW_ALTITUDE_M)
        if alt_agl < min_vertical_clearance:
            self._mav.send_velocity_ned(0.0, 0.0, -cfg.RETREAT_SPEED_MS)  # vD<0 = climb
            return 0.0

        # S2.6 — Standoff bearing with hysteresis + slew-rate limit.
        vN_p, vE_p = self._ekf.get_velocity_ned()
        bearing = self._update_bearing(
            pN=pN, pE=pE,
            drone_pN=drone_pN, drone_pE=drone_pE, sep=sep,
            vN_p=vN_p, vE_p=vE_p, now=now, dt=dt,
        )

        # Desired position is FOLLOW_STANDOFF_M behind person along bearing
        target_pN = pN - math.cos(bearing) * cfg.FOLLOW_STANDOFF_M
        target_pE = pE - math.sin(bearing) * cfg.FOLLOW_STANDOFF_M
        target_pD = -(cfg.FOLLOW_ALTITUDE_M)   # NED down; altitude from config only

        # Altitude safety clamp
        safe_alt  = self._safety.check_altitude(cfg.FOLLOW_ALTITUDE_M)
        target_pD = -safe_alt

        if not self._target_within_geofence(target_pN, target_pE, safe_alt):
            self._mav.send_zero_velocity()
            return 0.0

        # S2.2 — HOME keep-out. Refuse targets that would put the airframe
        # on top of the operator standing at HOME.
        if not self._safety.check_home_keepout(target_pN, target_pE):
            self._mav.send_zero_velocity()
            return 0.0

        self._mav.send_position_velocity_ned(
            target_pN, target_pE, target_pD,
            vN, vE, 0.0,
        )
        return pan_correction_rads

    # ------------------------------------------------------------------
    #  Command target safety checks
    # ------------------------------------------------------------------

    def _update_bearing(
        self,
        pN: float, pE: float,
        drone_pN: float, drone_pE: float, sep: float,
        vN_p: float, vE_p: float,
        now: float, dt: float,
    ) -> float:
        """Compute the slew-rate-limited standoff bearing (S2.6).

        Uses person velocity direction once velocity has been continuously
        above STANDOFF_VEL_THRESHOLD_MS for at least BEARING_LATCH_S;
        otherwise uses the drone→person line. Output is rate-limited by
        BEARING_SLEW_DEG_S so the target NED point can never teleport.
        """
        speed_p = math.hypot(vN_p, vE_p)
        if speed_p > cfg.STANDOFF_VEL_THRESHOLD_MS:
            if self._vel_above_t < 0.0:
                self._vel_above_t = now
            sustained = (now - self._vel_above_t) >= cfg.BEARING_LATCH_S
        else:
            self._vel_above_t = -1.0
            sustained = False

        if sustained:
            desired = math.atan2(vE_p, vN_p)
        elif sep > 0.1:
            desired = math.atan2(pE - drone_pE, pN - drone_pN)
        else:
            desired = self._bearing_rad if self._bearing_init else 0.0

        if not self._bearing_init:
            self._bearing_rad = desired
            self._bearing_init = True
            return desired

        # Shortest signed angular difference, clamped to slew limit.
        diff = (desired - self._bearing_rad + math.pi) % (2 * math.pi) - math.pi
        max_step = math.radians(cfg.BEARING_SLEW_DEG_S) * max(dt, 1e-3)
        step = max(-max_step, min(max_step, diff))
        self._bearing_rad += step
        # Normalise into (-π, π]
        if self._bearing_rad > math.pi:
            self._bearing_rad -= 2 * math.pi
        elif self._bearing_rad <= -math.pi:
            self._bearing_rad += 2 * math.pi
        return self._bearing_rad

    def _should_retreat(self, sep: float) -> bool:
        """Latch retreat with hysteresis on horizontal separation."""
        if sep < cfg.MIN_PERSON_DRONE_SEP_M:
            self._retreating = True
        elif sep > cfg.MIN_PERSON_DRONE_SEP_M + cfg.RETREAT_HYSTERESIS_M:
            self._retreating = False
        return self._retreating

    def _send_retreat_velocity(
        self,
        pN: float, pE: float,
        drone_pN: float, drone_pE: float,
        sep: float,
    ) -> None:
        """Command a constant-speed vector pointing away from the subject."""
        if sep < 0.01:
            # Degenerate case (drone exactly on subject) — climb instead.
            self._mav.send_velocity_ned(0.0, 0.0, -cfg.RETREAT_SPEED_MS)
            return
        away_n = (drone_pN - pN) / sep
        away_e = (drone_pE - pE) / sep
        self._mav.send_velocity_ned(
            away_n * cfg.RETREAT_SPEED_MS,
            away_e * cfg.RETREAT_SPEED_MS,
            0.0,
        )

    def get_protection_status(self) -> dict:
        """Return current person/drone protection status for telemetry."""
        return {
            "person_sep_m": (
                round(self._last_person_sep_m, 2)
                if self._last_person_sep_m is not None else None
            ),
            "vertical_clearance_m": (
                round(self._last_vertical_clearance_m, 2)
                if self._last_vertical_clearance_m is not None else None
            ),
            "retreating": bool(self._retreating),
            "min_person_sep_m": cfg.MIN_PERSON_DRONE_SEP_M,
            "min_vertical_sep_m": max(cfg.MIN_VERTICAL_SEP_M, cfg.FOLLOW_ALTITUDE_M),
        }

    def _target_within_geofence(
        self, target_pN: float, target_pE: float, alt_agl: float
    ) -> bool:
        """Return True if the commanded target position is inside geofence."""
        if not self._origin_set:
            return False
        try:
            target_lat, target_lon = self._ekf.ned_to_gps(
                target_pN, target_pE,
                self._origin_lat, self._origin_lon,
            )
        except Exception as exc:
            import logging as _log
            _log.warning("[Drone] Target geofence projection error: %s", exc)
            return False

        ok = self._safety.geofence_contains(target_lat, target_lon, alt_agl)
        if not ok:
            now = time.monotonic()
            if now - self._target_fence_warn_t >= cfg.MODE_WARN_INTERVAL_S:
                self._target_fence_warn_t = now
                print("[Drone] Commanded follow target outside geofence — holding")
        return ok

    # ------------------------------------------------------------------
    #  EKF update
    # ------------------------------------------------------------------

    def _update_ekf_from_detection(self, target_info: TargetDetection) -> None:
        """Project bounding box to GPS and update the EKF."""
        if not self._origin_set:
            return

        lat, lon, alt_agl = self._mav.get_gps()
        roll, pitch, yaw  = self._mav.get_attitude()

        # Use gimbal angles passed in from tracker via set_gimbal_angles().
        gimbal_pan_rad  = getattr(self, "_gimbal_pan_rad",  0.0)
        gimbal_tilt_rad = getattr(self, "_gimbal_tilt_rad", math.radians(cfg.GIMBAL_TILT_DEFAULT_DEG))

        try:
            x1, y1, x2, y2 = target_info.x1, target_info.y1, target_info.x2, target_info.y2
            # frame size is stored separately; use a reasonable default
            frame_w = getattr(self, "_frame_w", 1280)
            frame_h = getattr(self, "_frame_h", 720)

            result = self._geo.project(
                x1, y1, x2, y2, frame_w, frame_h,
                lat, lon, alt_agl,
                roll, pitch, yaw,
                gimbal_pan_rad, gimbal_tilt_rad,
            )
            if result is None:
                return

            person_lat, person_lon = result
            meas_n, meas_e = self._ekf.gps_to_ned(
                person_lat, person_lon,
                self._origin_lat, self._origin_lon,
            )
            self._ekf.update(meas_n, meas_e)
        except Exception as e:
            import logging as _log
            _log.warning("[EKF] update error: %s", e)

    def set_frame_size(self, w: int, h: int) -> None:
        """Called by tracker.py once the frame resolution is known."""
        self._frame_w = w
        self._frame_h = h

    def set_gimbal_angles(
        self, pan_rad: float, tilt_rad: float
    ) -> None:
        """Called by tracker.py each update so EKF projections use actual angles."""
        self._gimbal_pan_rad  = pan_rad
        self._gimbal_tilt_rad = tilt_rad

    # ------------------------------------------------------------------
    #  Failsafe hierarchy
    # ------------------------------------------------------------------

    def _handle_tracking_loss(
        self, dt_lost: float, now: float
    ) -> Optional[float]:
        """Implement the tracking-loss failsafe ladder.

        Returns:
            A pan_correction float if failsafe took control (caller should
            return this value immediately).
            None if failsafe did not activate (normal operation continues).
        """
        if dt_lost < cfg.TRACKING_LOSS_HOVER_S:
            # Phase 0: EKF prediction — let _compute_follow_velocity handle it
            return None

        if dt_lost < cfg.TRACKING_LOSS_LOITER_S:
            # Phase 1: Person temporarily lost — decelerate to hover
            self._mav.send_zero_velocity()
            return 0.0

        if not self._loiter_issued:
            # Phase 2: Issue LOITER and alert operator
            self._mav.send_loiter()
            self._loiter_issued = True
            self._loiter_t      = now
            print(
                f"[Drone] Tracking lost {dt_lost:.1f}s — LOITER issued. "
                f"Drone holding position."
            )
        elif (now - self._loiter_t >= cfg.LOITER_CONFIRM_TIMEOUT_S
              and self._mav.get_mode() not in ("LOITER", "BRAKE")):
            self._mav.send_loiter()
            self._loiter_t = now   # one retry; timer resets so it won't fire again
            print("[Drone] LOITER retry (mode not confirmed)")

        if dt_lost >= cfg.TRACKING_LOSS_ALERT_S and not self._alert_issued:
            self._alert_issued = True
            print(
                f"[Drone] ⚠ ALERT: Person not detected for {dt_lost:.0f}s. "
                f"Drone in LOITER. Operator action required."
            )

        return 0.0   # failsafe active

    # ------------------------------------------------------------------
    #  Following velocity computation
    # ------------------------------------------------------------------

    def _compute_follow_velocity(
        self, dt: float
    ) -> tuple[Optional[float], Optional[float]]:
        """Compute drone body NED velocity to follow the EKF person estimate.

        Pipeline:
          EKF position error → proportional + feedforward
              → EMA filter
              → jerk limiter

        Returns:
            (vN, vE) in m/s, or (None, None) if EKF is not valid.
        """
        if not self._ekf.is_valid or not self._origin_set:
            return None, None

        pN_person, pE_person = self._ekf.get_position_ned()
        vN_person, vE_person = self._ekf.get_velocity_ned()
        pN_drone,  pE_drone, _ = self._mav.get_position_ned()

        # Proportional error toward person position
        err_n = pN_person - pN_drone
        err_e = pE_person - pE_drone
        dist  = math.sqrt(err_n**2 + err_e**2)

        # Dead-band: prevent micro-oscillations when close enough.
        if dist < cfg.DRONE_FOLLOW_DEADBAND_M:
            return self._apply_smoother(0.0, 0.0, dt)

        # Scale velocity proportionally, capped at MAX_TRACKING_SPEED_MS
        raw_vn = cfg.DRONE_KP * err_n + vN_person   # proportional + feedforward
        raw_ve = cfg.DRONE_KP * err_e + vE_person

        # Apply EMA + jerk limiter
        return self._apply_smoother(raw_vn, raw_ve, dt)

    def _apply_smoother(
        self, raw_vn: float, raw_ve: float, dt: float
    ) -> tuple[float, float]:
        """EMA filter followed by jerk limiter.

        Args:
            raw_vn, raw_ve: Desired velocity (m/s).
            dt:             Time step (s).

        Returns:
            Smoothed (vN, vE).
        """
        alpha = cfg.VEL_EMA_ALPHA

        # EMA
        self._ema_vn = alpha * raw_vn + (1 - alpha) * self._ema_vn
        self._ema_ve = alpha * raw_ve + (1 - alpha) * self._ema_ve

        # Jerk limit
        max_jerk = cfg.MAX_JERK_MS3
        if dt > 0:
            desired_an = (self._ema_vn - self._prev_vn) / dt
            desired_ae = (self._ema_ve - self._prev_ve) / dt
            jerk_n = (desired_an - self._prev_an) / dt
            jerk_e = (desired_ae - self._prev_ae) / dt
            jerk_n = max(-max_jerk, min(max_jerk, jerk_n))
            jerk_e = max(-max_jerk, min(max_jerk, jerk_e))
            an_lim = self._prev_an + jerk_n * dt
            ae_lim = self._prev_ae + jerk_e * dt

            # S2.5 — hard acceleration cap. Magnitude-preserving clamp so the
            # commanded acceleration vector keeps the same direction.
            an_lim, ae_lim = self._clamp_accel(an_lim, ae_lim)

            vn_out = self._prev_vn + an_lim * dt
            ve_out = self._prev_ve + ae_lim * dt
            self._prev_vn = vn_out
            self._prev_ve = ve_out
            self._prev_an = an_lim
            self._prev_ae = ae_lim
        else:
            vn_out = self._ema_vn
            ve_out = self._ema_ve

        return vn_out, ve_out

    @staticmethod
    def _clamp_accel(an: float, ae: float) -> tuple[float, float]:
        """Magnitude-preserving acceleration clamp at cfg.MAX_ACCEL_MS2."""
        mag = math.hypot(an, ae)
        if mag <= cfg.MAX_ACCEL_MS2 or mag == 0.0:
            return an, ae
        scale = cfg.MAX_ACCEL_MS2 / mag
        return an * scale, ae * scale

    # ------------------------------------------------------------------
    #  Drone yaw / gimbal pan correction
    # ------------------------------------------------------------------

    def _compute_yaw_correction(
        self, gimbal_pan_deg: float, dt: float
    ) -> float:
        """Compute drone yaw rate to recenter gimbal pan.

        When the gimbal pan exceeds GIMBAL_PAN_SOFT_DEG, the drone slowly
        rotates toward the person so the gimbal returns toward centre.

        The gimbal must receive a counter-rotation command so the person
        does not jump in frame.  The returned value should be subtracted
        from the gimbal pan PID output by tracker.py.

        Args:
            gimbal_pan_deg: Actual gimbal pan angle (degrees).
            dt:             Time step (s).

        Returns:
            Gimbal pan correction rate (rad/s). Negative of drone yaw rate.
        """
        pan_abs = abs(gimbal_pan_deg)
        if pan_abs <= cfg.GIMBAL_PAN_SOFT_DEG:
            return 0.0

        # Proportional rate toward recenter
        excess = pan_abs - cfg.GIMBAL_PAN_SOFT_DEG
        sign   = 1.0 if gimbal_pan_deg > 0 else -1.0

        # At soft limit: gentle rotation. At hard limit: fast rotation.
        hard_excess = cfg.GIMBAL_PAN_HARD_DEG - cfg.GIMBAL_PAN_SOFT_DEG
        factor = min(1.0, excess / max(hard_excess, 1.0))
        max_yaw_rate = math.radians(cfg.DRONE_MAX_YAW_RATE_DEG)
        yaw_rate = sign * factor * max_yaw_rate * cfg.DRONE_KP_YAW

        # Gimbal compensation: negate so person stays centred
        gimbal_correction = -yaw_rate
        return gimbal_correction

    # ------------------------------------------------------------------
    #  State reset
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Reset controller state on re-acquisition, seeding EMA from live telemetry.

        Seeds _ema_vn/ve and _prev_vn/ve from current drone velocity so the
        jerk limiter starts from the actual motion state rather than zero,
        preventing a velocity spike on the first _compute_follow_velocity() call.
        """
        vn, ve, _ = self._mav.get_velocity_ned()
        self._ema_vn  = vn
        self._ema_ve  = ve
        self._prev_vn = vn
        self._prev_ve = ve
        self._prev_an = 0.0
        self._prev_ae = 0.0
        self._loiter_issued = False
        self._alert_issued  = False
        self._confirm_count = 0
        self._retreating    = False
        self._last_person_sep_m = None
        self._last_vertical_clearance_m = None
        self._vel_above_t   = -1.0
        self._bearing_init  = False
        self._session_start_t   = -1.0
        self._session_rtl_issued = False
        self._rc_loss_loiter_issued = False
        self._ekf.reset()
