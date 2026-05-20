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
        self._confirm_count:  int = 0
        self._confirm_misses: int = 0   # consecutive missed frames; resets count only after threshold

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
        self._fps_times: deque = deque(maxlen=self._s.fps_window_size)
        self._fps_warned: bool = False

        # Low-battery warning (one print per session; reset on unfollow)
        self._batt_low_warned: bool = False

        # RTL retry state
        self._rtl_attempts:    int   = 0
        self._rtl_last_t:      float = 0.0
        self._rtl_confirmed:   bool  = False

        # Pre-check warning rate-limiters
        self._mode_warn_t:       float = 0.0
        self._sensor_warn_issued: bool = False
        self._target_fence_warn_t: float = 0.0
        self._home_req_t:        float = 0.0   # rate-limit HOME re-requests

        # Initialised when set_gimbal_angles() is first called by tracker
        self._gimbal_pan_rad:  float = 0.0
        self._gimbal_tilt_rad: float = math.radians(self._s.gimbal_tilt_default_deg)

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
                self._confirm_misses  = 0
                self._last_detection  = now
                self._loiter_issued   = False
                self._alert_issued    = False
                if self._confirm_count < self._s.body_move_confirm_frames:
                    self._confirm_count += 1
            else:
                # Only reset the confirmation counter after LOST_CONFIRM_FRAMES
                # consecutive misses.  A single dropped frame (very common with
                # FFmpeg grabber or ByteTrack re-ID flicker) was previously
                # resetting the counter to 0 every other frame, making it
                # impossible to reach the 5-frame threshold.
                self._confirm_misses += 1
                if self._confirm_misses >= self._s.lost_confirm_frames:
                    self._confirm_count = 0

    def is_body_confirmed(self) -> bool:
        """True once BODY_MOVE_CONFIRM_FRAMES consecutive detections seen."""
        with self._lock:
            return self._confirm_count >= self._s.body_move_confirm_frames

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
            # S3.3 — session timer persists across brief unfollow/follow cycles so
            # the operator can't bypass the 10-minute cap by toggling.  Only reset
            # when the FCU has been actually disarmed (motors stopped, on the ground).
            if not self._mav.is_armed():
                self._session_start_t    = -1.0
                self._session_rtl_issued = False
            self._batt_low_warned = False   # reset so next follow session warns fresh
            return 0.0

        now  = time.monotonic()
        dt   = now - self._last_update if self._last_update > 0 else 0.1
        self._last_update = now

        # S3.3 — session-time RTL. Caps the autonomous-tracking session at
        # MAX_FLIGHT_TIME_S so a forgotten run can never exceed safe battery.
        if self._session_start_t < 0:
            self._session_start_t = now
        if (now - self._session_start_t) >= self._s.max_flight_time_s:
            if not self._session_rtl_issued:
                print(f"[Drone] Session time {self._s.max_flight_time_s:.0f}s reached "
                      "— issuing RTL")
                get_flight_log().event(
                    "session_rtl",
                    elapsed_s=now - self._session_start_t,
                    limit_s=self._s.max_flight_time_s,
                )
                self._mav.send_rtl()
                self._session_rtl_issued = True
            return 0.0

        # Advance EKF state estimate every tick so velocity feedforward stays fresh
        # between detection updates (predict-only steps incur no measurement cost).
        self._ekf.predict(dt)

        # Ground-test mode: bypass all flight-critical gates and run the full
        # velocity pipeline. MAVLink TX is already suppressed by the client;
        # commands are logged as [DRY-RUN] so operators can verify the pipeline
        # on a bench without GPS, arming, or RC.
        if self._mav.is_ground_test():
            return self._update_ground_test_pipeline(
                gimbal_pan_deg, gimbal_tilt_deg, target_info, now, dt
            )

        # --- Safety pre-checks ---
        if not self._mav.is_connected():
            return 0.0

        if not self._safety.watchdog_heartbeat(self._mav.get_last_heartbeat_time()):
            print("[Drone] MAVLink heartbeat lost — stopping commands")
            return 0.0

        # --- A1: GUIDED mode gate ---
        if self._mav.get_mode() != "GUIDED":
            if now - self._mode_warn_t >= self._s.mode_warn_interval_s:
                self._mode_warn_t = now
                print(f"[Drone] Not in GUIDED ({self._mav.get_mode()}) — commands suppressed")
            return 0.0   # no send; GUID_TIMEOUT irrelevant outside GUIDED

        # --- S3.6: RC-override latch ---
        if self._mav.is_rc_override_active():
            if now - self._mode_warn_t >= self._s.mode_warn_interval_s:
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
            if now - self._mode_warn_t >= self._s.mode_warn_interval_s:
                self._mode_warn_t = now
                print("[Drone] FCU not armed — body commands suppressed "
                      "(arm the FCU before 'follow')")
            return 0.0

        # --- A3/A4: Establish NED origin — HOME preferred, GPS fallback, (0,0) last resort ---
        # The EKF needs a fixed reference frame.  We never block on this:
        # HOME > live GPS > (0,0) dummy.  Using (0,0) still lets the
        # pipeline run; EKF projections will be garbage without real GPS but
        # the failsafe / FPS / confirm logic all still exercises correctly.
        if not self._origin_set:
            if self._mav.is_home_set():
                home_lat, home_lon, _ = self._mav.get_home_position()
                self._origin_lat = home_lat
                self._origin_lon = home_lon
                self._safety.set_home(home_lat, home_lon)
            else:
                lat_fb, lon_fb, _ = self._mav.get_gps()
                if lat_fb != 0.0 or lon_fb != 0.0:
                    self._origin_lat = lat_fb
                    self._origin_lon = lon_fb
                    self._safety.set_home(lat_fb, lon_fb)
                    print(f"[Drone] HOME not received — NED origin from GPS "
                          f"({lat_fb:.6f}, {lon_fb:.6f})")
                else:
                    print("[Drone] HOME not received, GPS unavailable — "
                          "using (0,0) dummy origin")
            self._origin_set = True

        # --- C3: Sensor health gate ---
        if not self._mav.is_sensors_healthy():
            if not self._sensor_warn_issued:
                print("[Drone] Critical sensors not healthy — suppressing commands")
                self._sensor_warn_issued = True
            self._mav.send_zero_velocity()
            return 0.0
        self._sensor_warn_issued = False

        # --- E1: Battery low → warn operator once per session ---
        batt_v = self._mav.get_battery_voltage()
        if self._safety.is_battery_low(batt_v) and not self._batt_low_warned:
            self._batt_low_warned = True
            warn_v = self._s.cell_warn_mv / 1000.0
            from utils import terminal as _t
            _t.event(f"[Drone] ⚠ BATTERY LOW — land soon "
                     f"({batt_v:.2f}V, warn threshold {warn_v:.2f}V/cell)")

        # --- E2: Battery critical → RTL with fire-and-verify retry ---
        if self._safety.is_battery_critical(batt_v):
            if not self._rtl_confirmed:
                if self._mav.get_mode() == "RTL":
                    self._rtl_confirmed = True
                    print("[Drone] RTL confirmed by flight controller")
                elif (not self._rtl_issued
                      or now - self._rtl_last_t >= self._s.rtl_retry_interval_s):
                    if self._rtl_attempts < self._s.rtl_max_attempts:
                        self._mav.send_rtl()
                        self._rtl_issued    = True
                        self._rtl_last_t    = now
                        self._rtl_attempts += 1
                        print(f"[Drone] BATTERY CRITICAL — RTL attempt "
                              f"{self._rtl_attempts}/{self._s.rtl_max_attempts}")
                    else:
                        print(f"[Drone] ⚠ RTL FAILED after {self._s.rtl_max_attempts} "
                              "attempts — operator must intervene!")
            return 0.0

        # --- GPS check ---
        if not self._mav.is_gps_ok():
            print("[Drone] GPS lost — zero velocity")
            self._mav.send_zero_velocity()
            return 0.0

        # --- D1: Geofence check — return toward home instead of just stopping ---
        lat, lon, alt = self._mav.get_gps()
        if alt < self._s.min_alt_m:
            self._mav.send_velocity_ned(0.0, 0.0, -self._s.retreat_speed_ms)  # vD<0 = climb
            print(f"[Drone] Geofence altitude low ({alt:.1f}m) — climbing")
            return 0.0
        if alt > self._s.max_alt_m:
            self._mav.send_velocity_ned(0.0, 0.0, self._s.retreat_speed_ms)   # vD>0 = descend
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
                    speed = min(self._s.geofence_return_max_ms,
                                dist * self._s.geofence_return_kp)
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
        fps = self.effective_fps()
        if fps > 0.0 and fps < self._s.min_tracking_fps:
            if not self._fps_warned:
                print(f"[Drone] Detection FPS {fps:.1f} < "
                      f"{self._s.min_tracking_fps:.1f} — body holding")
                get_flight_log().event(
                    "fps_floor", fps=fps, threshold=self._s.min_tracking_fps,
                    state="hold",
                )
                self._fps_warned = True
            self._mav.send_zero_velocity()
            return 0.0
        elif fps >= self._s.min_tracking_fps and self._fps_warned:
            print(f"[Drone] Detection FPS recovered ({fps:.1f}) — body resuming")
            get_flight_log().event(
                "fps_floor", fps=fps, threshold=self._s.min_tracking_fps,
                state="resume",
            )
            self._fps_warned = False

        # --- S1.5: Body-movement confirmation gate ---
        if not self.is_body_confirmed():
            if now - getattr(self, '_confirm_warn_t', 0.0) >= 2.0:
                self._confirm_warn_t = now
                print(f"[Drone] Body confirm: {self._confirm_count}/"
                      f"{self._s.body_move_confirm_frames} consecutive detections "
                      f"— holding until confirmed")
            self._mav.send_zero_velocity()
            return 0.0

        # --- EKF validity ---
        if not self._ekf.is_valid or not self._origin_set:
            if now - getattr(self, '_ekf_warn_t', 0.0) >= 2.0:
                self._ekf_warn_t = now
                print("[Drone] EKF not yet initialised — waiting for first "
                      "valid camera projection (needs alt > 0.5m and gimbal down)")
            self._mav.send_zero_velocity()
            return 0.0

        # ----------------------------------------------------------------
        # GPS-based follow: eliminates NED-frame mismatch between our
        # EKF origin and ArduPilot's LOCAL_NED EKF origin. All distances
        # and bearings are computed from GPS coordinates; the final command
        # is velocity-only (no position target) so the wrong-frame position
        # component can never send the drone to the wrong absolute location.
        # ----------------------------------------------------------------

        pN_p, pE_p       = self._ekf.get_position_ned()
        vN_p, vE_p       = self._ekf.get_velocity_ned()

        # Person GPS from our EKF (referenced to our origin — GPS or HOME)
        person_lat, person_lon = self._ekf.ned_to_gps(
            pN_p, pE_p, self._origin_lat, self._origin_lon
        )

        # Drone GPS from FCU telemetry — absolute, frame-independent
        drone_lat, drone_lon, _ = self._mav.get_gps()
        if drone_lat == 0.0 and drone_lon == 0.0:
            self._mav.send_zero_velocity()
            return 0.0

        # Drone→person vector in NED metres (flat-earth, GPS-based)
        _R        = 6_371_000.0
        _cos_lat  = math.cos(math.radians(drone_lat))
        dn = (person_lat - drone_lat) * _R * (math.pi / 180.0)
        de = (person_lon - drone_lon) * _R * (math.pi / 180.0) * _cos_lat
        sep = math.hypot(dn, de)
        self._last_person_sep_m = sep

        # S1.4 — Separation guard: retreat if inside MIN_PERSON_DRONE_SEP_M
        if self._should_retreat(sep):
            if now - getattr(self, '_retreat_warn_t', 0.0) >= 2.0:
                self._retreat_warn_t = now
                print(f"[Drone] Person {sep:.1f}m from drone "
                      f"(min {self._s.min_person_drone_sep_m:.0f}m) — retreating")
            if sep > 0.01:
                self._mav.send_velocity_ned(
                    -dn / sep * self._s.retreat_speed_ms,
                    -de / sep * self._s.retreat_speed_ms,
                    0.0,
                )
            return 0.0

        # S1.4 — Vertical clearance guard (0.5 m hysteresis prevents GPS-noise oscillation)
        alt_agl = self._mav.get_altitude_agl()
        self._last_vertical_clearance_m = alt_agl
        min_vert = max(self._s.min_vertical_sep_m, self._s.follow_altitude_m)
        # Use a lower trigger threshold so GPS noise (±0.5 m) at target altitude
        # doesn't flip between climb and follow on every frame.
        if alt_agl < (min_vert - 0.5):
            if now - getattr(self, '_vclear_warn_t', 0.0) >= 3.0:
                self._vclear_warn_t = now
                print(f"[Drone] Altitude {alt_agl:.1f}m below "
                      f"{min_vert - 0.5:.1f}m follow threshold — climbing "
                      f"(target {min_vert:.0f}m)")
            self._mav.send_velocity_ned(0.0, 0.0, -self._s.retreat_speed_ms)
            return 0.0

        # S2.6 — Bearing with slew-rate limit.
        # Pass GPS-based dn/de as the "NED-from-drone-to-person" vector;
        # _update_bearing treats its pN/pE - drone_pN/pE as that vector.
        bearing = self._update_bearing(
            pN=dn, pE=de,
            drone_pN=0.0, drone_pE=0.0, sep=max(sep, 0.1),
            vN_p=vN_p, vE_p=vE_p, now=now, dt=dt,
        )

        # Standoff target: FOLLOW_STANDOFF_M behind person along bearing.
        # target_dn/de = how far the drone still needs to travel.
        target_dn = dn - math.cos(bearing) * self._s.follow_standoff_m
        target_de = de - math.sin(bearing) * self._s.follow_standoff_m
        target_dist = math.hypot(target_dn, target_de)

        # Proportional velocity toward standoff target + person feedforward
        if target_dist < self._s.drone_follow_deadband_m:
            vN, vE = self._apply_smoother(0.0, 0.0, dt)
        else:
            raw_vn = self._s.drone_kp * target_dn + vN_p
            raw_ve = self._s.drone_kp * target_de + vE_p
            vN, vE = self._apply_smoother(raw_vn, raw_ve, dt)

        # S2.2 — HOME keep-out: only meaningful when actual HOME_POSITION
        # was received (not GPS fallback where origin = drone hover spot).
        if self._mav.is_home_set():
            tkN = pN_p - math.cos(bearing) * self._s.follow_standoff_m
            tkE = pE_p - math.sin(bearing) * self._s.follow_standoff_m
            if not self._safety.check_home_keepout(tkN, tkE):
                self._mav.send_zero_velocity()
                return 0.0

        # --- Drone yaw correction for gimbal pan ---
        pan_correction_rads = self._compute_yaw_correction(gimbal_pan_deg, dt)

        # Confirmation log every 3 s so operator knows commands are flowing
        if now - getattr(self, '_follow_log_t', 0.0) >= 3.0:
            self._follow_log_t = now
            print(f"[Drone] Following ✓  vel=({vN:.2f},{vE:.2f}) m/s  "
                  f"person={sep:.1f}m  to_target={target_dist:.1f}m  "
                  f"alt={alt_agl:.1f}m")

        # Send VELOCITY ONLY — position target is frame-dependent and wrong;
        # velocity computed from GPS is always in the correct direction.
        self._mav.send_velocity_ned(vN, vE, 0.0)
        return pan_correction_rads

    # ------------------------------------------------------------------
    #  Ground-test pipeline  (--ground-test bench verification)
    # ------------------------------------------------------------------

    def _update_ground_test_pipeline(
        self,
        gimbal_pan_deg:  float,
        gimbal_tilt_deg: float,
        target_info:     Optional[TargetDetection],
        now:             float,
        dt:              float,
    ) -> float:
        """Run the full following pipeline without any flight-critical gates.

        Called in place of the main update() body when --ground-test is active.
        All MAVLink sends are intercepted by the client and logged as [DRY-RUN]
        instead of transmitted.  Safety guards that make no sense on a bench
        (ARM, HOME, GPS quality, RC link, geofence, altitude, separation) are
        skipped so the EKF → velocity → command path can be fully exercised.

        Prints a one-time banner so the operator knows the ground-test pipeline
        is running, then periodically reports computed velocities.
        """
        if not getattr(self, "_gt_announced", False):
            print("[Drone] Ground-test pipeline active — safety gates bypassed, "
                  "TX suppressed. Tracking pipeline will log [DRY-RUN] commands.")
            self._gt_announced = True

        # Use a dummy NED origin when home position is unavailable on the bench.
        if not self._origin_set:
            if self._mav.is_home_set():
                home_lat, home_lon, _ = self._mav.get_home_position()
                self._origin_lat = home_lat
                self._origin_lon = home_lon
            else:
                self._origin_lat = 0.0
                self._origin_lon = 0.0
            self._origin_set = True

        # Update EKF with latest detection (may use near-zero GPS in bench mode).
        if target_info is not None and self._origin_set:
            self._update_ekf_from_detection(target_info)

        # Tracking-loss failsafe still applies — identical logic to real flight.
        with self._lock:
            dt_lost = now - self._last_detection
        pan_correction = self._handle_tracking_loss(dt_lost, now)
        if pan_correction is not None:
            return pan_correction

        # FPS floor: same as real flight.
        fps = self.effective_fps()
        if fps > 0.0 and fps < self._s.min_tracking_fps:
            if not self._fps_warned:
                print(f"[Drone] Detection FPS {fps:.1f} < "
                      f"{self._s.min_tracking_fps:.1f} — body holding")
                self._fps_warned = True
            self._mav.send_zero_velocity()
            return 0.0
        elif fps >= self._s.min_tracking_fps and self._fps_warned:
            print(f"[Drone] Detection FPS recovered ({fps:.1f}) — body resuming")
            self._fps_warned = False

        # Body-movement confirmation: same as real flight.
        if not self.is_body_confirmed():
            self._mav.send_zero_velocity()
            return 0.0

        # Compute following velocity from EKF state.
        vN, vE = self._compute_follow_velocity(dt)
        if vN is None:
            self._mav.send_zero_velocity()
            return 0.0

        pan_correction_rads = self._compute_yaw_correction(gimbal_pan_deg, dt)

        if not self._ekf.is_valid or not self._origin_set:
            self._mav.send_zero_velocity()
            return 0.0

        pN, pE             = self._ekf.get_position_ned()
        drone_pN, drone_pE, _ = self._mav.get_position_ned()

        # Skip separation / vertical-clearance guards — drone is on the bench.

        sep = math.hypot(pN - drone_pN, pE - drone_pE)
        vN_p, vE_p = self._ekf.get_velocity_ned()
        bearing = self._update_bearing(
            pN=pN, pE=pE,
            drone_pN=drone_pN, drone_pE=drone_pE, sep=max(sep, 0.1),
            vN_p=vN_p, vE_p=vE_p, now=now, dt=dt,
        )

        target_pN = pN - math.cos(bearing) * self._s.follow_standoff_m
        target_pE = pE - math.sin(bearing) * self._s.follow_standoff_m
        target_pD = -(self._s.follow_altitude_m)

        # Skip geofence / home-keepout checks — not meaningful on a bench.

        # TX is suppressed by the client in ground-test; this call logs [DRY-RUN].
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
        if speed_p > self._s.standoff_vel_threshold_ms:
            if self._vel_above_t < 0.0:
                self._vel_above_t = now
            sustained = (now - self._vel_above_t) >= self._s.bearing_latch_s
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
        max_step = math.radians(self._s.bearing_slew_deg_s) * max(dt, 1e-3)
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
        if sep < self._s.min_person_drone_sep_m:
            self._retreating = True
        elif sep > self._s.min_person_drone_sep_m + self._s.retreat_hysteresis_m:
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
            self._mav.send_velocity_ned(0.0, 0.0, -self._s.retreat_speed_ms)
            return
        away_n = (drone_pN - pN) / sep
        away_e = (drone_pE - pE) / sep
        self._mav.send_velocity_ned(
            away_n * self._s.retreat_speed_ms,
            away_e * self._s.retreat_speed_ms,
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
            "min_person_sep_m": self._s.min_person_drone_sep_m,
            "min_vertical_sep_m": max(self._s.min_vertical_sep_m, self._s.follow_altitude_m),
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
            if now - self._target_fence_warn_t >= self._s.mode_warn_interval_s:
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
        gimbal_tilt_rad = getattr(self, "_gimbal_tilt_rad", math.radians(self._s.gimbal_tilt_default_deg))

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
        if dt_lost < self._s.tracking_loss_hover_s:
            # Phase 0: EKF prediction — let _compute_follow_velocity handle it
            return None

        if dt_lost < self._s.tracking_loss_loiter_s:
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
        elif (now - self._loiter_t >= self._s.loiter_confirm_timeout_s
              and self._mav.get_mode() not in ("LOITER", "BRAKE")):
            self._mav.send_loiter()
            self._loiter_t = now   # one retry; timer resets so it won't fire again
            print("[Drone] LOITER retry (mode not confirmed)")

        if dt_lost >= self._s.tracking_loss_alert_s and not self._alert_issued:
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
        if dist < self._s.drone_follow_deadband_m:
            return self._apply_smoother(0.0, 0.0, dt)

        # Scale velocity proportionally, capped at MAX_TRACKING_SPEED_MS
        raw_vn = self._s.drone_kp * err_n + vN_person   # proportional + feedforward
        raw_ve = self._s.drone_kp * err_e + vE_person

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
        alpha = self._s.vel_ema_alpha

        # EMA
        self._ema_vn = alpha * raw_vn + (1 - alpha) * self._ema_vn
        self._ema_ve = alpha * raw_ve + (1 - alpha) * self._ema_ve

        # Jerk limit
        max_jerk = self._s.max_jerk_ms3
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

    def _clamp_accel(self, an: float, ae: float) -> tuple[float, float]:
        """Magnitude-preserving acceleration clamp at MAX_ACCEL_MS2."""
        mag = math.hypot(an, ae)
        max_a = self._s.max_accel_ms2
        if mag <= max_a or mag == 0.0:
            return an, ae
        scale = max_a / mag
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
        if pan_abs <= self._s.gimbal_pan_soft_deg:
            return 0.0

        # Proportional rate toward recenter
        excess = pan_abs - self._s.gimbal_pan_soft_deg
        sign   = 1.0 if gimbal_pan_deg > 0 else -1.0

        # At soft limit: gentle rotation. At hard limit: fast rotation.
        hard_excess = self._s.gimbal_pan_hard_deg - self._s.gimbal_pan_soft_deg
        factor = min(1.0, excess / max(hard_excess, 1.0))
        max_yaw_rate = math.radians(self._s.drone_max_yaw_rate_deg)
        yaw_rate = sign * factor * max_yaw_rate * self._s.drone_kp_yaw

        # Gimbal compensation: negate so person stays centred
        gimbal_correction = -yaw_rate
        return gimbal_correction

    # ------------------------------------------------------------------
    #  State reset
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Fully reset controller state, seeding EMA from live telemetry.

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
        self._loiter_issued   = False
        self._alert_issued    = False
        self._confirm_count   = 0
        self._confirm_misses  = 0
        self._retreating      = False
        self._last_person_sep_m = None
        self._last_vertical_clearance_m = None
        self._vel_above_t   = -1.0
        self._bearing_init  = False
        # Preserve session timer across reset() — it only resets when FCU disarms.
        # This prevents the operator bypassing the 10-min cap via unfollow+follow.
        self._rc_loss_loiter_issued = False
        self._ekf.reset()

    def on_target_reacquired(self) -> None:
        """Lightweight visual-reacquisition hook.

        Unlike reset(), this preserves EKF continuity, session timers, retreat
        state, and body-confirmation counters. It only seeds the velocity
        smoother from live telemetry so the next follow command does not jump
        after a brief detector dropout.
        """
        vn, ve, _ = self._mav.get_velocity_ned()
        self._ema_vn = vn
        self._ema_ve = ve
        self._prev_vn = vn
        self._prev_ve = ve
        self._prev_an = 0.0
        self._prev_ae = 0.0
        self._loiter_issued = False
        self._alert_issued = False
