"""
mavlink_client.py — Thread-safe MAVLink connection manager.

This is the ONLY module in the system that opens a MAVLink connection.
All other modules call methods here; they never touch pymavlink directly.

Responsibilities:
  - Open serial connection to Orange Cube+ (ArduPilot)
  - Request attitude, position, GPS, and battery message streams
  - Maintain a heartbeat watchdog timestamp
  - Auto-detect battery cell count from SYS_STATUS voltage on startup
  - Provide thread-safe getters for all drone telemetry
  - Send velocity and position-target commands to ArduPilot GUIDED mode
  - Send LOITER and RTL commands

ArduPilot parameters that MUST be set on the flight controller:
    GUID_TIMEOUT  = 5       (seconds before stopping if no command arrives)
    WPNAV_SPEED   = 500     (cm/s = 5 m/s max)
    WPNAV_ACCEL   = 150     (cm/s²)
    PSC_JERK_XY   = 3.0     (m/s³)
    FENCE_ENABLE  = 1
    FENCE_ALT_MIN = 10      (m AGL)

Usage:
    mav = MAVLinkClient()
    if mav.connect():
        mav.send_velocity_ned(0.5, 0.0, 0.0)
    mav.close()
"""

import collections
import math
import threading
import time
from typing import Optional

from config.settings import Settings, load_settings
from safety import SafetyMonitor
from utils.flight_log import safe_event

try:
    from pymavlink import mavutil
    _PYMAVLINK_AVAILABLE = True
except ImportError:
    _PYMAVLINK_AVAILABLE = False
    print("[MAVLink] WARNING: pymavlink not installed — drone control disabled")


# MAVLink type_mask constants for SET_POSITION_TARGET_LOCAL_NED
_MASK_POS_VEL   = 3520   # 0b0000_1101_1100_0000 — use position + velocity feedforward
_MASK_VEL_ONLY  = 3527   # 0b0000_1101_1100_0111 — use velocity only

# MAV_SYS_STATUS_SENSOR bitmask: gyro(1) + accel(2) + mag(4) + baro(8)
# GPS is checked independently via is_gps_ok() / GPS_RAW_INT.fix_type
_CRITICAL_SENSORS: int = 0x01 | 0x02 | 0x04 | 0x08   # = 0x0F

# MAVLink command IDs and mode flag — defined as plain integers so _set_mode()
# and send_takeoff() work in unit tests where pymavlink is not installed.
# Values match the ArduPilot / MAVLink common.xml spec; do not change.
_MAV_CMD_DO_SET_MODE:              int = 176
_MAV_CMD_NAV_TAKEOFF:              int = 22
_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: int = 1
_MAV_FRAME_LOCAL_NED:              int = 1    # MAV_FRAME_LOCAL_NED
_MAV_CMD_COMPONENT_ARM_DISARM:     int = 400  # arm/disarm command


class MAVLinkClient:
    """Thread-safe ArduPilot MAVLink connection manager.

    Args:
        safety:  SafetyMonitor instance (shared with DroneController).
        device:  Serial device path (e.g. '/dev/ttyACM0').
        baud:    Serial baud rate.
    """

    def __init__(
        self,
        safety: SafetyMonitor,
        device: str | None     = None,
        baud: int | None       = None,
        settings: Settings | None = None,
    ) -> None:
        self._s        = settings or load_settings()
        self._safety   = safety
        self._device   = device or self._s.mavlink_device
        self._baud     = baud or self._s.mavlink_baud
        self._mav      = None     # pymavlink MAVLink connection
        self._lock     = threading.Lock()
        self._running  = False
        self._rx_thread: Optional[threading.Thread] = None

        # COMMAND_ACK registry — keyed by command ID
        self._ack_lock:    threading.Lock                   = threading.Lock()
        self._ack_events:  dict[int, threading.Event]       = {}
        self._ack_results: dict[int, int]                   = {}
        # COMMAND_ACK only identifies the command type. Serialise command_long
        # calls so concurrent mode/takeoff/estop requests cannot consume each
        # other's ACK when they share the same command ID.
        self._command_lock: threading.Lock                  = threading.Lock()

        # Telemetry cache — updated by _rx_loop under _lock
        self._pos_n:        float = 0.0    # NED north offset from EKF origin (m)
        self._pos_e:        float = 0.0    # NED east  offset from EKF origin (m)
        self._pos_d:        float = 0.0    # NED down  (negative = up)
        self._vel_n:        float = 0.0    # m/s
        self._vel_e:        float = 0.0    # m/s
        self._vel_d:        float = 0.0    # m/s
        self._roll:         float = 0.0    # rad
        self._pitch:        float = 0.0    # rad
        self._yaw:          float = 0.0    # rad
        self._lat:          float = 0.0    # degrees
        self._lon:          float = 0.0    # degrees
        self._alt_msl:      float = 0.0    # m MSL
        self._alt_rel:      float = 0.0    # m AGL (relative to home)
        self._vbat_mv:      int   = 0      # mV (from SYS_STATUS)
        self._gps_fix:      int   = 0      # fix type (0–6)
        self._n_sats:       int   = 0
        self._gps_hdop:     float = 99.99  # GPS_RAW_INT eph / 100 (S2.1)
        self._ekf_pos_var:  float = 0.0    # EKF_STATUS_REPORT pos_horiz_variance (S2.1)
        self._ekf_status_seen: bool = False  # only enforce var when message received
        self._armed:        bool  = False
        self._landed_state: int | None = None
        self._mode:         str   = "UNKNOWN"
        self._hb_time:      float = 0.0    # monotonic timestamp of last heartbeat
        self._global_pos_t:  float = 0.0
        self._gps_raw_t:     float = 0.0
        self._ekf_status_t:  float = 0.0
        self._sys_status_t:  float = 0.0

        self._home_lat:     float = 0.0
        self._home_lon:     float = 0.0
        self._home_alt:     float = 0.0
        self._home_set:     bool  = False

        self._cell_count:   int   = self._s.default_cells
        self._cell_detected: bool = False
        self._cell_ambig_warned: bool = False  # rate-limits the "ambiguous" log to one line per session

        # C1: ArduPilot onboard fence breach
        self._fence_breached:    bool = False
        self._fence_breach_type: int  = 0

        # C3: Sensor health from SYS_STATUS
        self._sensors_health:  int = 0
        self._sensors_present: int = 0

        # S2.3: parameter cache populated by PARAM_VALUE messages
        self._params:        dict[str, float]              = {}
        self._param_events:  dict[str, threading.Event]    = {}
        self._param_lock:    threading.Lock                = threading.Lock()

        # S3.6: RC-override latch. Trips when the FCU exits GUIDED (RC pilot
        # took control, autopilot failsafe, etc.). Cleared only by an
        # explicit arm-tracker request after preflight passes again.
        self._rc_override_latched: bool = False

        # RC link health (preflight gate — transmitter must be ON before takeoff)
        self._rc_last_t:     float = 0.0   # monotonic ts of last RC_CHANNELS
        self._rc_chancount:  int   = 0
        self._rc_rssi:       int   = 0     # 0–255 (RC_CHANNELS.rssi)

        # S1.2: ground-test (dry-run) mode — when True, no TX is emitted.
        self._ground_test: bool = bool(getattr(self._s, "ground_test", False))
        self._ground_test_banner_t: float = 0.0
        if self._ground_test:
            print("[MAVLink] GROUND-TEST MODE — all TX suppressed; RX unchanged")

        # Mode-flapping detector. RC pilot input vs tracker SET_MODE GUIDED can
        # produce hundreds of mode transitions per minute (see logs 2026-05-16).
        # Track transitions in a rolling window and collapse them into a single
        # "flapping" summary so the log stays usable.
        self._mode_xitions:  collections.deque[float] = collections.deque(maxlen=64)
        self._mode_flapping: bool = False
        self._mode_flap_count: int = 0
        self._MODE_FLAP_WINDOW_S:    float = 10.0
        self._MODE_FLAP_TRIGGER_N:   int   = 6    # 6 transitions in 10s = flapping
        self._MODE_FLAP_CLEAR_N:     int   = 2    # ≤2 in the window = stable again

    # ------------------------------------------------------------------
    #  Connection lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Open the MAVLink serial connection and request message streams.

        Returns:
            True if connected and heartbeat received. False otherwise.
        """
        if not _PYMAVLINK_AVAILABLE:
            print("[MAVLink] Cannot connect — pymavlink not installed")
            return False

        print(f"[MAVLink] Connecting to {self._device} @ {self._baud}...")
        try:
            self._mav = mavutil.mavlink_connection(
                self._device, baud=self._baud, source_system=255
            )
            print("[MAVLink] Waiting for heartbeat...")
            hb = self._mav.wait_heartbeat(timeout=10)
            if hb is None:
                raise TimeoutError("heartbeat timeout")
            self._hb_time = time.monotonic()
            print(
                f"[MAVLink] Connected — "
                f"sysid={self._mav.target_system}  "
                f"compid={self._mav.target_component}"
            )
        except Exception as exc:
            print(f"[MAVLink] Connection failed: {exc}")
            try:
                if self._mav is not None:
                    self._mav.close()
            except Exception:
                pass
            self._mav = None
            self._running = False
            return False

        self._running = True
        self._request_streams()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name="MAVLinkRX"
        )
        self._rx_thread.start()

        # Give streams a moment to arrive then read home position
        time.sleep(0.5)
        self._read_home_position()
        return True

    def is_connected(self) -> bool:
        """Return True if the MAVLink link is open and the heartbeat is fresh."""
        if self._mav is None or not self._running:
            return False
        return self._safety.watchdog_heartbeat(self._hb_time)

    def close(self) -> None:
        """Gracefully shut down the receive thread and close the connection."""
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=2.0)
        if self._mav:
            try:
                self._mav.close()
            except Exception:
                pass
            self._mav = None
        print("[MAVLink] Connection closed")

    # ------------------------------------------------------------------
    #  Message stream requests
    # ------------------------------------------------------------------

    def _request_streams(self) -> None:
        """Ask ArduPilot to send the telemetry messages we need."""
        if self._mav is None:
            return

        def req(stream_id: int, rate_hz: int) -> None:
            self._mav.mav.request_data_stream_send(
                self._mav.target_system,
                self._mav.target_component,
                stream_id,
                rate_hz,
                1,   # start
            )

        # ATTITUDE + LOCAL_POSITION_NED at 20 Hz
        req(mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20)    # ATTITUDE
        req(mavutil.mavlink.MAV_DATA_STREAM_POSITION, 20)  # LOCAL_POSITION_NED, GPS

        # Battery / status at 2 Hz
        req(mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2)     # AHRS3, hardware status
        req(mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 2)

        print("[MAVLink] Telemetry streams requested")

    # ------------------------------------------------------------------
    #  Receive loop
    # ------------------------------------------------------------------

    def _rx_loop(self) -> None:
        """Background thread: parse incoming MAVLink messages."""
        while self._running and self._mav is not None:
            try:
                msg = self._mav.recv_match(blocking=True, timeout=0.5)
                if msg is None:
                    continue
                t = msg.get_type()

                with self._lock:
                    if t == "HEARTBEAT":
                        self._process_heartbeat(msg)

                    elif t == "ATTITUDE":
                        self._roll  = msg.roll
                        self._pitch = msg.pitch
                        self._yaw   = msg.yaw

                    elif t == "LOCAL_POSITION_NED":
                        self._pos_n = msg.x
                        self._pos_e = msg.y
                        self._pos_d = msg.z
                        self._vel_n = msg.vx
                        self._vel_e = msg.vy
                        self._vel_d = msg.vz

                    elif t == "EXTENDED_SYS_STATE":
                        self._landed_state = getattr(msg, "landed_state", None)

                    elif t == "GLOBAL_POSITION_INT":
                        self._global_pos_t = time.monotonic()
                        self._lat     = msg.lat  / 1e7
                        self._lon     = msg.lon  / 1e7
                        self._alt_msl = msg.alt  / 1000.0
                        self._alt_rel = msg.relative_alt / 1000.0

                    elif t == "GPS_RAW_INT":
                        self._gps_raw_t = time.monotonic()
                        self._gps_fix = msg.fix_type
                        self._n_sats  = msg.satellites_visible
                        # eph is the horizontal-position uncertainty in cm.
                        # MAVLink encodes "unknown" as 65535. Treat that as
                        # very bad HDOP so the gate fails closed.
                        eph = getattr(msg, "eph", 65535)
                        self._gps_hdop = 99.99 if eph >= 65535 else eph / 100.0

                    elif t == "EKF_STATUS_REPORT":
                        self._ekf_status_t = time.monotonic()
                        # Worst-case horizontal variance — combined N/E.
                        self._ekf_pos_var = float(
                            getattr(msg, "pos_horiz_variance", 0.0)
                        )
                        self._ekf_status_seen = True

                    elif t == "RC_CHANNELS":
                        # FCU forwards RC_CHANNELS whenever the receiver
                        # delivers a frame. Stop arrival = RC link lost.
                        self._rc_last_t    = time.monotonic()
                        self._rc_chancount = int(getattr(msg, "chancount", 0))
                        self._rc_rssi      = int(getattr(msg, "rssi", 0))

                    elif t == "SYS_STATUS":
                        self._sys_status_t     = time.monotonic()
                        self._vbat_mv          = msg.voltage_battery
                        self._sensors_health   = msg.onboard_control_sensors_health   # C3
                        self._sensors_present  = msg.onboard_control_sensors_present  # C3
                        if not self._cell_detected and self._vbat_mv > 0:
                            self._detect_cell_count()

                    elif t == "HOME_POSITION":
                        if not self._home_set:
                            self._home_lat = msg.latitude  / 1e7
                            self._home_lon = msg.longitude / 1e7
                            self._home_alt = msg.altitude  / 1000.0
                            self._home_set = True
                            self._safety.set_home(self._home_lat, self._home_lon)
                            print(
                                f"[MAVLink] Home position: "
                                f"({self._home_lat:.6f}, {self._home_lon:.6f})"
                                f"  alt={self._home_alt:.1f}m MSL"
                            )

                    elif t == "FENCE_STATUS":   # C1: ArduPilot onboard fence
                        self._fence_breached    = bool(msg.breach_status != 0)
                        self._fence_breach_type = msg.breach_type
                        if self._fence_breached:
                            print(f"[MAVLink] ⚠ ArduPilot FENCE BREACH "
                                  f"type={msg.breach_type} count={msg.breach_count}")

                    elif t == "PARAM_VALUE":
                        # S2.3 — cache the value and wake any fetch_param waiter.
                        try:
                            raw_name = msg.param_id
                            name = (raw_name.decode() if isinstance(raw_name, bytes)
                                    else str(raw_name)).rstrip("\x00").strip()
                            value = float(msg.param_value)
                        except Exception:
                            name, value = "", 0.0
                        if name:
                            with self._param_lock:
                                self._params[name] = value
                                ev = self._param_events.get(name)
                                if ev is not None:
                                    ev.set()

                    elif t == "COMMAND_ACK":
                        cmd_id = msg.command
                        result = msg.result
                        with self._ack_lock:
                            ev = self._ack_events.get(cmd_id)
                            if ev is not None:
                                self._ack_results[cmd_id] = result
                                ev.set()

            except Exception as e:
                import logging as _log
                _log.debug("[MAVLink] RX loop error: %s", e)

    def _read_home_position(self) -> None:
        """Request home position once on connect."""
        if self._mav is None:
            return
        try:
            self._mav.mav.command_long_send(
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                0, 0, 0, 0, 0, 0, 0, 0,
            )
        except Exception:
            pass

    def request_home_position(self) -> None:
        """Re-request HOME_POSITION from ArduPilot (public retry path).

        Called by DroneController when home has not been received yet.
        Safe to call repeatedly; the FCU will respond with a HOME_POSITION
        message which the rx_loop will pick up and set _home_set = True.
        No-op in ground-test mode (HOME is bypassed there).
        """
        if self._ground_test:
            return
        self._read_home_position()

    # ------------------------------------------------------------------
    #  HEARTBEAT handler (filtered to the autopilot component only)
    # ------------------------------------------------------------------

    def _process_heartbeat(self, msg) -> None:
        """Process one HEARTBEAT message, filtering out peripheral sources.

        Modern ArduPilot FCUs (CubeOrangePlus etc.) emit heartbeats from
        multiple components: the autopilot (compid=MAV_COMP_ID_AUTOPILOT1=1,
        autopilot=ARDUPILOTMEGA), the IO MCU (compid=0, autopilot=INVALID),
        and sometimes other peripherals. Only the autopilot's heartbeat
        carries valid mode/armed state. Pre-filter so peripheral heartbeats
        don't:
          (a) update _mode and produce phantom "mode flapping" at the
              bench (field session 2026-05-16_18-59 was flapping between
              STABILIZE and Mode(0x4) once per second);
          (b) mask a real autopilot stall in the heartbeat watchdog
              (an IOMCU can stay healthy while the autopilot is dead).
        """
        if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return
        # On the very first autopilot heartbeat, adopt its compid as our
        # target so downstream TX paths (SET_MODE, command_long, param
        # fetch, position target) address the autopilot directly instead
        # of whatever component wait_heartbeat() happened to capture.
        src_comp = msg.get_srcComponent()
        if self._mav.target_component != src_comp:
            print(
                f"[MAVLink] Adopting autopilot compid={src_comp} "
                f"(wait_heartbeat had picked compid="
                f"{self._mav.target_component}; peripheral component)"
            )
            self._mav.target_component = src_comp

        self._hb_time = time.monotonic()
        self._armed   = bool(
            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        # C2: log mode transitions, but collapse runaway flapping so a
        # stuck RC switch can't produce 100+ lines of noise.
        new_mode = mavutil.mode_string_v10(msg)
        if new_mode != self._mode and self._mode not in ("UNKNOWN", ""):
            self._handle_mode_transition(self._mode, new_mode)
        self._mode = new_mode

    # ------------------------------------------------------------------
    #  Mode transition handler (with flap collapsing)
    # ------------------------------------------------------------------

    def _handle_mode_transition(self, prev_mode: str, new_mode: str) -> None:
        """Log a flight-mode transition with built-in flap suppression.

        Pure function of (prev_mode, new_mode, current time). Pulled out of
        the rx loop so it can be unit-tested without spinning up a real
        MAVLink connection. Mutates self._mode_xitions / self._mode_flapping
        / self._mode_flap_count / self._rc_override_latched.
        """
        now_m = time.monotonic()
        self._mode_xitions.append(now_m)
        cutoff = now_m - self._MODE_FLAP_WINDOW_S
        while self._mode_xitions and self._mode_xitions[0] < cutoff:
            self._mode_xitions.popleft()
        recent_n = len(self._mode_xitions)

        # Trigger flap mode once we cross the threshold.
        if not self._mode_flapping and recent_n >= self._MODE_FLAP_TRIGGER_N:
            self._mode_flapping = True
            self._mode_flap_count = recent_n
            print(f"[MAVLink] ⚠ Mode flapping detected "
                  f"({recent_n} transitions in "
                  f"{self._MODE_FLAP_WINDOW_S:.0f}s) — "
                  f"suppressing per-transition logs; "
                  f"likely RC pilot vs tracker conflict")
            safe_event("mode_flap_start", count=recent_n,
                       window_s=self._MODE_FLAP_WINDOW_S)

        if self._mode_flapping:
            self._mode_flap_count += 1
            if recent_n <= self._MODE_FLAP_CLEAR_N:
                print(f"[MAVLink] Mode flapping ended "
                      f"(total {self._mode_flap_count} transitions); "
                      f"now {new_mode}")
                safe_event("mode_flap_end", total=self._mode_flap_count,
                           final=new_mode)
                self._mode_flapping = False
                self._mode_flap_count = 0
        else:
            print(f"[MAVLink] Flight mode: {prev_mode} → {new_mode}")

        safe_event("mode_change", prev=prev_mode, new=new_mode)

        if prev_mode == "GUIDED" and new_mode != "GUIDED":
            if not self._mode_flapping:
                print("[MAVLink] ⚠ Left GUIDED mode — "
                      "operator override or failsafe")
            # S3.6 — latch RC override. Tracker must re-arm explicitly to clear.
            self._rc_override_latched = True
            safe_event("rc_override", prev_mode=prev_mode, new_mode=new_mode)

    # ------------------------------------------------------------------
    #  Battery cell-count auto-detection
    # ------------------------------------------------------------------

    def _detect_cell_count(self) -> None:
        """Estimate battery cell count from pack voltage (S3.4).

        If --cells N was passed at launch, that override takes priority and
        the auto-detector is bypassed (logged as confidence=override).

        Otherwise the cell count is the rounded ratio of pack voltage to
        nominal cell voltage. Confidence reflects how close the actual
        voltage falls to its rounded multiple:

            high   — within 10 % of nominal*n
            medium — within 20 %
            low    — beyond 20 %, or ratio outside the 3–6 envelope

        Falls back to DEFAULT_CELLS on low confidence.
        Must be called with self._lock held.
        """
        # S3.4 — explicit override always wins.
        override = int(getattr(self._s, "cells_override", 0))
        if 3 <= override <= 6:
            self._cell_count    = override
            self._cell_detected = True
            self._safety.set_cell_count(override)
            print(f"[MAVLink] Battery: cell_count.override n={override}S "
                  "confidence=override")
            return

        v_mv = self._vbat_mv
        # Try nominal (3700 mV) first, then full-charge (4200 mV) as fallback.
        # This handles the common case where the battery is fresh off the charger
        # (4.2 V/cell), causing the nominal reference to produce a non-integer
        # ratio that falls outside the 10 % confidence band.
        _CELL_FULL_MV = 4200  # max LiPo cell voltage
        _CELL_MAX_MV  = 4250  # reject per-cell estimates above this (impossible)

        estimated  = round(v_mv / self._s.cell_nominal_mv)
        confidence = "low"
        for ref_mv in (self._s.cell_nominal_mv, _CELL_FULL_MV):
            ratio = v_mv / ref_mv
            est   = round(ratio)
            if not (3 <= est <= 6):
                continue
            if v_mv / est > _CELL_MAX_MV:
                continue
            delta = abs(ratio - est)
            if delta < 0.1:
                estimated, confidence = est, "high"
                break
            if delta < 0.2 and confidence != "high":
                estimated, confidence = est, "medium"

        if 3 <= estimated <= 6 and confidence != "low":
            self._cell_count     = estimated
            self._cell_detected  = True
            self._safety.set_cell_count(estimated)
            print(f"[MAVLink] Battery: cell_count.detected n={estimated}S "
                  f"v_per_cell={v_mv/estimated:.0f}mV confidence={confidence}")
        elif not self._cell_ambig_warned:
            # First time we see ambiguity, tell the operator how to fix it.
            # Stay silent on subsequent SYS_STATUS messages so the log isn't
            # spammed at ~2 Hz; detection will still re-run each tick in case
            # the voltage settles into a non-ambiguous range.
            self._cell_ambig_warned = True
            nom_mv = self._s.cell_nominal_mv
            ratio_display = v_mv / nom_mv if nom_mv > 0 else 0.0
            print(
                f"[MAVLink] Battery cell detection ambiguous "
                f"({v_mv}mV / {nom_mv}mV = {ratio_display:.2f}, "
                f"confidence={confidence}) — using default "
                f"{self._s.default_cells}S. Override with --cells N. "
                f"(suppressing further ambiguity warnings this session)"
            )

    # ------------------------------------------------------------------
    #  Telemetry getters (thread-safe)
    # ------------------------------------------------------------------

    def get_position_ned(self) -> tuple[float, float, float]:
        """NED position in metres from EKF origin."""
        with self._lock:
            return self._pos_n, self._pos_e, self._pos_d

    def get_velocity_ned(self) -> tuple[float, float, float]:
        """NED velocity in m/s."""
        with self._lock:
            return self._vel_n, self._vel_e, self._vel_d

    def get_attitude(self) -> tuple[float, float, float]:
        """Drone attitude: (roll, pitch, yaw) in radians."""
        with self._lock:
            return self._roll, self._pitch, self._yaw

    def get_gps(self) -> tuple[float, float, float]:
        """(lat_deg, lon_deg, alt_rel_m)."""
        with self._lock:
            return self._lat, self._lon, self._alt_rel

    def get_altitude_agl(self) -> float:
        """Altitude above home in metres."""
        with self._lock:
            return self._alt_rel

    def get_battery_voltage(self) -> float:
        """Pack voltage in Volts (0.0 if no data)."""
        with self._lock:
            return self._vbat_mv / 1000.0

    def get_gps_fix(self) -> int:
        """GPS fix type (0=no fix, 3=3D fix, 4=DGPS, 6=RTK)."""
        with self._lock:
            return self._gps_fix

    def get_gps_hdop(self) -> float:
        """GPS HDOP from GPS_RAW_INT (eph / 100). 99.99 if unknown."""
        with self._lock:
            return self._gps_hdop

    def get_sat_count(self) -> int:
        """Number of GPS satellites currently visible."""
        with self._lock:
            return self._n_sats

    def get_ekf_horizontal_variance(self) -> float:
        """EKF horizontal position variance from EKF_STATUS_REPORT.

        Returns 0.0 if no report has been received yet (caller should treat
        the value as informational until is_ekf_status_seen() is True).
        """
        with self._lock:
            return self._ekf_pos_var

    def is_ekf_status_seen(self) -> bool:
        """True after at least one EKF_STATUS_REPORT has been parsed."""
        with self._lock:
            return self._ekf_status_seen

    def _fresh(self, ts: float, max_age_s: float | None = None) -> bool:
        if ts <= 0.0:
            return False
        limit = self._s.telemetry_stale_s if max_age_s is None else max_age_s
        return (time.monotonic() - ts) <= limit

    def is_global_position_fresh(self) -> bool:
        with self._lock:
            return self._fresh(self._global_pos_t)

    def is_gps_raw_fresh(self) -> bool:
        with self._lock:
            return self._fresh(self._gps_raw_t)

    def is_ekf_status_fresh(self) -> bool:
        with self._lock:
            return self._fresh(self._ekf_status_t)

    def is_sys_status_fresh(self) -> bool:
        with self._lock:
            return self._fresh(self._sys_status_t)

    def get_telemetry_age_s(self, name: str) -> float:
        """Return age of a telemetry stream, or inf if never received."""
        key = name.lower()
        with self._lock:
            ts = {
                "global_position_int": self._global_pos_t,
                "gps_raw_int": self._gps_raw_t,
                "ekf_status_report": self._ekf_status_t,
                "sys_status": self._sys_status_t,
                "rc_channels": self._rc_last_t,
            }.get(key, 0.0)
        if ts <= 0.0:
            return float("inf")
        return time.monotonic() - ts

    def is_gps_ok(self) -> bool:
        """Return True if GPS quality is sufficient for autonomous flight (S2.1).

        Combines four checks:
          - fix_type >= GPS_MIN_FIX_TYPE
          - HDOP    <= GPS_MAX_HDOP
          - sats    >= GPS_MIN_SATS
          - GLOBAL_POSITION_INT, GPS_RAW_INT, and EKF_STATUS_REPORT are fresh
          - EKF horizontal variance <= EKF_MAX_VARIANCE
        """
        with self._lock:
            fix       = self._gps_fix
            hdop      = self._gps_hdop
            sats      = self._n_sats
            ekf_var   = self._ekf_pos_var
            global_ok = self._fresh(self._global_pos_t)
            gps_ok    = self._fresh(self._gps_raw_t)
            ekf_ok    = self._fresh(self._ekf_status_t)
        if not (global_ok and gps_ok and ekf_ok):
            return False
        if fix < self._s.gps_min_fix_type:
            return False
        if hdop > self._s.gps_max_hdop:
            return False
        if sats < self._s.gps_min_sats:
            return False
        if ekf_var > self._s.ekf_max_variance:
            return False
        return True

    def is_armed(self) -> bool:
        with self._lock:
            return self._armed

    def is_landed(self) -> bool:
        """Best-effort landed-state from EXTENDED_SYS_STATE.

        If EXTENDED_SYS_STATE has not arrived yet, a disarmed FCU is treated
        as landed and an armed FCU is treated as not landed.
        """
        with self._lock:
            state = self._landed_state
            armed = self._armed
        if state is None:
            return not armed
        landed = getattr(mavutil.mavlink, "MAV_LANDED_STATE_ON_GROUND", 1)
        return state == landed

    def get_landed_state(self) -> int | None:
        """Raw MAV_LANDED_STATE value, or None if not received yet."""
        with self._lock:
            return self._landed_state

    def get_last_heartbeat_time(self) -> float:
        with self._lock:
            return self._hb_time

    def get_mode(self) -> str:
        """Current ArduPilot flight mode string from HEARTBEAT."""
        with self._lock:
            return self._mode

    def get_home_position(self) -> tuple:
        """(home_lat_deg, home_lon_deg, home_alt_msl_m). All zeros if not yet received."""
        with self._lock:
            return self._home_lat, self._home_lon, self._home_alt

    def is_home_set(self) -> bool:
        """True after HOME_POSITION message has been received from ArduPilot."""
        with self._lock:
            return self._home_set

    def is_fence_breached(self) -> bool:
        """True if ArduPilot's onboard geofence is currently triggered."""
        with self._lock:
            return self._fence_breached

    def is_rc_connected(self) -> bool:
        """Return True if RC_CHANNELS has been seen recently (transmitter ON).

        Used as a preflight gate — takeoff is refused without a live RC
        link. False if no RC_CHANNELS has ever arrived, or the last one
        is older than RC_WATCHDOG_S, or chancount is below RC_MIN_CHANNELS.
        """
        with self._lock:
            last  = self._rc_last_t
            count = self._rc_chancount
        if last <= 0.0:
            return False
        if (time.monotonic() - last) > self._s.rc_watchdog_s:
            return False
        return count >= self._s.rc_min_channels

    def get_rc_age_s(self) -> float:
        """Seconds since the last RC_CHANNELS message. inf if never seen."""
        with self._lock:
            last = self._rc_last_t
        if last <= 0.0:
            return float("inf")
        return time.monotonic() - last

    def get_rc_channel_count(self) -> int:
        with self._lock:
            return self._rc_chancount

    def get_rc_rssi(self) -> int:
        """RC receiver RSSI, 0–255 (FCU-reported)."""
        with self._lock:
            return self._rc_rssi

    def is_rc_override_active(self) -> bool:
        """Return True if a GUIDED→other transition has been observed (S3.6).

        Once latched, only an explicit clear_rc_override() releases it.
        """
        with self._lock:
            return self._rc_override_latched

    def clear_rc_override(self) -> None:
        """Release the RC-override latch (called by /follow re-enable)."""
        with self._lock:
            if self._rc_override_latched:
                print("[MAVLink] RC override latch cleared (operator re-enabled follow)")
            self._rc_override_latched = False

    def is_sensors_healthy(self) -> bool:
        """True if gyro, accel, mag, and baro all report healthy in SYS_STATUS.
        """
        with self._lock:
            if not self._fresh(self._sys_status_t):
                return False
            required = _CRITICAL_SENSORS & self._sensors_present
            if required == 0:
                return False
            return bool((self._sensors_health & required) == required)

    # ------------------------------------------------------------------
    #  Parameter fetch  (S2.3 — preflight verifier)
    # ------------------------------------------------------------------

    def fetch_param(self, name: str, timeout: float = 2.0) -> Optional[float]:
        """Request an ArduPilot parameter and wait for the PARAM_VALUE reply.

        Returns the value on success, None on timeout or transport error.
        Subsequent calls for the same name return the cached value
        immediately if the FCU has already published it.

        Ground-test note: PARAM_REQUEST_READ is a read-only query that has
        no effect on the vehicle (the FCU just sends back its current
        value). Unlike command_long / SET_POSITION_TARGET, it does not
        need to be suppressed in --ground-test, and suppressing it broke
        param-based preflight on the bench — every check returned 'not
        advertised by FCU' because the request never went out. The two
        TX paths that actually command the vehicle (send_command_with_ack
        and _send_position_target_local_ned) keep their --ground-test
        guards.
        """
        if self._mav is None:
            return None

        with self._param_lock:
            if name in self._params:
                return self._params[name]
            ev = self._param_events.setdefault(name, threading.Event())
            ev.clear()

        try:
            self._mav.mav.param_request_read_send(
                self._mav.target_system,
                self._mav.target_component,
                name.encode("ascii"),
                -1,                        # use name, not index
            )
        except Exception as exc:
            print(f"[MAVLink] param_request_read({name!r}) error: {exc}")
            return None

        if not ev.wait(timeout):
            return None
        with self._param_lock:
            return self._params.get(name)

    def prefetch_params(self, names: list[str], timeout_s: float = 5.0) -> dict[str, Optional[float]]:
        """Fire PARAM_REQUEST_READ for every name in parallel, then wait
        collectively for the PARAM_VALUE replies.

        Sequential ``fetch_param`` calls each pay a per-request round-trip
        on a cold autopilot — first few would time out at 2 s while the
        FCU's param thread warmed up (field session 2026-05-16_19-18
        had 5 of 7 timeouts and only the last 2 reqs succeeded). Firing
        them all at once and then waiting amortises the warm-up across
        the batch, so a verifier that consumes the cache afterwards
        sees a complete picture in one shot.

        Returns a dict {name: value-or-None}. None means the FCU did not
        respond within ``timeout_s`` for that name. Cached values are
        returned without re-issuing a request.
        """
        if self._mav is None:
            return {name: None for name in names}

        events: dict[str, threading.Event] = {}
        with self._param_lock:
            for name in names:
                if name in self._params:
                    continue
                ev = self._param_events.setdefault(name, threading.Event())
                ev.clear()
                events[name] = ev

        for name in events:
            try:
                self._mav.mav.param_request_read_send(
                    self._mav.target_system,
                    self._mav.target_component,
                    name.encode("ascii"),
                    -1,
                )
            except Exception as exc:
                print(f"[MAVLink] param_request_read({name!r}) error: {exc}")

        deadline = time.monotonic() + timeout_s
        for ev in events.values():
            remaining = max(0.0, deadline - time.monotonic())
            if remaining > 0.0:
                ev.wait(remaining)

        with self._param_lock:
            return {name: self._params.get(name) for name in names}

    # ------------------------------------------------------------------
    #  Mode commands
    # ------------------------------------------------------------------

    def send_command_with_ack(
        self,
        command: int,
        p1: float = 0, p2: float = 0, p3: float = 0,
        p4: float = 0, p5: float = 0, p6: float = 0, p7: float = 0,
        retries: int = 3,
        timeout: float = 2.0,
    ) -> bool:
        """Send a MAVLink command_long and wait for COMMAND_ACK.

        Returns True if the autopilot accepted the command (result == 0).
        Thread-safe; can be called from any thread.
        """
        if self._mav is None:
            return False
        if self._ground_test:
            self._log_ground_test(
                f"command_long cmd={command} "
                f"params=({p1:.2f},{p2:.2f},{p3:.2f},{p4:.2f},{p5:.2f},{p6:.2f},{p7:.2f})"
            )
            return True   # pretend ACK so callers proceed identically
        with self._command_lock:
            ev = threading.Event()
            with self._ack_lock:
                self._ack_events[command] = ev
                self._ack_results.pop(command, None)
            try:
                for attempt in range(retries):
                    ev.clear()
                    try:
                        self._mav.mav.command_long_send(
                            self._mav.target_system,
                            self._mav.target_component,
                            command, 0,
                            p1, p2, p3, p4, p5, p6, p7,
                        )
                    except Exception as exc:
                        print(f"[MAVLink] send_command_with_ack send error: {exc}")
                        return False
                    if ev.wait(timeout):
                        result = self._ack_results.get(command, -1)
                        if result == 0:
                            return True
                        print(f"[MAVLink] ACK rejected command={command} result={result}")
                        return False
                    print(f"[MAVLink] ACK timeout command={command} "
                          f"attempt={attempt + 1}/{retries}")
            finally:
                with self._ack_lock:
                    self._ack_events.pop(command, None)
        return False

    # ArduCopter custom-mode numbers used with MAV_CMD_DO_SET_MODE.
    _MODE_NUMBERS = {
        "GUIDED": 4,
        "LOITER": 5,
        "RTL":    6,
        "LAND":   9,
        "BRAKE": 17,
    }

    def _set_mode(self, label: str, suffix: str = "") -> bool:
        """Switch to an ArduCopter flight mode (ACK-confirmed).

        Args:
            label:  One of ``_MODE_NUMBERS``.
            suffix: Optional trailing tag printed on success
                    (e.g. ``"E-STOP stage 1"``).

        Returns:
            True if autopilot accepted the command. False on ACK
            rejection or timeout. Note: a False return does NOT mean
            the FCU is in the WRONG mode — preflight reads the live
            HEARTBEAT-reported mode separately, so if the FCU happens
            to already be in the target mode, the arm path can still
            proceed.
        """
        ok = self.send_command_with_ack(
            _MAV_CMD_DO_SET_MODE,
            p1=_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=self._MODE_NUMBERS[label],
        )
        if ok:
            tail = f" ({suffix})" if suffix else ""
            print(f"[MAVLink] {label} mode confirmed{tail}")
            return True

        # ACK rejection or timeout — give the operator something
        # actionable rather than a single-line WARNING.
        current = self._mode if self._mode not in ("UNKNOWN", "") else "?"
        print(f"[MAVLink] WARNING: {label} mode not confirmed by autopilot "
              f"(FCU currently reports: {current})")
        print(f"[MAVLink]   Possible causes:")
        print(f"[MAVLink]     - RC mode switch overriding "
              f"(move it off conflicting modes before retry)")
        print(f"[MAVLink]     - MAVLink link congestion "
              f"(another GCS on the same TELEM port?)")
        print(f"[MAVLink]     - FCU busy (boot, calibration, "
              f"arming sequence) — wait a few seconds and retry")
        print(f"[MAVLink]   Preflight verifies the live HEARTBEAT mode "
              f"separately, so if the FCU is already in {label}, arm can")
        print(f"[MAVLink]   still proceed. Run 'preflight' from stdin to "
              f"check.")
        safe_event("mode_set_failed", target=label, current=current)
        return False

    def set_mode_guided(self) -> bool:
        """Switch to GUIDED mode (ACK-confirmed)."""
        return self._set_mode("GUIDED")

    def send_takeoff(self, altitude_m: float) -> bool:
        """Issue MAV_CMD_NAV_TAKEOFF — ascend to altitude_m AGL.

        Preconditions are the caller's responsibility — ArduCopter will
        reject the command unless the FCU is ARMED and in GUIDED. In
        --ground-test mode the command is logged but not transmitted,
        and this returns True so callers proceed identically.
        """
        ok = self.send_command_with_ack(
            _MAV_CMD_NAV_TAKEOFF,
            p7=float(altitude_m),
        )
        if ok:
            print(f"[MAVLink] TAKEOFF confirmed (alt={altitude_m:.1f} m AGL)")
        else:
            print(f"[MAVLink] WARNING: TAKEOFF not confirmed by autopilot "
                  f"(alt={altitude_m:.1f} m)")
        return ok

    # ------------------------------------------------------------------
    #  Velocity / position commands
    # ------------------------------------------------------------------

    def send_velocity_ned(
        self, vN: float, vE: float, vD: float = 0.0,
        yaw_rad: float | None = None,
    ) -> None:
        """Send a velocity-only command in Earth NED frame.

        Args:
            vN: North velocity m/s.
            vE: East  velocity m/s.
            vD: Down  velocity m/s (0 = maintain altitude).
            yaw_rad: Optional target heading in radians (0=N, π/2=E, in NED
                     yaw frame).  When provided, the drone yaws to face this
                     direction; otherwise yaw is ignored and ArduPilot keeps
                     whatever heading it has.

        Safety: velocity is clamped by SafetyMonitor before sending.
        """
        vN, vE, vD = self._safety.check_velocity(vN, vE, vD)
        if yaw_rad is None:
            mask = _MASK_VEL_ONLY
            yaw_val = 0.0
        else:
            # Clear the YAW_IGNORE bit (1024) so ArduPilot uses our yaw target.
            mask = _MASK_VEL_ONLY & ~1024
            yaw_val = float(yaw_rad)
        self._send_position_target(
            0, 0, 0,
            vN, vE, vD,
            mask,
            yaw=yaw_val,
        )

    def send_position_velocity_ned(
        self,
        pN: float, pE: float, pD: float,
        vN: float, vE: float, vD: float,
    ) -> None:
        """Send position target + velocity feedforward in Earth NED frame.

        This is the preferred tracking command: ArduPilot uses the position
        as the convergence target and the velocity as feedforward so it
        anticipates motion rather than always lagging behind.

        Args:
            pN, pE, pD: Target NED position (m from EKF origin).
            vN, vE, vD: Feedforward velocity (m/s).

        Safety: velocity components are clamped before sending.
        """
        vN, vE, vD = self._safety.check_velocity(vN, vE, vD)
        # pD is DOWN; apply altitude floor on pD (more negative = higher)
        # alt_agl = home_alt_rel - pD (approx); clamp pD to prevent going below floor
        # We just clamp the velocity; ArduPilot's FENCE_ALT_MIN is the hardware stop.
        self._send_position_target(pN, pE, pD, vN, vE, vD, _MASK_POS_VEL)

    def _send_position_target(
        self,
        pN: float, pE: float, pD: float,
        vN: float, vE: float, vD: float,
        type_mask: int,
        yaw: float = 0.0,
        yaw_rate: float = 0.0,
    ) -> None:
        """Low-level wrapper for SET_POSITION_TARGET_LOCAL_NED."""
        if self._mav is None:
            return
        if not all(math.isfinite(v) for v in (pN, pE, pD, vN, vE, vD, yaw, yaw_rate)):
            print(f"[MAVLink] WARN: Non-finite value in command — discarded "
                  f"pos=({pN:.2f},{pE:.2f},{pD:.2f}) vel=({vN:.2f},{vE:.2f},{vD:.2f})")
            return
        if self._ground_test:
            self._log_ground_test(
                f"pos_target mask=0x{type_mask:04x} "
                f"pos=({pN:.2f},{pE:.2f},{pD:.2f}) "
                f"vel=({vN:.2f},{vE:.2f},{vD:.2f}) "
                f"yaw={math.degrees(yaw):.0f}°"
            )
            return
        if not self._position_target_allowed(pN, pE, pD, vN, vE, vD, type_mask):
            return
        try:
            self._mav.mav.set_position_target_local_ned_send(
                0,                                          # time_boot_ms (unused)
                self._mav.target_system,
                self._mav.target_component,
                _MAV_FRAME_LOCAL_NED,                       # frame = 1
                type_mask,
                pN, pE, pD,                                 # position (m)
                vN, vE, vD,                                 # velocity (m/s)
                0.0, 0.0, 0.0,                              # accel (ignored)
                yaw, yaw_rate,                              # yaw target (rad), yaw rate (rad/s)
            )
        except Exception as exc:
            print(f"[MAVLink] send_position_target error: {exc}")

    def _position_target_allowed(
        self,
        pN: float, pE: float, pD: float,
        vN: float, vE: float, vD: float,
        type_mask: int,
    ) -> bool:
        """Last-chance guard for live position-target TX paths.

        Unit tests and disconnected setup helpers may seed _mav directly
        without running connect(), so strict live-flight state checks are
        relaxed only when no heartbeat has ever been received. If heartbeat
        telemetry exists but the RX loop is down, fail closed.
        """
        if self._mav is None:
            return False
        hb_t = self.get_last_heartbeat_time()
        if not self._running and hb_t <= 0.0:
            return True
        is_zero_hold = (
            type_mask == _MASK_VEL_ONLY
            and abs(vN) < 1e-6 and abs(vE) < 1e-6 and abs(vD) < 1e-6
        )
        if is_zero_hold:
            return True
        if not self._running:
            print("[MAVLink] WARN: command dropped — RX loop not running")
            return False
        if not self._safety.watchdog_heartbeat(hb_t):
            print("[MAVLink] WARN: command dropped — heartbeat stale")
            return False
        if self.get_mode() != "GUIDED":
            print(f"[MAVLink] WARN: command dropped — mode is {self.get_mode()}")
            return False
        if not self.is_armed():
            print("[MAVLink] WARN: command dropped — FCU disarmed")
            return False
        if self.is_rc_override_active():
            print("[MAVLink] WARN: command dropped — RC override latched")
            return False
        if not self.is_rc_connected():
            print("[MAVLink] WARN: command dropped — RC link lost")
            return False
        return True

    # ------------------------------------------------------------------
    #  Emergency / mode commands
    # ------------------------------------------------------------------

    def send_loiter(self) -> bool:
        """Switch to LOITER flight mode (ACK-confirmed).

        Safe failsafe action when tracking is lost — removes tracker from
        control loop and lets ArduPilot hold position independently.
        """
        return self._set_mode("LOITER")

    def send_rtl(self) -> bool:
        """Switch to RTL flight mode (ACK-confirmed).

        Used for battery-critical failsafe ONLY.
        Do NOT call this on tracking loss — use send_loiter() instead.
        """
        return self._set_mode("RTL")

    def send_brake(self) -> bool:
        """Switch to BRAKE flight mode (ACK-confirmed).

        BRAKE is the aggressive-stop mode used as the first stage of the
        software E-STOP (S1.1). Drone decelerates to a hover; no further
        operator action required for steady state.
        """
        return self._set_mode("BRAKE", "E-STOP stage 1")

    def send_land(self) -> bool:
        """Switch to LAND flight mode (ACK-confirmed).

        Second stage of the software E-STOP (S1.1) — used when the operator
        double-taps the E-STOP button or BRAKE fails to ACK.
        """
        return self._set_mode("LAND", "E-STOP stage 2")

    def send_disarm_if_landed(self) -> bool:
        """Disarm the FCU only after landed-state says it is on the ground."""
        if not self.is_landed():
            print("[MAVLink] DISARM refused — FCU does not report landed")
            return False
        ok = self.send_command_with_ack(
            _MAV_CMD_COMPONENT_ARM_DISARM,
            p1=0,   # disarm
        )
        if ok:
            print("[MAVLink] DISARM confirmed")
        else:
            print("[MAVLink] WARNING: DISARM not confirmed by autopilot")
        return ok

    def send_zero_velocity(self) -> None:
        """Send (0, 0, 0) velocity — graceful deceleration to hover."""
        self.send_velocity_ned(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    #  Ground-test (S1.2)
    # ------------------------------------------------------------------

    def is_ground_test(self) -> bool:
        """True when this client is suppressing all MAVLink TX."""
        return self._ground_test

    def _log_ground_test(self, body: str) -> None:
        """Log a would-be MAVLink TX in ground-test mode.

        Prints the intended command and re-prints the banner every 5 s so the
        operator cannot lose track of dry-run state while watching the stream.
        """
        now = time.monotonic()
        if now - self._ground_test_banner_t >= 5.0:
            print("[MAVLink] ⚠ GROUND-TEST — TX suppressed (no commands sent to FCU)")
            self._ground_test_banner_t = now
        # Individual packet lines are log-only — operator already sees the banner.
        from utils import terminal as _t
        _t.log_only(f"[MAVLink] [DRY-RUN] {body}")
