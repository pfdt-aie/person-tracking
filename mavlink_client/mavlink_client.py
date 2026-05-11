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

import math
import threading
import time
from typing import Optional

import config as cfg
from config.settings import Settings, load_settings
from safety import SafetyMonitor

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
        self._armed:        bool  = False
        self._mode:         str   = "UNKNOWN"
        self._hb_time:      float = 0.0    # monotonic timestamp of last heartbeat

        self._home_lat:     float = 0.0
        self._home_lon:     float = 0.0
        self._home_alt:     float = 0.0
        self._home_set:     bool  = False

        self._cell_count:   int   = cfg.DEFAULT_CELLS
        self._cell_detected: bool = False

        # C1: ArduPilot onboard fence breach
        self._fence_breached:    bool = False
        self._fence_breach_type: int  = 0

        # C3: Sensor health from SYS_STATUS
        self._sensors_health:  int = 0
        self._sensors_present: int = 0

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
            self._mav.wait_heartbeat(timeout=10)
            self._hb_time = time.monotonic()
            print(
                f"[MAVLink] Connected — "
                f"sysid={self._mav.target_system}  "
                f"compid={self._mav.target_component}"
            )
        except Exception as exc:
            print(f"[MAVLink] Connection failed: {exc}")
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
                        self._hb_time = time.monotonic()
                        self._armed   = bool(
                            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        )
                        # C2: log mode transitions
                        new_mode = mavutil.mode_string_v10(msg)
                        if new_mode != self._mode and self._mode not in ("UNKNOWN", ""):
                            print(f"[MAVLink] Flight mode: {self._mode} → {new_mode}")
                            if self._mode == "GUIDED" and new_mode != "GUIDED":
                                print("[MAVLink] ⚠ Left GUIDED mode — "
                                      "operator override or failsafe")
                        self._mode = new_mode

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

                    elif t == "GLOBAL_POSITION_INT":
                        self._lat     = msg.lat  / 1e7
                        self._lon     = msg.lon  / 1e7
                        self._alt_msl = msg.alt  / 1000.0
                        self._alt_rel = msg.relative_alt / 1000.0

                    elif t == "GPS_RAW_INT":
                        self._gps_fix = msg.fix_type
                        self._n_sats  = msg.satellites_visible

                    elif t == "SYS_STATUS":
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

    # ------------------------------------------------------------------
    #  Battery cell-count auto-detection
    # ------------------------------------------------------------------

    def _detect_cell_count(self) -> None:
        """Estimate battery cell count from pack voltage.

        Uses nominal cell voltage (CELL_NOMINAL_MV) to divide.
        Valid range: 3–6 cells. Falls back to DEFAULT_CELLS if ambiguous.
        Must be called with self._lock held.
        """
        v_mv    = self._vbat_mv
        nominal = cfg.CELL_NOMINAL_MV  # e.g. 3700 mV

        estimated = round(v_mv / nominal)
        if 3 <= estimated <= 6:
            self._cell_count     = estimated
            self._cell_detected  = True
            self._safety.set_cell_count(estimated)
            print(f"[MAVLink] Battery: {v_mv}mV → detected {estimated}S pack")
        else:
            print(
                f"[MAVLink] Battery cell detection inconclusive "
                f"({v_mv}mV / {nominal}mV = {v_mv/nominal:.1f}) "
                f"— using default {cfg.DEFAULT_CELLS}S"
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

    def is_gps_ok(self) -> bool:
        """Return True if GPS fix meets the minimum requirement."""
        return self.get_gps_fix() >= cfg.GPS_MIN_FIX_TYPE

    def is_armed(self) -> bool:
        with self._lock:
            return self._armed

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

    def is_sensors_healthy(self) -> bool:
        """True if gyro, accel, mag, and baro all report healthy in SYS_STATUS.

        Returns True before first SYS_STATUS arrives to avoid blocking startup.
        """
        with self._lock:
            required = _CRITICAL_SENSORS & self._sensors_present
            if required == 0:
                return True   # no SYS_STATUS yet — don't block startup
            return bool((self._sensors_health & required) == required)

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

    def set_mode_guided(self) -> bool:
        """Switch to GUIDED mode (ACK-confirmed).

        Returns:
            True if autopilot accepted the command.
        """
        ok = self.send_command_with_ack(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=4,   # ArduCopter GUIDED mode number
        )
        if ok:
            print("[MAVLink] GUIDED mode confirmed")
        else:
            print("[MAVLink] WARNING: GUIDED mode not confirmed by autopilot")
        return ok

    # ------------------------------------------------------------------
    #  Velocity / position commands
    # ------------------------------------------------------------------

    def send_velocity_ned(
        self, vN: float, vE: float, vD: float = 0.0
    ) -> None:
        """Send a velocity-only command in Earth NED frame.

        Args:
            vN: North velocity m/s.
            vE: East  velocity m/s.
            vD: Down  velocity m/s (0 = maintain altitude).

        Safety: velocity is clamped by SafetyMonitor before sending.
        """
        vN, vE, vD = self._safety.check_velocity(vN, vE, vD)
        self._send_position_target(
            0, 0, 0,   # position (ignored)
            vN, vE, vD,
            _MASK_VEL_ONLY,
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
    ) -> None:
        """Low-level wrapper for SET_POSITION_TARGET_LOCAL_NED."""
        if self._mav is None:
            return
        if not all(math.isfinite(v) for v in (pN, pE, pD, vN, vE, vD)):
            print(f"[MAVLink] WARN: Non-finite value in command — discarded "
                  f"pos=({pN:.2f},{pE:.2f},{pD:.2f}) vel=({vN:.2f},{vE:.2f},{vD:.2f})")
            return
        try:
            self._mav.mav.set_position_target_local_ned_send(
                0,                                          # time_boot_ms (unused)
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,        # frame = 1
                type_mask,
                pN, pE, pD,                                 # position (m)
                vN, vE, vD,                                 # velocity (m/s)
                0.0, 0.0, 0.0,                              # accel (ignored)
                0.0, 0.0,                                   # yaw, yaw_rate (ignored)
            )
        except Exception as exc:
            print(f"[MAVLink] send_position_target error: {exc}")

    # ------------------------------------------------------------------
    #  Emergency / mode commands
    # ------------------------------------------------------------------

    def send_loiter(self) -> bool:
        """Switch to LOITER flight mode (ACK-confirmed).

        Safe failsafe action when tracking is lost — removes tracker from
        control loop and lets ArduPilot hold position independently.
        Uses MAV_CMD_DO_SET_MODE (mode=5) — same pattern as set_mode_guided().
        """
        ok = self.send_command_with_ack(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=5,   # ArduCopter LOITER mode number
        )
        if ok:
            print("[MAVLink] LOITER mode confirmed")
        else:
            print("[MAVLink] WARNING: LOITER mode not confirmed by autopilot")
        return ok

    def send_rtl(self) -> bool:
        """Switch to RTL flight mode (ACK-confirmed).

        Used for battery-critical failsafe ONLY.
        Do NOT call this on tracking loss — use send_loiter() instead.
        Uses MAV_CMD_DO_SET_MODE (mode=6) — same pattern as set_mode_guided().
        """
        ok = self.send_command_with_ack(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=6,   # ArduCopter RTL mode number
        )
        if ok:
            print("[MAVLink] RTL mode confirmed")
        else:
            print("[MAVLink] WARNING: RTL mode not confirmed by autopilot")
        return ok

    def send_zero_velocity(self) -> None:
        """Send (0, 0, 0) velocity — graceful deceleration to hover."""
        self.send_velocity_ned(0.0, 0.0, 0.0)
