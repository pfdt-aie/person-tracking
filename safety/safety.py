"""
safety.py — Safety constraint enforcement for the tracking drone.

This module is the SINGLE authority for all safety rules. It is imported
by drone_controller.py and tracker.py. It NEVER imports from control modules.

NEVER VIOLATE these rules:
  1. Altitude floor: every altitude command clamped to >= MIN_ALT_M.
  2. Speed cap: every velocity command clamped to <= MAX_TRACKING_SPEED_MS.
  3. Geofence: position outside boundary → caller must send zero velocity.
  4. Watchdog: MAVLink heartbeat stale → caller must stop sending commands.
  5. Battery critical: per-cell voltage < CELL_CRITICAL_MV → trigger RTL.

All public methods are thread-safe (use internal lock for cell_count).
"""

import math
import threading
import time
from dataclasses import dataclass

import config as cfg
from config.settings import Settings, load_settings
from utils.flight_log import safe_event
from utils import terminal


#  Geofence definition

@dataclass
class GeofenceCircle:
    """Circular geofence centred on home position.

    Args:
        centre_lat: Home latitude (degrees).
        centre_lon: Home longitude (degrees).
        radius_m:   Maximum allowed distance from centre (metres).
        alt_min_m:  Minimum AGL altitude (metres).
        alt_max_m:  Maximum AGL altitude (metres).
    """
    centre_lat: float = 0.0
    centre_lon: float = 0.0
    radius_m:   float = cfg.GEOFENCE_RADIUS_M
    alt_min_m:  float = cfg.MIN_ALT_M
    alt_max_m:  float = cfg.MAX_ALT_M


#  Main safety monitor

class SafetyMonitor:
    """Central safety constraint enforcer.

    Usage:
        safety = SafetyMonitor()
        safety.set_home(lat, lon)

        # Before every MAVLink send:
        vN, vE, vD = safety.check_velocity(vN, vE, vD)
        if not safety.check_geofence(lat, lon, alt):
            # send zero velocity instead
        if not safety.watchdog_heartbeat(last_hb_t):
            # stop sending, alert operator

    Args:
        geofence: Geofence definition. Defaults to 500 m radius with
                  altitude limits from config.
    """

    def __init__(self, geofence: GeofenceCircle | None = None, settings: Settings | None = None) -> None:
        self._s       = settings or load_settings()
        # Build the default fence from Settings (not cfg) so CLI overrides
        # to radius/altitude limits propagate. Explicit fence still wins.
        self._fence   = geofence or GeofenceCircle(
            radius_m  = self._s.geofence_radius_m,
            alt_min_m = self._s.min_alt_m,
            alt_max_m = self._s.max_alt_m,
        )
        self._lock    = threading.Lock()
        self._n_cells = self._s.default_cells   # updated by mavlink_client on connect
        self._hb_warn_logged: bool = False  # latch so early-warn prints once per gap

    #  Initialisation

    def set_home(self, lat: float, lon: float) -> None:
        """Set the geofence centre to the current home position.

        Args:
            lat: Home latitude (degrees).
            lon: Home longitude (degrees).
        """
        with self._lock:
            self._fence.centre_lat = lat
            self._fence.centre_lon = lon
        terminal.event(f"[Safety] Geofence centre set: ({lat:.6f}, {lon:.6f})")

    def set_cell_count(self, n_cells: int) -> None:
        """Update detected battery cell count (called by MAVLinkClient).

        Args:
            n_cells: Number of LiPo cells (3–6 valid range).
        """
        if 3 <= n_cells <= 6:
            with self._lock:
                self._n_cells = n_cells
            terminal.event(f"[Safety] Battery: {n_cells}S detected")
        else:
            terminal.event(f"[Safety] WARNING: implausible cell count {n_cells} — ignoring")

  
    #  Rule 1: Altitude floor / ceiling


    def check_altitude(self, alt_m: float) -> float:
        """Clamp a commanded altitude to the safe operating window.

        Args:
            alt_m: Desired altitude AGL (metres).

        Returns:
            Clamped altitude. Logs a warning if clamping was needed.
        """
        floor   = self._fence.alt_min_m
        ceiling = self._fence.alt_max_m
        if alt_m < floor:
            self._log(f"Altitude {alt_m:.1f}m below floor {floor:.1f}m — clamped")
            return floor
        if alt_m > ceiling:
            self._log(f"Altitude {alt_m:.1f}m above ceiling {ceiling:.1f}m — clamped")
            return ceiling
        return alt_m


    #  Rule 2: Speed cap           

    def check_velocity(
        self, vN: float, vE: float, vD: float
    ) -> tuple[float, float, float]:
        """Hard-clamp horizontal velocity to MAX_TRACKING_SPEED_MS.

        Vertical velocity is clamped independently (half the horizontal limit
        for extra caution during altitude transitions).

        Args:
            vN, vE, vD: Desired NED velocity (m/s).

        Returns:
            (vN, vE, vD) with horizontal magnitude <= MAX_TRACKING_SPEED_MS
            and vertical magnitude <= MAX_TRACKING_SPEED_MS / 2.
        """
        max_h = self._s.max_tracking_speed_ms
        max_v = max_h / 2.0

        horiz = math.sqrt(vN**2 + vE**2)
        if horiz > max_h:
            scale = max_h / horiz
            vN   *= scale
            vE   *= scale

        vD = max(-max_v, min(max_v, vD))
        return vN, vE, vD


    #  Rule 3: Geofence


    def check_geofence(
        self, lat: float, lon: float, alt_agl: float
    ) -> bool:
        """Return False if the given position violates the geofence.

        Checks both the circular horizontal boundary and altitude limits.
        Caller must send zero velocity if this returns False.

        Args:
            lat:     Current/target latitude (degrees).
            lon:     Current/target longitude (degrees).
            alt_agl: Current/target altitude AGL (metres).

        Returns:
            True if position is safe. False if breach detected.
        """
        return self._geofence_contains(lat, lon, alt_agl, log=True)

    def geofence_contains(
        self, lat: float, lon: float, alt_agl: float
    ) -> bool:
        """Return True if a position is inside the geofence without logging.

        Use this for candidate command targets that may be checked at high rate.
        `check_geofence()` remains the logging variant for actual vehicle state.
        """
        return self._geofence_contains(lat, lon, alt_agl, log=False)

    def _geofence_contains(
        self, lat: float, lon: float, alt_agl: float, log: bool
    ) -> bool:
        with self._lock:
            fence = self._fence

        # Altitude check
        if alt_agl < fence.alt_min_m or alt_agl > fence.alt_max_m:
            if log:
                self._log(
                    f"Geofence altitude breach: {alt_agl:.1f}m "
                    f"(limits {fence.alt_min_m}–{fence.alt_max_m}m)"
                )
            return False

        # Horizontal radius check (haversine, flat-earth OK for <500 m)
        if fence.centre_lat == 0.0 and fence.centre_lon == 0.0:
            return True   # home not set yet — skip horizontal check

        R = 6_371_000.0
        dlat = math.radians(lat - fence.centre_lat)
        dlon = math.radians(lon - fence.centre_lon)
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(math.radians(fence.centre_lat))
            * math.cos(math.radians(lat))
            * math.sin(dlon / 2) ** 2
        )
        dist_m = 2 * R * math.asin(math.sqrt(a))

        if dist_m > fence.radius_m:
            if log:
                self._log(
                    f"Geofence radius breach: {dist_m:.1f}m "
                    f"(limit {fence.radius_m:.1f}m)"
                )
            return False

        return True

    #  Rule 3b: HOME keep-out (S2.2)

    def check_home_keepout(self, pN: float, pE: float) -> bool:
        """Return True if NED point (pN, pE) is OUTSIDE the HOME keep-out.

        The keep-out is a no-fly cylinder around HOME at radius
        HOME_KEEPOUT_RADIUS_M, used to prevent the drone from flying over
        the operator who is typically standing at HOME.

        Geometry note: NED origin in DroneController is set to HOME, so
        keep-out radius equals euclidean distance from the NED origin in
        the horizontal plane.

        Args:
            pN, pE: candidate target NED position (metres from HOME).

        Returns:
            True if safe (outside keep-out), False if inside.
        """
        dist = math.hypot(pN, pE)
        keepout = self._s.home_keepout_radius_m
        if dist < keepout:
            self._log(
                f"HOME keep-out breach: target {dist:.1f}m from HOME "
                f"(min {keepout:.1f}m)"
            )
            return False
        return True


    #  Rule 4: Heartbeat watchdog


    def watchdog_heartbeat(self, last_heartbeat_t: float) -> bool:
        """Return False if the MAVLink heartbeat is stale.

        Args:
            last_heartbeat_t: monotonic timestamp of the last received heartbeat.
                              <= 0.0 is the convention for "never received yet"
                              (see MAVLinkClient._hb_time initial value).

        Returns:
            True if heartbeat is fresh. False if stale (> HEARTBEAT_WATCHDOG_S)
            or never received.
        """
        watchdog = self._s.heartbeat_watchdog_s
        with self._lock:
            if last_heartbeat_t <= 0.0:
                if not self._hb_warn_logged:
                    self._log("MAVLink heartbeat lost (none received yet)")
                    self._hb_warn_logged = True
                return False
            age = time.monotonic() - last_heartbeat_t
            if age <= self._s.heartbeat_warn_s:
                self._hb_warn_logged = False
            elif age > watchdog:
                if not self._hb_warn_logged:
                    self._log(f"MAVLink heartbeat lost ({age:.1f}s stale)")
                    self._hb_warn_logged = True
                return False
            elif not self._hb_warn_logged:
                self._log(f"MAVLink heartbeat late ({age:.1f}s) — watching")
                self._hb_warn_logged = True
            return age <= watchdog


    #  Rule 5: Battery critical


    def is_battery_critical(self, voltage_v: float) -> bool:
        """Return True if per-cell voltage is below the critical threshold.

        Args:
            voltage_v: Total pack voltage (Volts).

        Returns:
            True if battery is critically low → caller should trigger RTL.
        """
        if voltage_v <= 0.0:
            return False   # no data yet — don't false-alarm
        with self._lock:
            n = self._n_cells
        per_cell = voltage_v / n
        crit_v = self._s.cell_critical_mv / 1000.0
        if per_cell < crit_v:
            self._log(
                f"Battery critical: {voltage_v:.2f}V "
                f"({per_cell:.3f}V/cell < {crit_v:.3f}V)"
            )
            return True
        return False


    #  Geofence helpers (for return-to-safe velocity in drone_controller)


    def get_fence_centre(self) -> tuple:
        """Return geofence centre (lat_deg, lon_deg)."""
        with self._lock:
            return self._fence.centre_lat, self._fence.centre_lon

    @property
    def is_home_set(self) -> bool:
        """True after set_home() has been called with valid coordinates."""
        with self._lock:
            return not (self._fence.centre_lat == 0.0
                        and self._fence.centre_lon == 0.0)


    #  Logging


    def _log(self, msg: str) -> None:
        """Print a safety violation to the terminal and emit a structured event."""
        ts = time.strftime("%H:%M:%S")
        terminal.event(f"[Safety] {ts} WARNING: {msg}")
        safe_event("safety_warning", message=msg)
