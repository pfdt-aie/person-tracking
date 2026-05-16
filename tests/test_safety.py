"""tests/test_safety.py — SafetyMonitor constraint tests."""
import math, sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from safety.safety import SafetyMonitor, GeofenceCircle


def _safety(radius_m=500.0, alt_min=10.0, alt_max=80.0) -> SafetyMonitor:
    fence = GeofenceCircle(
        centre_lat=0.0, centre_lon=0.0,
        radius_m=radius_m, alt_min_m=alt_min, alt_max_m=alt_max,
    )
    return SafetyMonitor(geofence=fence)


# --- Altitude ---

def test_altitude_within_range():
    s = _safety()
    assert s.check_altitude(30.0) == 30.0


def test_altitude_clamped_to_floor():
    s = _safety(alt_min=10.0)
    assert s.check_altitude(5.0) == 10.0


def test_altitude_clamped_to_ceiling():
    s = _safety(alt_max=80.0)
    assert s.check_altitude(100.0) == 80.0


def test_altitude_at_exact_floor():
    s = _safety(alt_min=10.0)
    assert s.check_altitude(10.0) == 10.0


# --- Speed cap ---

def test_velocity_within_limit():
    s = _safety()
    vN, vE, vD = s.check_velocity(1.0, 1.0, 0.0)
    assert math.hypot(vN, vE) <= cfg.MAX_TRACKING_SPEED_MS + 1e-9


def test_velocity_clamped_horizontal():
    s = _safety()
    big = cfg.MAX_TRACKING_SPEED_MS * 10
    vN, vE, vD = s.check_velocity(big, 0.0, 0.0)
    assert abs(vN - cfg.MAX_TRACKING_SPEED_MS) < 1e-9


def test_velocity_clamped_diagonal():
    s = _safety()
    big = cfg.MAX_TRACKING_SPEED_MS * 5
    vN, vE, vD = s.check_velocity(big, big, 0.0)
    assert math.hypot(vN, vE) <= cfg.MAX_TRACKING_SPEED_MS + 1e-9


def test_velocity_vertical_clamped():
    s = _safety()
    big = cfg.MAX_TRACKING_SPEED_MS * 10
    _, _, vD = s.check_velocity(0.0, 0.0, big)
    assert vD <= cfg.MAX_TRACKING_SPEED_MS / 2 + 1e-9


# --- Geofence ---

def test_geofence_inside():
    s = _safety(radius_m=500.0)
    s.set_home(51.5, -0.1)
    assert s.check_geofence(51.5, -0.1, 20.0) is True


def test_geofence_outside_radius():
    s = _safety(radius_m=10.0)
    s.set_home(51.5, -0.1)        # London — non-zero home avoids "not set" guard
    assert s.check_geofence(52.5, -0.1, 20.0) is False   # ~111 km away


def test_geofence_contains_matches_check_without_logging_path():
    s = _safety(radius_m=10.0)
    s.set_home(51.5, -0.1)
    assert s.geofence_contains(51.5, -0.1, 20.0) is True
    assert s.geofence_contains(52.5, -0.1, 20.0) is False


def test_geofence_altitude_violation():
    s = _safety(alt_min=10.0, alt_max=80.0)
    assert s.check_geofence(0.0, 0.0, 5.0) is False


def test_geofence_home_not_set_skips_radius():
    s = _safety()
    # centre 0,0 with flag — should skip horizontal check
    assert s.check_geofence(0.0, 0.0, 20.0) is True


# --- Battery ---

def test_battery_not_critical_above_threshold():
    s = _safety()
    vbat_mv = (cfg.CELL_CRITICAL_MV + 100) * cfg.DEFAULT_CELLS
    assert s.is_battery_critical(vbat_mv / 1000.0) is False


def test_battery_critical_below_threshold():
    s = _safety()
    vbat_mv = (cfg.CELL_CRITICAL_MV - 100) * cfg.DEFAULT_CELLS
    assert s.is_battery_critical(vbat_mv / 1000.0) is True


# --- Heartbeat watchdog ---

def test_watchdog_fresh():
    s = _safety()
    assert s.watchdog_heartbeat(time.monotonic()) is True


def test_watchdog_stale():
    s = _safety()
    stale_t = time.monotonic() - (cfg.HEARTBEAT_WATCHDOG_S + 1.0)
    assert s.watchdog_heartbeat(stale_t) is False


def test_watchdog_tightened_to_realistic_floor():
    # ArduPilot HB = 1 Hz; must allow one missed packet (>1.0 s) but be
    # substantially tighter than the original 3.0 s.
    assert cfg.HEARTBEAT_WATCHDOG_S <= 2.5
    assert cfg.HEARTBEAT_WATCHDOG_S > 1.0


def test_watchdog_warn_band_returns_true_but_logs():
    s = _safety()
    warn_age = (cfg.HEARTBEAT_WARN_S + cfg.HEARTBEAT_WATCHDOG_S) / 2.0
    t = time.monotonic() - warn_age
    assert s.watchdog_heartbeat(t) is True   # still in healthy band


def test_watchdog_never_received_returns_false():
    # MAVLinkClient._hb_time defaults to 0.0 before the first HEARTBEAT
    # arrives. The watchdog must report False (not "now-0 seconds stale").
    s = _safety()
    assert s.watchdog_heartbeat(0.0) is False
    assert s.watchdog_heartbeat(-1.0) is False
