"""S2.2 — HOME keep-out zone tests."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from safety.safety import SafetyMonitor, GeofenceCircle


def _safety() -> SafetyMonitor:
    return SafetyMonitor(geofence=GeofenceCircle(
        centre_lat=51.5, centre_lon=-0.1,
        radius_m=500.0, alt_min_m=10.0, alt_max_m=80.0,
    ))


def test_target_outside_keepout_allowed():
    s = _safety()
    assert s.check_home_keepout(20.0, 0.0) is True


def test_target_inside_keepout_rejected():
    s = _safety()
    inside = cfg.HOME_KEEPOUT_RADIUS_M - 0.5
    assert s.check_home_keepout(inside, 0.0) is False


def test_target_exactly_at_home_rejected():
    s = _safety()
    assert s.check_home_keepout(0.0, 0.0) is False


def test_target_at_boundary_allowed():
    s = _safety()
    # Just outside the radius
    boundary = cfg.HOME_KEEPOUT_RADIUS_M + 0.001
    assert s.check_home_keepout(boundary, 0.0) is True


def test_target_inside_diagonal_rejected():
    s = _safety()
    # 3m N + 3m E = ~4.24m radius — still inside 5m default
    assert s.check_home_keepout(3.0, 3.0) is False
