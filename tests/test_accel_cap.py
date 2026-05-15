"""S2.5 — horizontal acceleration cap in _apply_smoother()."""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from config.settings import load_settings
from control.drone_controller import DroneController


def _ctrl() -> DroneController:
    c = DroneController.__new__(DroneController)
    c._s = load_settings()
    return c


def test_within_cap_unchanged():
    an, ae = _ctrl()._clamp_accel(0.3, 0.3)
    assert math.isclose(an, 0.3)
    assert math.isclose(ae, 0.3)


def test_exceeds_cap_scaled_to_limit():
    an, ae = _ctrl()._clamp_accel(10.0, 0.0)
    assert math.isclose(an, cfg.MAX_ACCEL_MS2)
    assert math.isclose(ae, 0.0)


def test_diagonal_direction_preserved():
    an_in, ae_in = 8.0, 6.0   # magnitude 10
    an, ae = _ctrl()._clamp_accel(an_in, ae_in)
    mag = math.hypot(an, ae)
    assert math.isclose(mag, cfg.MAX_ACCEL_MS2, rel_tol=1e-6)
    # angle preserved
    assert math.isclose(an / mag, an_in / 10.0, rel_tol=1e-6)
    assert math.isclose(ae / mag, ae_in / 10.0, rel_tol=1e-6)


def test_zero_accel_returns_zero():
    an, ae = _ctrl()._clamp_accel(0.0, 0.0)
    assert an == 0.0 and ae == 0.0
