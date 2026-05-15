"""S2.6 — standoff bearing hysteresis + slew-rate limiter."""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from config.settings import load_settings
from control.drone_controller import DroneController


def _controller() -> DroneController:
    c = DroneController.__new__(DroneController)
    c._s = load_settings()
    c._vel_above_t = -1.0
    c._bearing_rad = 0.0
    c._bearing_init = False
    return c


def test_first_call_snaps_to_desired():
    c = _controller()
    # Stationary person N of drone — desired bearing = 0 (north)
    out = c._update_bearing(
        pN=10.0, pE=0.0, drone_pN=0.0, drone_pE=0.0, sep=10.0,
        vN_p=0.0, vE_p=0.0, now=1.0, dt=0.1,
    )
    assert math.isclose(out, 0.0, abs_tol=1e-6)


def test_brief_motion_does_not_switch_to_velocity_bearing():
    c = _controller()
    # Init from stationary
    c._update_bearing(pN=10, pE=0, drone_pN=0, drone_pE=0, sep=10,
                      vN_p=0, vE_p=0, now=0.0, dt=0.1)
    # Single high-velocity sample — sustained for < BEARING_LATCH_S
    out = c._update_bearing(
        pN=10, pE=0, drone_pN=0, drone_pE=0, sep=10,
        vN_p=0.0, vE_p=5.0,   # east motion
        now=0.2, dt=0.1,
    )
    # Should still be approximately north (drone→person bearing), not east
    assert abs(out) < math.radians(15.0)


def test_sustained_motion_switches_to_velocity_bearing():
    c = _controller()
    # Initial state — looking north
    c._update_bearing(pN=10, pE=0, drone_pN=0, drone_pE=0, sep=10,
                      vN_p=0, vE_p=0, now=0.0, dt=0.1)
    # Sustained east motion for > BEARING_LATCH_S, give plenty of time for slew to settle
    t = 0.1
    for _ in range(200):
        c._update_bearing(pN=10, pE=0, drone_pN=0, drone_pE=0, sep=10,
                          vN_p=0.0, vE_p=5.0, now=t, dt=0.1)
        t += 0.1
    # Bearing should be pointing east (~π/2)
    assert math.isclose(c._bearing_rad, math.pi / 2.0, abs_tol=math.radians(2.0))


def test_slew_rate_is_bounded():
    c = _controller()
    # Init at 0 rad
    c._update_bearing(pN=1, pE=0, drone_pN=0, drone_pE=0, sep=1,
                      vN_p=0, vE_p=0, now=0.0, dt=0.1)
    # Desired suddenly π (180°) — single dt=0.1 step should advance at most
    # BEARING_SLEW_DEG_S * 0.1 degrees.
    out = c._update_bearing(pN=-1, pE=0, drone_pN=0, drone_pE=0, sep=1,
                            vN_p=0, vE_p=0, now=0.1, dt=0.1)
    max_step_rad = math.radians(cfg.BEARING_SLEW_DEG_S) * 0.1
    assert abs(out) <= max_step_rad + 1e-9


def test_falling_edge_resets_latch_timer():
    c = _controller()
    c._update_bearing(pN=1, pE=0, drone_pN=0, drone_pE=0, sep=1,
                      vN_p=5.0, vE_p=0.0, now=0.5, dt=0.1)
    assert c._vel_above_t >= 0.0   # latched
    # Velocity drops below threshold
    c._update_bearing(pN=1, pE=0, drone_pN=0, drone_pE=0, sep=1,
                      vN_p=0.0, vE_p=0.0, now=0.6, dt=0.1)
    assert c._vel_above_t < 0.0     # reset by sentinel
