"""S2.6 — standoff bearing hysteresis + slew-rate limiter."""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from config.settings import load_settings
from control.drone_controller import DroneController


def _controller(drone_yaw: float = 0.0) -> DroneController:
    """Minimal DroneController for bearing tests.

    Supplies a fake _mav stub so get_attitude() returns the given drone_yaw.
    This matches the production code path: bearing init seeds from drone heading.
    """
    class _FakeMav:
        def __init__(self, yaw): self._yaw = yaw
        def get_attitude(self): return (0.0, 0.0, self._yaw)

    c = DroneController.__new__(DroneController)
    c._s = load_settings()
    c._vel_above_t = -1.0
    c._bearing_rad = 0.0
    c._bearing_init = False
    c._mav = _FakeMav(drone_yaw)
    return c


def test_first_call_seeds_from_drone_heading_not_person():
    """On first call, bearing seeds from drone yaw, not person direction.

    Person is directly North (desired=0).  Drone faces East (π/2).
    The returned bearing must be close to drone yaw (within one slew step)
    not close to the person direction, preventing an on-follow turn.
    """
    c = _controller(drone_yaw=math.pi / 2)  # drone faces East
    out = c._update_bearing(
        pN=10.0, pE=0.0, drone_pN=0.0, drone_pE=0.0, sep=10.0,
        vN_p=0.0, vE_p=0.0, now=1.0, dt=0.1,
    )
    max_step = math.radians(cfg.BEARING_SLEW_DEG_S) * 0.1
    # Output must be within one slew step of the seeded drone yaw, not at 0 (person dir)
    assert abs(out - math.pi / 2) <= max_step + 1e-9


def test_first_call_when_drone_faces_person_no_rotation():
    """If drone already faces person, no rotation on follow."""
    c = _controller(drone_yaw=0.0)   # drone faces North = person direction
    out = c._update_bearing(
        pN=10.0, pE=0.0, drone_pN=0.0, drone_pE=0.0, sep=10.0,
        vN_p=0.0, vE_p=0.0, now=1.0, dt=0.1,
    )
    assert math.isclose(out, 0.0, abs_tol=1e-6)


def test_brief_motion_does_not_switch_to_velocity_bearing():
    c = _controller(drone_yaw=0.0)
    # Init from stationary — drone faces North, person North
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
    c = _controller(drone_yaw=0.0)
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
    c = _controller(drone_yaw=0.0)
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
    c = _controller(drone_yaw=0.0)
    c._update_bearing(pN=1, pE=0, drone_pN=0, drone_pE=0, sep=1,
                      vN_p=5.0, vE_p=0.0, now=0.5, dt=0.1)
    assert c._vel_above_t >= 0.0   # latched
    # Velocity drops below threshold
    c._update_bearing(pN=1, pE=0, drone_pN=0, drone_pE=0, sep=1,
                      vN_p=0.0, vE_p=0.0, now=0.6, dt=0.1)
    assert c._vel_above_t < 0.0     # reset by sentinel
