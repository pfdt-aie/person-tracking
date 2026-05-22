"""Position target slew limiter for smooth person following."""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from config.settings import load_settings
from control.drone_controller import DroneController


def _controller(max_speed: float = 3.0) -> DroneController:
    controller = DroneController.__new__(DroneController)
    controller._s = load_settings(max_tracking_speed_ms=max_speed)
    controller._slew_tN = 0.0
    controller._slew_tE = 0.0
    controller._slew_init = False
    return controller


def test_slew_seeds_from_drone_position_and_steps_toward_target():
    controller = _controller(max_speed=3.0)

    n, e = controller._slew_position_target(
        10.0, 0.0, 0.1,
        seed_n=0.0, seed_e=0.0,
    )

    assert math.isclose(n, 0.3, abs_tol=1e-9)
    assert math.isclose(e, 0.0, abs_tol=1e-9)


def test_slew_never_moves_faster_than_tracking_speed():
    controller = _controller(max_speed=2.0)

    n1, e1 = controller._slew_position_target(
        10.0, 0.0, 0.5,
        seed_n=0.0, seed_e=0.0,
    )
    n2, e2 = controller._slew_position_target(10.0, 0.0, 0.5)

    assert math.hypot(n1, e1) <= 1.0 + 1e-9
    assert math.hypot(n2 - n1, e2 - e1) <= 1.0 + 1e-9


def test_slew_without_seed_preserves_legacy_immediate_start():
    controller = _controller(max_speed=3.0)

    n, e = controller._slew_position_target(4.0, -2.0, 0.1)

    assert (n, e) == (4.0, -2.0)
