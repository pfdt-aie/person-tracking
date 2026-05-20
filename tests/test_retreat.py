"""S1.4 — retreat-on-approach and hysteresis around MIN_PERSON_DRONE_SEP_M."""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from config.settings import load_settings
from control.drone_controller import DroneController


class _FakeMav:
    def __init__(self) -> None:
        self.cmds: list[tuple] = []

    def send_velocity_ned(self, vN, vE, vD=0.0):
        self.cmds.append(("vel", vN, vE, vD))

    def send_zero_velocity(self):
        self.cmds.append(("zero",))


def _controller() -> tuple[DroneController, _FakeMav]:
    mav = _FakeMav()
    c = DroneController.__new__(DroneController)
    c._s = load_settings()
    c._mav = mav
    c._retreating = False
    return c, mav


def test_retreat_engages_when_sep_below_threshold():
    c, _ = _controller()
    assert c._should_retreat(cfg.MIN_PERSON_DRONE_SEP_M - 0.5) is True


def test_retreat_does_not_engage_when_sep_above_threshold():
    c, _ = _controller()
    assert c._should_retreat(cfg.MIN_PERSON_DRONE_SEP_M + 0.5) is False


def test_retreat_hysteresis_holds_engagement():
    """Once retreating, must NOT release until sep > MIN_SEP + RETREAT_HYSTERESIS."""
    c, _ = _controller()
    # Trip
    c._should_retreat(cfg.MIN_PERSON_DRONE_SEP_M - 1.0)
    assert c._retreating is True
    # Hover inside the hysteresis band — still retreating
    inside_band = cfg.MIN_PERSON_DRONE_SEP_M + (cfg.RETREAT_HYSTERESIS_M / 2.0)
    assert c._should_retreat(inside_band) is True


def test_retreat_releases_above_hysteresis():
    c, _ = _controller()
    c._should_retreat(cfg.MIN_PERSON_DRONE_SEP_M - 1.0)
    safe = cfg.MIN_PERSON_DRONE_SEP_M + cfg.RETREAT_HYSTERESIS_M + 0.5
    assert c._should_retreat(safe) is False


def test_retreat_velocity_points_away_from_subject():
    c, mav = _controller()
    # Person at origin, drone +N of person
    c._send_retreat_velocity(pN=0.0, pE=0.0, drone_pN=2.0, drone_pE=0.0, sep=2.0)
    assert mav.cmds[-1][0] == "vel"
    vN = mav.cmds[-1][1]
    vE = mav.cmds[-1][2]
    assert vN > 0.0           # north away
    assert abs(vE) < 1e-9     # no east component
    assert math.isclose(vN, cfg.RETREAT_SPEED_MS, rel_tol=1e-6)


def test_retreat_degenerate_zero_sep_climbs():
    c, mav = _controller()
    c._send_retreat_velocity(0.0, 0.0, 0.0, 0.0, 0.0)
    kind, vN, vE, vD = mav.cmds[-1]
    assert kind == "vel"
    assert vN == 0.0 and vE == 0.0
    assert vD < 0.0           # NED down negative = climb


# ---------------------------------------------------------------------------
# 3D separation geometry — ensures update() correctly uses sep_3d, not sep_2d
# ---------------------------------------------------------------------------

def test_3d_sep_no_retreat_at_cruise_altitude():
    """Drone at 7m alt, 3m horiz: sep_3d=7.6m > MIN_SEP=4m — must NOT retreat."""
    sep_3d = math.hypot(3.0, 7.0)   # sqrt(9+49) ≈ 7.6 m
    c, _ = _controller()
    assert c._should_retreat(sep_3d) is False


def test_3d_sep_triggers_retreat_at_low_altitude():
    """Drone at 1m alt, 3.5m horiz: sep_3d=3.64m < MIN_SEP=4m — must retreat."""
    sep_3d = math.hypot(3.5, 1.0)   # sqrt(12.25+1) ≈ 3.64 m
    c, _ = _controller()
    assert c._should_retreat(sep_3d) is True


def test_retreat_sends_command_even_at_zero_horizontal_sep():
    """Degenerate sep=0 during retreat must send a velocity command (climb)."""
    c, mav = _controller()
    # Simulate inline logic of update(): person directly below drone (sep=0)
    c._send_retreat_velocity(pN=0.0, pE=0.0, drone_pN=0.0, drone_pE=0.0, sep=0.0)
    assert len(mav.cmds) == 1, "No velocity command sent for degenerate retreat"
    kind = mav.cmds[0][0]
    assert kind == "vel", f"Expected vel command, got {kind}"
    vD = mav.cmds[0][3]
    assert vD < 0.0, f"Expected climb (vD<0) for degenerate retreat, got vD={vD}"
