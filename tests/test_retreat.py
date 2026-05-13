"""S1.4 — retreat-on-approach and hysteresis around MIN_PERSON_DRONE_SEP_M."""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
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
