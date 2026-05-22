"""Regression tests for SET_POSITION_TARGET_LOCAL_NED type masks."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from config.settings import load_settings
from mavlink_client.mavlink_client import (
    MAVLinkClient,
    _MASK_POS_ONLY,
    _MASK_POS_VEL,
    _MASK_VEL_ONLY,
)
from safety.safety import SafetyMonitor


class _SpyMavlink:
    def __init__(self):
        self.tx: list[tuple] = []
        self.target_system = 1
        self.target_component = 1

        class _Inner:
            def __init__(self, parent):
                self._p = parent

            def set_position_target_local_ned_send(self, *args):
                self._p.tx.append(args)

        self.mav = _Inner(self)


def _client() -> tuple[MAVLinkClient, _SpyMavlink]:
    settings = load_settings(ground_test=False)
    safety = SafetyMonitor(settings=settings)
    client = MAVLinkClient(safety=safety, settings=settings)
    spy = _SpyMavlink()
    client._mav = spy
    return client, spy


def test_position_only_mask_ignores_velocity_and_yaw():
    client, spy = _client()

    client.send_position_ned(1.0, 2.0, -7.0)

    args = spy.tx[-1]
    assert args[4] == _MASK_POS_ONLY == 0b0000110111111000
    assert args[8:11] == (0.0, 0.0, 0.0)
    assert args[14] == 0.0
    assert args[15] == 0.0


def test_velocity_only_mask_is_not_the_yaw_active_mask():
    client, spy = _client()

    client.send_velocity_ned(0.5, 0.0, 0.0)

    args = spy.tx[-1]
    assert args[4] == _MASK_VEL_ONLY == 0b0000110111000111
    assert args[4] != 0b0000001111000111
    assert args[14] == 0.0
    assert args[15] == 0.0


def test_position_velocity_mask_still_ignores_yaw_by_default():
    client, spy = _client()

    client.send_position_velocity_ned(1.0, 2.0, -7.0, 0.2, 0.1, 0.0)

    args = spy.tx[-1]
    assert args[4] == _MASK_POS_VEL == 0b0000110111000000
    assert args[14] == 0.0
    assert args[15] == 0.0
