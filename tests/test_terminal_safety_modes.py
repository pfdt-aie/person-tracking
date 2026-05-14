"""Terminal safety mode commands fail closed before changing FCU mode."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from tracker import PersonGimbalTracker
from tracking.tracker_state import TrackerState
from tracking.state_machine import State


class _FakeMav:
    def __init__(self):
        self.calls = []
        self.connected = True

    def is_connected(self):
        return self.connected

    def send_zero_velocity(self):
        self.calls.append("zero")

    def send_brake(self):
        self.calls.append("brake")
        return True

    def send_land(self):
        self.calls.append("land")
        return True

    def send_rtl(self):
        self.calls.append("rtl")
        return True


class _FakeDroneController:
    def __init__(self):
        self.reset_calls = 0

    def reset(self):
        self.reset_calls += 1


class _FakeGsm:
    def __init__(self):
        self.reset_calls = 0

    def reset_all(self):
        self.reset_calls += 1


def _tracker():
    t = PersonGimbalTracker.__new__(PersonGimbalTracker)
    t._drone_enabled = True
    t._ts = TrackerState(drone_armed=True)
    t.mav = _FakeMav()
    t.drone_ctrl = _FakeDroneController()
    t._gsm = _FakeGsm()
    return t


def test_mode_land_disarms_body_before_land():
    t = _tracker()
    result = t._request_land()
    assert result["status"] == "ok"
    assert t._ts.drone_armed is False
    assert t.drone_ctrl.reset_calls == 1
    assert t.mav.calls == ["zero", "land"]


def test_mode_rtl_disarms_body_before_rtl():
    t = _tracker()
    result = t._request_rtl()
    assert result["status"] == "ok"
    assert t._ts.drone_armed is False
    assert t.drone_ctrl.reset_calls == 1
    assert t.mav.calls == ["zero", "rtl"]


def test_mode_brake_disarms_body_before_brake():
    t = _tracker()
    result = t._request_brake()
    assert result["status"] == "ok"
    assert t._ts.drone_armed is False
    assert t.drone_ctrl.reset_calls == 1
    assert t.mav.calls == ["zero", "brake"]


def test_manual_mode_sends_zero_velocity_and_requires_rearm():
    t = _tracker()
    t._enter_manual()
    assert t._ts.drone_armed is False
    assert t._ts.mode == "MANUAL"
    assert t._ts.state == State.WAITING
    assert t.mav.calls == ["zero"]
