"""S1.1 — software E-STOP escalation, token bypass, log event."""
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gcs.stream_server import StreamServer, _ESTOP_DOUBLE_TAP_S
from config.settings import load_settings


def _server() -> StreamServer:
    return StreamServer(settings=load_settings(stream_token="secret-123"))


def test_first_press_invokes_brake():
    s = _server()
    seen = []
    s.set_estop_callback(lambda action: seen.append(action) or {"status": "ok"})
    result = s.trigger_estop()
    assert seen == ["brake"]
    assert result["action"] == "brake"
    assert result["status"] == "ok"


def test_second_press_within_window_escalates_to_land():
    s = _server()
    seen = []
    s.set_estop_callback(lambda action: seen.append(action) or {"status": "ok"})
    s.trigger_estop()
    s.trigger_estop()   # immediate second tap
    assert seen == ["brake", "land"]


def test_second_press_after_window_stays_brake():
    s = _server()
    seen = []
    s.set_estop_callback(lambda action: seen.append(action) or {"status": "ok"})
    s.trigger_estop()
    # Simulate the double-tap window expiring by rewinding the latch.
    s._estop_last_t -= (_ESTOP_DOUBLE_TAP_S + 0.5)
    s.trigger_estop()
    assert seen == ["brake", "brake"]


def test_estop_returns_error_when_callback_unwired():
    s = _server()
    # No set_estop_callback() called.
    result = s.trigger_estop()
    assert result["status"] == "error"
    assert result["action"] == "brake"
    assert "not wired" in result["msg"].lower()


def test_estop_returns_error_when_callback_raises():
    s = _server()

    def boom(_action):
        raise RuntimeError("mavlink offline")

    s.set_estop_callback(boom)
    result = s.trigger_estop()
    assert result["status"] == "error"
    assert "mavlink offline" in result["msg"]


def test_estop_path_not_in_control_paths():
    """E-STOP must bypass token auth — life safety always reachable."""
    from gcs.stream_server import _CONTROL_PATHS
    assert "/estop" not in _CONTROL_PATHS
