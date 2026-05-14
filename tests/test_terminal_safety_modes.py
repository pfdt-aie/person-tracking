"""Terminal safety mode commands fail closed before changing FCU mode.

Also covers SSH stdin verbs added in the operator_input.py SSH-parity
sprint (Phases 3–5): the helpers (_print_help / _print_status /
_print_preflight), the arm/disarm/estop wrappers (_handle_stdin_arm /
_handle_stdin_estop), the shared toggle helpers, and the
track-id-vs-track-on/off disambiguation inside _stdin_loop.
"""
import io
import pathlib
import sys
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from tracker import PersonGimbalTracker
from tracking.operator_input import (
    OperatorInputController,
    _ESTOP_DOUBLE_TAP_S,
    _HELP,
)
from tracking.state_machine import State
from tracking.tracker_state import TrackerState


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


# ----------------------------------------------------------------------
#  SSH stdin verbs (OperatorInputController)
# ----------------------------------------------------------------------

def _make_op(
    *,
    handle_estop=None,
    handle_arm=None,
    handle_preflight=None,
    handle_telemetry=None,
    state=None,
    stream=None,
):
    """Build an OperatorInputController with stubbed deps for unit tests."""
    st = state or TrackerState()
    ctrl = MagicMock(name="ctrl")
    zoom = MagicMock(name="zoom_ctrl"); zoom.enabled = True
    recorder = MagicMock(name="recorder"); recorder.is_recording = False
    grabber = MagicMock(name="grabber"); grabber.frame_w = 1280; grabber.frame_h = 720
    return OperatorInputController(
        state=st, ctrl=ctrl, zoom_ctrl=zoom, stream=stream,
        recorder=recorder, grabber=grabber,
        enter_manual=MagicMock(), enter_auto=MagicMock(),
        reset_all=MagicMock(),
        search=MagicMock(), init_scan=MagicMock(),
        expand_search=MagicMock(), lissajous=MagicMock(),
        handle_estop=handle_estop, handle_arm=handle_arm,
        handle_preflight=handle_preflight, handle_telemetry=handle_telemetry,
    )


# ---- _print_help -----------------------------------------------------

def test_help_lists_every_command(capsys):
    c = _make_op()
    c._print_help()
    out = capsys.readouterr().out
    for usage, _doc in _HELP:
        assert usage in out, f"help missing: {usage!r}"


# ---- _print_status ---------------------------------------------------

def test_status_calls_telemetry_callback(capsys):
    cb = MagicMock(return_value={"mode_tracker": "MANUAL", "drone_enabled": False})
    c = _make_op(handle_telemetry=cb)
    c._print_status()
    cb.assert_called_once_with()
    assert "tracker=MANUAL" in capsys.readouterr().out


def test_status_tolerates_missing_keys(capsys):
    """Most telemetry keys are optional; missing ones must render as em-dash."""
    c = _make_op(handle_telemetry=lambda: {})
    c._print_status()
    out = capsys.readouterr().out
    assert "—" in out               # explicit em-dash for None fields
    assert "status error" not in out


def test_status_without_callback_refuses(capsys):
    c = _make_op()
    c._print_status()
    assert "unavailable" in capsys.readouterr().out


def test_status_json_emits_single_line_dict(capsys):
    c = _make_op(handle_telemetry=lambda: {"x": 1, "y": "z"})
    c._print_status(json_mode=True)
    out = capsys.readouterr().out
    assert '"x": 1' in out and '"y": "z"' in out


# ---- _print_preflight ------------------------------------------------

def test_preflight_prints_each_item_and_summary(capsys):
    cb = MagicMock(return_value=[
        {"name": "MAVLink connected", "ok": True,  "message": "link up"},
        {"name": "GPS fix OK",        "ok": False, "message": "fix=2<3"},
        {"name": "Battery above critical", "ok": True, "message": ""},
    ])
    c = _make_op(handle_preflight=cb)
    c._print_preflight()
    out = capsys.readouterr().out
    assert "MAVLink connected" in out
    assert "GPS fix OK" in out
    assert "[FAIL]" in out
    assert "passed=2/3" in out


def test_preflight_without_callback_refuses(capsys):
    c = _make_op()
    c._print_preflight()
    assert "unavailable" in capsys.readouterr().out


# ---- _handle_stdin_arm -----------------------------------------------

def test_arm_invokes_handler_with_true_and_prints_armed(capsys):
    cb = MagicMock(return_value={"status": "ok", "armed": True, "msg": "ready"})
    c = _make_op(handle_arm=cb)
    c._handle_stdin_arm(True)
    cb.assert_called_once_with(True)
    assert "ARMED" in capsys.readouterr().out


def test_disarm_invokes_handler_with_false_and_prints_disarmed(capsys):
    cb = MagicMock(return_value={"status": "ok", "armed": False, "msg": ""})
    c = _make_op(handle_arm=cb)
    c._handle_stdin_arm(False)
    cb.assert_called_once_with(False)
    assert "DISARMED" in capsys.readouterr().out


def test_arm_refused_when_preflight_fails(capsys):
    cb = MagicMock(return_value={"status": "error", "armed": False,
                                  "msg": "preflight failing: GPS"})
    c = _make_op(handle_arm=cb)
    c._handle_stdin_arm(True)
    out = capsys.readouterr().out
    assert "arm refused" in out
    assert "preflight failing" in out


def test_arm_without_handler_refuses(capsys):
    c = _make_op()
    c._handle_stdin_arm(True)
    out = capsys.readouterr().out
    assert "unavailable" in out and "--drone" in out


# ---- _handle_stdin_estop --------------------------------------------

def test_estop_first_press_sends_brake(capsys):
    cb = MagicMock(side_effect=lambda action: {"status": "ok", "action": action})
    c = _make_op(handle_estop=cb)
    c._handle_stdin_estop()
    assert cb.call_args.args == ("brake",)
    assert "BRAKE" in capsys.readouterr().out


def test_estop_double_tap_within_window_escalates_to_land(monkeypatch):
    cb = MagicMock(side_effect=lambda action: {"status": "ok", "action": action})
    c = _make_op(handle_estop=cb)
    fake_now = [100.0]
    monkeypatch.setattr("time.monotonic", lambda: fake_now[0])
    c._handle_stdin_estop()
    fake_now[0] += _ESTOP_DOUBLE_TAP_S / 2.0   # inside window
    c._handle_stdin_estop()
    actions = [call.args[0] for call in cb.call_args_list]
    assert actions == ["brake", "land"]


def test_estop_second_press_outside_window_stays_brake(monkeypatch):
    cb = MagicMock(side_effect=lambda action: {"status": "ok", "action": action})
    c = _make_op(handle_estop=cb)
    fake_now = [100.0]
    monkeypatch.setattr("time.monotonic", lambda: fake_now[0])
    c._handle_stdin_estop()
    fake_now[0] += _ESTOP_DOUBLE_TAP_S + 0.5   # outside window
    c._handle_stdin_estop()
    actions = [call.args[0] for call in cb.call_args_list]
    assert actions == ["brake", "brake"]


def test_estop_without_handler_refuses(capsys):
    c = _make_op()
    c._handle_stdin_estop()
    assert "unavailable" in capsys.readouterr().out


def test_estop_handler_failure_prints_failed(capsys):
    cb = MagicMock(return_value={"status": "error", "action": "brake", "msg": "no link"})
    c = _make_op(handle_estop=cb)
    c._handle_stdin_estop()
    out = capsys.readouterr().out
    assert "FAILED" in out and "no link" in out


# ---- _zoom_pulse -----------------------------------------------------

def test_zoom_pulse_disables_autozoom_and_calls_ctrl_in():
    c = _make_op()
    c._zoom.enabled = True
    c._zoom_pulse("in")
    assert c._zoom.enabled is False
    assert c._ctrl.zoom_in.called and not c._ctrl.zoom_out.called


def test_zoom_pulse_disables_autozoom_and_calls_ctrl_out():
    c = _make_op()
    c._zoom.enabled = True
    c._zoom_pulse("out")
    assert c._zoom.enabled is False
    assert c._ctrl.zoom_out.called and not c._ctrl.zoom_in.called


def test_zoom_pulse_unknown_direction_does_not_move(capsys):
    c = _make_op()
    c._zoom_pulse("sideways")
    out = capsys.readouterr().out
    assert "unknown direction" in out
    assert not c._ctrl.zoom_in.called and not c._ctrl.zoom_out.called


# ---- _toggle_stream refusal -----------------------------------------

def test_stream_toggle_refuses_when_disabled_at_launch(capsys):
    c = _make_op(stream=None)
    c._toggle_stream(state=True)
    assert "disabled at launch" in capsys.readouterr().out


# ---- _stdin_loop dispatch -------------------------------------------

def _drive_stdin(monkeypatch, c, lines):
    """Feed lines + quit through _stdin_loop; auto-reset state.running each call."""
    c._state.running = True
    script = "\n".join(list(lines) + ["q", ""]) + "\n"
    monkeypatch.setattr("sys.stdin", io.StringIO(script))
    c._stdin_loop()


def test_stdin_track_numeric_id_locks_target(monkeypatch):
    c = _make_op()
    _drive_stdin(monkeypatch, c, ["track 5"])
    assert c._state.lock_id == 5


def test_stdin_track_on_enables_tracking(monkeypatch):
    st = TrackerState(); st.tracking_enabled = False
    c = _make_op(state=st)
    _drive_stdin(monkeypatch, c, ["track on"])
    assert st.tracking_enabled is True


def test_stdin_track_off_disables_tracking(monkeypatch):
    st = TrackerState(); st.tracking_enabled = True
    c = _make_op(state=st)
    _drive_stdin(monkeypatch, c, ["track off"])
    assert st.tracking_enabled is False


def test_stdin_track_id_does_not_collide_with_on_off(monkeypatch):
    """An ID of 7 must still lock, never get parsed as on/off."""
    c = _make_op()
    _drive_stdin(monkeypatch, c, ["track 7"])
    assert c._state.lock_id == 7
    assert c._state.tracking_enabled is True   # unchanged from default


def test_stdin_unknown_command_routes_to_print_unknown(capsys, monkeypatch):
    c = _make_op()
    _drive_stdin(monkeypatch, c, ["wibble wobble"])
    out = capsys.readouterr().out
    assert "unknown" in out
    assert "wibble wobble" in out


def test_stdin_bare_mode_prints_current(capsys, monkeypatch):
    c = _make_op()
    _drive_stdin(monkeypatch, c, ["mode"])
    out = capsys.readouterr().out
    assert "mode=" in out


def test_stdin_arm_routes_to_handler(monkeypatch):
    cb = MagicMock(return_value={"status": "ok", "armed": True, "msg": ""})
    c = _make_op(handle_arm=cb)
    _drive_stdin(monkeypatch, c, ["arm"])
    cb.assert_called_with(True)


def test_stdin_disarm_routes_to_handler(monkeypatch):
    cb = MagicMock(return_value={"status": "ok", "armed": False, "msg": ""})
    c = _make_op(handle_arm=cb)
    _drive_stdin(monkeypatch, c, ["disarm"])
    cb.assert_called_with(False)


def test_stdin_estop_routes_with_double_tap(monkeypatch):
    cb = MagicMock(side_effect=lambda action: {"status": "ok", "action": action})
    c = _make_op(handle_estop=cb)
    fake_now = [200.0]
    monkeypatch.setattr("time.monotonic", lambda: fake_now[0])
    # Drive both presses through the loop on a single call so state.running
    # stays True for the duration.
    _drive_stdin(monkeypatch, c, ["estop", "estop"])
    actions = [call.args[0] for call in cb.call_args_list]
    assert actions == ["brake", "land"]


def test_stdin_handler_exception_does_not_kill_loop(monkeypatch, capsys):
    """One bad command must not silently kill the SSH input thread.

    Before the Commit-B hardening this raised → `except Exception: break`,
    leaving the operator with a dead loop and no error shown. The fix
    catches handler exceptions, prints them, logs to flight_log, and
    continues reading commands.

    We monkeypatch a helper that has no inner try/except (_toggle_tracking)
    so the failure bubbles to the new outer guard rather than being
    swallowed by per-verb wrappers like _handle_stdin_arm.
    """
    calls = {"n": 0}
    def flaky(*a, **kw):
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("boom")
    c = _make_op()
    monkeypatch.setattr(c, "_toggle_tracking", flaky)
    _drive_stdin(monkeypatch, c, ["track on", "track on"])
    out = capsys.readouterr().out
    assert "error handling 'track on'" in out
    assert "boom" in out
    # Most important assertion: the second 'track on' reached the
    # helper, so the loop survived the first exception.
    assert calls["n"] == 2
