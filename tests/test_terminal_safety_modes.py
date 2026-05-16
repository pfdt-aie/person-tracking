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
    # _handle_estop now stops manual gimbal motion (B3) so the gimbal
    # controller must be addressable. MagicMock keeps the helper usable
    # for every existing _request_* test that previously ignored ctrl.
    t.ctrl = MagicMock(name="ctrl")
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
#  _handle_estop hardening (Commit C — B2 + B3)
# ----------------------------------------------------------------------

class _StubLog:
    """Capture flight-log events so tests can inspect what was emitted."""
    def __init__(self):
        self.events: list[tuple] = []
    def event(self, name, **fields):
        self.events.append((name, fields))


def _patch_flight_log(monkeypatch):
    """Install a _StubLog under utils.flight_log.get_flight_log and return it."""
    stub = _StubLog()
    monkeypatch.setattr("utils.flight_log.get_flight_log", lambda: stub)
    return stub


def test_estop_zeroes_manual_gimbal_speeds_and_calls_ctrl_stop(monkeypatch):
    """B3: an E-STOP mid-pan/tilt must also stop the manual gimbal."""
    _patch_flight_log(monkeypatch)
    t = _tracker()
    t._ts.manual_yaw_speed   = 50
    t._ts.manual_pitch_speed = -30
    t._ts.manual_key_t       = 9999999.0
    t._handle_estop("brake")
    assert t._ts.manual_yaw_speed   == 0
    assert t._ts.manual_pitch_speed == 0
    assert t._ts.manual_key_t       == 0.0
    assert t.ctrl.stop.called


def test_estop_logs_disarm_event_with_reason(monkeypatch):
    """B2: when E-STOP disarms the tracker it emits a disarm event itself."""
    log = _patch_flight_log(monkeypatch)
    t = _tracker()    # starts armed
    t._handle_estop("brake")
    names = [name for name, _ in log.events]
    assert "estop" in names
    assert "disarm" in names
    disarm = next(fields for name, fields in log.events if name == "disarm")
    assert disarm.get("reason") == "estop"
    assert disarm.get("action") == "brake"


def test_estop_when_already_disarmed_does_not_log_disarm(monkeypatch):
    """B2: idempotent — no spurious disarm event when nothing changes."""
    log = _patch_flight_log(monkeypatch)
    t = _tracker()
    t._ts.drone_armed = False     # already disarmed
    t._handle_estop("brake")
    disarm_events = [e for e in log.events if e[0] == "disarm"]
    assert disarm_events == []


def test_disarm_after_estop_does_not_double_log(monkeypatch):
    """B2: the operator's follow-up 'disarm' on an already-disarmed
    tracker must not emit a second disarm event."""
    log = _patch_flight_log(monkeypatch)
    t = _tracker()
    t.preflight = MagicMock()    # _handle_arm only consults preflight when arming
    # First action: E-STOP disarms and logs one disarm with reason=estop.
    t._handle_estop("brake")
    disarms_after_estop = [e for e in log.events if e[0] == "disarm"]
    assert len(disarms_after_estop) == 1
    # Operator now types 'disarm' — should be a no-op, no new event.
    result = t._handle_arm(False)
    assert result["status"] == "ok"
    assert result["msg"] == "already disarmed"
    disarms_total = [e for e in log.events if e[0] == "disarm"]
    assert len(disarms_total) == 1, log.events


def test_handle_arm_disarm_emits_event_when_transitioning(monkeypatch):
    """Sanity: when 'disarm' actually transitions armed → disarmed, log fires."""
    log = _patch_flight_log(monkeypatch)
    t = _tracker()    # starts armed
    result = t._handle_arm(False)
    assert result["msg"] == "disarmed"
    disarms = [e for e in log.events if e[0] == "disarm"]
    assert len(disarms) == 1
    assert disarms[0][1].get("source") == "operator"


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


def test_stdin_status_tolerates_extra_whitespace(monkeypatch, capsys):
    """B1: 'status  json' (double space) must reach json mode, not unknown."""
    cb = MagicMock(return_value={"x": 1})
    c = _make_op(handle_telemetry=cb)
    _drive_stdin(monkeypatch, c, ["status  json"])    # two spaces
    out = capsys.readouterr().out
    cb.assert_called_once_with()
    assert '"x": 1' in out
    assert "unknown" not in out


def test_stdin_status_rejects_garbage_arg_with_usage(monkeypatch, capsys):
    """B1: 'status foo' is a usage error, not silently routed to unknown."""
    cb = MagicMock(return_value={"x": 1})
    c = _make_op(handle_telemetry=cb)
    _drive_stdin(monkeypatch, c, ["status foo"])
    out = capsys.readouterr().out
    assert "Usage: status [json]" in out
    assert not cb.called
    assert "unknown" not in out


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


# ----------------------------------------------------------------------
#  Sprint A — UX clarity for arm/takeoff refusals + display-key hints
# ----------------------------------------------------------------------

def test_arm_refused_renders_each_failing_item_on_its_own_line(capsys):
    """A2 — operator must see per-item reasons under a refused arm, not
    just a comma-joined name list. Multi-part param messages (joined by
    '; ') indent each part for scannability."""
    refused = {
        "status": "error",
        "armed":  False,
        "msg":    "preflight failing: Vehicle ARMED, ArduPilot params correct",
        "items": [
            {"name": "MAVLink connected", "ok": True,  "message": "link up"},
            {"name": "Vehicle ARMED",     "ok": False, "message": "FCU reports disarmed"},
            {"name": "ArduPilot params correct", "ok": False,
             "message": "2 failing — FENCE_ENABLE: 0.0 == 1.0; SR1_RC_CHAN: 0.0 == 5.0"},
        ],
    }
    c = _make_op(handle_arm=MagicMock(return_value=refused))
    c._handle_stdin_arm(True)
    out = capsys.readouterr().out
    assert "arm refused" in out
    assert "2 preflight item(s) failing" in out
    assert "Vehicle ARMED" in out
    assert "FCU reports disarmed" in out
    # First param appears on the parent line, the rest are indented.
    assert "FENCE_ENABLE" in out
    assert "SR1_RC_CHAN" in out
    # The OK check is NOT echoed (only failing ones).
    assert "MAVLink connected" not in out


def test_arm_success_does_not_render_breakdown(capsys):
    """The breakdown is only for the refused path — a successful arm
    must not spew the (empty) failing-items list."""
    ok = {"status": "ok", "armed": True, "msg": "armed", "items": []}
    c = _make_op(handle_arm=MagicMock(return_value=ok))
    c._handle_stdin_arm(True)
    out = capsys.readouterr().out
    assert "ARMED" in out
    assert "preflight item(s) failing" not in out


def test_display_key_t_prints_hint(capsys):
    """A3 — typing 't' (a display-window shortcut) from stdin must
    produce an actionable hint, NOT silently alias to 'track on/off'."""
    c = _make_op()
    c._print_unknown("t")
    out = capsys.readouterr().out
    assert "unknown" in out
    assert "display-window shortcut" in out
    assert "track on" in out


def test_display_key_hint_is_case_insensitive(capsys):
    c = _make_op()
    c._print_unknown("T")
    out = capsys.readouterr().out
    assert "display-window shortcut" in out


def test_truly_unknown_command_still_just_says_unknown(capsys):
    """Random garbage that isn't a display-shortcut should NOT get a
    spurious hint — only single-key shortcuts trigger the hint path."""
    c = _make_op()
    c._print_unknown("blargh")
    out = capsys.readouterr().out
    assert "unknown" in out
    assert "display-window shortcut" not in out


def test_display_key_hint_does_not_alias_to_real_command():
    """Safety guardrail — hints must NOT actually execute the command.
    A stray 't' while armed must not toggle tracking off."""
    arm_cb = MagicMock()
    track_cb = MagicMock()
    c = _make_op(handle_arm=arm_cb)
    # If 't' were aliased to anything, calling _print_unknown('t')
    # would have side effects on the controller. It should not.
    c._print_unknown("t")
    arm_cb.assert_not_called()
