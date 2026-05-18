"""Terminal safety mode commands fail closed before changing FCU mode.

Also covers SSH stdin verbs added in the operator_input.py SSH-parity
sprint (Phases 3–5): the helpers (_print_help / _print_status /
_print_preflight), the arm/disarm/estop wrappers (_handle_stdin_follow /
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
    t._ts = TrackerState(drone_following=True)
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
    assert t._ts.drone_following is False
    assert t.drone_ctrl.reset_calls == 1
    assert t.mav.calls == ["zero", "land"]


def test_mode_rtl_disarms_body_before_rtl():
    t = _tracker()
    result = t._request_rtl()
    assert result["status"] == "ok"
    assert t._ts.drone_following is False
    assert t.drone_ctrl.reset_calls == 1
    assert t.mav.calls == ["zero", "rtl"]


def test_mode_brake_disarms_body_before_brake():
    t = _tracker()
    result = t._request_brake()
    assert result["status"] == "ok"
    assert t._ts.drone_following is False
    assert t.drone_ctrl.reset_calls == 1
    assert t.mav.calls == ["zero", "brake"]


def test_manual_mode_sends_zero_velocity_and_requires_rearm():
    t = _tracker()
    t._enter_manual()
    assert t._ts.drone_following is False
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


def test_estop_logs_unfollow_event_with_reason(monkeypatch):
    """B2: when E-STOP stops body-follow it emits an unfollow event itself."""
    log = _patch_flight_log(monkeypatch)
    t = _tracker()    # starts following
    t._handle_estop("brake")
    names = [name for name, _ in log.events]
    assert "estop" in names
    assert "unfollow" in names
    unfollow = next(fields for name, fields in log.events if name == "unfollow")
    assert unfollow.get("reason") == "estop"
    assert unfollow.get("action") == "brake"


def test_estop_when_already_stopped_does_not_log_unfollow(monkeypatch):
    """B2: idempotent — no spurious unfollow event when nothing changes."""
    log = _patch_flight_log(monkeypatch)
    t = _tracker()
    t._ts.drone_following = False     # already stopped
    t._handle_estop("brake")
    unfollow_events = [e for e in log.events if e[0] == "unfollow"]
    assert unfollow_events == []


def test_unfollow_after_estop_does_not_double_log(monkeypatch):
    """B2: the operator's follow-up 'unfollow' on an already-stopped
    tracker must not emit a second unfollow event."""
    log = _patch_flight_log(monkeypatch)
    t = _tracker()
    t.preflight = MagicMock()    # _handle_follow only consults preflight when enabling
    # First action: E-STOP stops follow and logs one unfollow with reason=estop.
    t._handle_estop("brake")
    unfollow_after_estop = [e for e in log.events if e[0] == "unfollow"]
    assert len(unfollow_after_estop) == 1
    # Operator now types 'unfollow' — should be a no-op, no new event.
    result = t._handle_follow(False)
    assert result["status"] == "ok"
    assert result["msg"] == "already stopped"
    unfollow_total = [e for e in log.events if e[0] == "unfollow"]
    assert len(unfollow_total) == 1, log.events


def test_handle_follow_unfollow_emits_event_when_transitioning(monkeypatch):
    """Sanity: when 'unfollow' actually transitions following → stopped, log fires."""
    log = _patch_flight_log(monkeypatch)
    t = _tracker()    # starts following
    result = t._handle_follow(False)
    assert result["msg"] == "stopped"
    unfollow = [e for e in log.events if e[0] == "unfollow"]
    assert len(unfollow) == 1
    assert unfollow[0][1].get("source") == "operator"


# ----------------------------------------------------------------------
#  SSH stdin verbs (OperatorInputController)
# ----------------------------------------------------------------------

def _make_op(
    *,
    handle_estop=None,
    handle_follow=None,
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
        handle_estop=handle_estop, handle_follow=handle_follow,
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


# ---- _handle_stdin_follow -----------------------------------------------

def test_follow_invokes_handler_with_true_and_prints_following(capsys):
    cb = MagicMock(return_value={"status": "ok", "following": True, "msg": "ready"})
    c = _make_op(handle_follow=cb)
    c._handle_stdin_follow(True)
    cb.assert_called_once_with(True)
    assert "FOLLOWING" in capsys.readouterr().out


def test_unfollow_invokes_handler_with_false_and_prints_stopped(capsys):
    cb = MagicMock(return_value={"status": "ok", "following": False, "msg": ""})
    c = _make_op(handle_follow=cb)
    c._handle_stdin_follow(False)
    cb.assert_called_once_with(False)
    assert "FOLLOW STOPPED" in capsys.readouterr().out


def test_follow_refused_when_preflight_fails(capsys):
    cb = MagicMock(return_value={"status": "error", "following": False,
                                  "msg": "preflight failing: GPS"})
    c = _make_op(handle_follow=cb)
    c._handle_stdin_follow(True)
    out = capsys.readouterr().out
    assert "follow refused" in out
    assert "preflight failing" in out


def test_follow_without_handler_refuses(capsys):
    c = _make_op()
    c._handle_stdin_follow(True)
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


def test_stdin_follow_routes_to_handler(monkeypatch):
    cb = MagicMock(return_value={"status": "ok", "following": True, "msg": ""})
    c = _make_op(handle_follow=cb)
    _drive_stdin(monkeypatch, c, ["follow"])
    cb.assert_called_with(True)


def test_stdin_unfollow_routes_to_handler(monkeypatch):
    cb = MagicMock(return_value={"status": "ok", "following": False, "msg": ""})
    c = _make_op(handle_follow=cb)
    _drive_stdin(monkeypatch, c, ["unfollow"])
    cb.assert_called_with(False)


def test_stdin_old_arm_word_returns_unknown_with_rename_hint(monkeypatch, capsys):
    """The pre-rename word `arm` must NOT silently map to follow — a stray
    `arm` keypress shouldn't accidentally start drone-body following. It
    should return `unknown` with a hint pointing at the new word."""
    cb = MagicMock()
    c = _make_op(handle_follow=cb)
    _drive_stdin(monkeypatch, c, ["arm"])
    cb.assert_not_called()
    out = capsys.readouterr().out
    assert "unknown" in out
    assert "follow" in out   # the hint points to the new word


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
    swallowed by per-verb wrappers like _handle_stdin_follow.
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
    c = _make_op(handle_follow=MagicMock(return_value=refused))
    c._handle_stdin_follow(True)
    out = capsys.readouterr().out
    assert "follow refused" in out
    assert "2 preflight item(s) failing" in out
    assert "Vehicle ARMED" in out                 # preflight item name (FCU concept — unchanged)
    assert "FCU reports disarmed" in out          # FCU-side message (unchanged)
    # First param appears on the parent line, the rest are indented.
    assert "FENCE_ENABLE" in out
    assert "SR1_RC_CHAN" in out
    # The OK check is NOT echoed (only failing ones).
    assert "MAVLink connected" not in out


def test_follow_success_does_not_render_breakdown(capsys):
    """The breakdown is only for the refused path — a successful follow
    must not spew the (empty) failing-items list."""
    ok = {"status": "ok", "following": True, "msg": "following", "items": []}
    c = _make_op(handle_follow=MagicMock(return_value=ok))
    c._handle_stdin_follow(True)
    out = capsys.readouterr().out
    assert "FOLLOWING" in out
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
    c = _make_op(handle_follow=arm_cb)
    # If 't' were aliased to anything, calling _print_unknown('t')
    # would have side effects on the controller. It should not.
    c._print_unknown("t")
    arm_cb.assert_not_called()


# ----------------------------------------------------------------------
#  Sprint F — outdoor_only items render as [WAIT] not [FAIL]
# ----------------------------------------------------------------------

def test_preflight_renders_wait_for_outdoor_only_fails(capsys):
    """Indoor-expected failures (GPS, HOME) render as [WAIT] so the
    operator's eye is drawn to fixable issues, not the laws of physics."""
    items = [
        {"name": "MAVLink connected", "ok": True,  "message": "link up",
         "outdoor_only": False},
        {"name": "HOME position set", "ok": False, "message": "no HOME yet",
         "outdoor_only": True},
        {"name": "GPS fix OK",        "ok": False, "message": "fix=0<3",
         "outdoor_only": True},
        {"name": "Battery above critical", "ok": True, "message": "",
         "outdoor_only": False},
        {"name": "ArduPilot params correct", "ok": False,
         "message": "FENCE_ENABLE: 0.0 == 1.0", "outdoor_only": False},
    ]
    c = _make_op(handle_preflight=lambda: items)
    c._print_preflight()
    out = capsys.readouterr().out

    assert "[WAIT] HOME position set" in out
    assert "[WAIT] GPS fix OK" in out
    assert "[FAIL] ArduPilot params correct" in out   # real fail, not WAIT
    assert "[OK]   MAVLink connected" in out
    # Summary should mention the waiting items so it's clear they're
    # expected on a bench.
    assert "waiting on GPS/HOME" in out


def test_preflight_summary_count_excludes_waiting(capsys):
    """passed=N/M should reflect actually-OK items only. With 1 OK + 2
    waiting + 0 fail in a 3-item list, the summary should say 1/3 with
    a note that 2 are waiting."""
    items = [
        {"name": "MAVLink connected", "ok": True,  "message": "link up",
         "outdoor_only": False},
        {"name": "HOME position set", "ok": False, "message": "no HOME yet",
         "outdoor_only": True},
        {"name": "GPS fix OK",        "ok": False, "message": "fix=0<3",
         "outdoor_only": True},
    ]
    c = _make_op(handle_preflight=lambda: items)
    c._print_preflight()
    out = capsys.readouterr().out

    assert "passed=1/3" in out
    assert "2 waiting" in out


def test_preflight_no_outdoor_only_failures_keeps_existing_summary(capsys):
    """If nothing is outdoor-waiting, the summary stays unchanged (no
    'X waiting' tail). Backwards-compat for outdoor / live-flight runs."""
    items = [
        {"name": "MAVLink connected", "ok": True, "message": "",
         "outdoor_only": False},
        {"name": "GPS fix OK", "ok": True, "message": "fix=3 sats=14",
         "outdoor_only": True},
    ]
    c = _make_op(handle_preflight=lambda: items)
    c._print_preflight()
    out = capsys.readouterr().out

    assert "passed=2/2" in out
    assert "waiting" not in out
