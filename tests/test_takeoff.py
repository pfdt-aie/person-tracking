"""Takeoff command: bounds, preflight gate, landed gate, auto-GUIDED switch.

Gates that overlap preflight (MAVLink link, FCU ARMED, mode == GUIDED)
are exercised here through the preflight stub so the handler keeps a
single source of truth — mirrors how `_handle_follow` is tested.
"""
import io
import pathlib
import sys
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from tracker import PersonGimbalTracker
from tracking.operator_input import OperatorInputController
from tracking.tracker_state import TrackerState


class _PreflightItem:
    def __init__(self, name, ok, outdoor_only=False):
        self.name, self.ok, self.outdoor_only = name, ok, outdoor_only
    def to_dict(self):
        return {"name": self.name, "ok": self.ok, "message": "", "outdoor_only": self.outdoor_only}


class _FakePreflight:
    """Preflight stub that lets each test name the failing item(s).

    Real preflight intentionally skips ARMED/GUIDED in --ground-test
    (safety/preflight.py:103, 117). Tests here exercise the handler's
    behaviour, not the gate's content — they set ``failing`` to drive
    specific refusal paths.
    """
    def __init__(self, failing=()):
        self._failing = set(failing)

    def run(self):
        names = ("MAVLink connected", "Vehicle ARMED", "Flight mode = GUIDED",
                 "GPS fix OK", "HOME position set")
        return [_PreflightItem(n, n not in self._failing) for n in names]


class _FakeMav:
    """Just enough surface for _handle_takeoff to interact with."""
    def __init__(self, *, landed=True, mode="GUIDED",
                 guided_ok=True, takeoff_ok=True, armed=True):
        self.landed    = landed
        self.mode      = mode
        self.guided_ok = guided_ok
        self.takeoff_ok = takeoff_ok
        self.armed     = armed
        self.takeoff_calls: list[float] = []
        self.guided_calls = 0

    def is_landed(self):     return self.landed
    def get_mode(self):      return self.mode
    def is_armed(self):      return self.armed

    def set_mode_guided(self):
        self.guided_calls += 1
        if self.guided_ok:
            self.mode = "GUIDED"
        return self.guided_ok

    def send_takeoff(self, alt):
        self.takeoff_calls.append(alt)
        return self.takeoff_ok


class _StubLog:
    def __init__(self):
        self.events: list[tuple] = []
    def event(self, name, **fields):
        self.events.append((name, fields))


def _patch_flight_log(monkeypatch):
    stub = _StubLog()
    monkeypatch.setattr("utils.flight_log.get_flight_log", lambda: stub)
    return stub


def _tracker(*, preflight_failing=(), **mav_kwargs):
    t = PersonGimbalTracker.__new__(PersonGimbalTracker)
    t._drone_enabled = True
    t._ts = TrackerState()
    t.mav = _FakeMav(**mav_kwargs)
    t.preflight = _FakePreflight(failing=preflight_failing)
    return t


# ---- altitude bounds -------------------------------------------------

def test_default_altitude_is_seven_meters(monkeypatch):
    _patch_flight_log(monkeypatch)
    t = _tracker()
    result = t._handle_takeoff(None)
    assert result["status"] == "ok"
    assert result["altitude_m"] == cfg.DEFAULT_TAKEOFF_ALT_M == 7.0
    assert t.mav.takeoff_calls == [7.0]


def test_altitude_above_max_refused():
    t = _tracker()
    result = t._handle_takeoff(cfg.MAX_TAKEOFF_ALT_M + 1)
    assert result["status"] == "error"
    assert "outside" in result["msg"]
    assert t.mav.takeoff_calls == []


def test_altitude_below_min_refused():
    t = _tracker()
    result = t._handle_takeoff(0.0)
    assert result["status"] == "error"
    assert "outside" in result["msg"]
    assert t.mav.takeoff_calls == []


# ---- safety gates ----------------------------------------------------

def test_refuses_without_drone_flag():
    t = _tracker()
    t._drone_enabled = False
    result = t._handle_takeoff(7.0)
    assert result["status"] == "error"
    assert "--drone" in result["msg"]


def test_refuses_when_mavlink_disconnected():
    """MAVLink-link gate is delegated to preflight (same as `arm`)."""
    t = _tracker(preflight_failing=["MAVLink connected"])
    result = t._handle_takeoff(7.0)
    assert result["status"] == "error"
    assert "preflight failing" in result["msg"]
    assert "MAVLink connected" in result["msg"]
    assert t.mav.takeoff_calls == []


def test_refuses_when_preflight_fails():
    t = _tracker(preflight_failing=["GPS fix OK"])
    result = t._handle_takeoff(7.0)
    assert result["status"] == "error"
    assert "preflight failing" in result["msg"]
    assert "items" in result
    assert t.mav.takeoff_calls == []


def test_refuses_when_fcu_not_armed():
    """FCU-armed gate is delegated to preflight (`Vehicle ARMED`)."""
    t = _tracker(preflight_failing=["Vehicle ARMED"])
    result = t._handle_takeoff(7.0)
    assert result["status"] == "error"
    assert "preflight failing" in result["msg"]
    assert "Vehicle ARMED" in result["msg"]
    assert t.mav.takeoff_calls == []


def test_refuses_when_fcu_not_landed():
    """Landed gate is takeoff-specific (not part of preflight)."""
    t = _tracker(landed=False)
    result = t._handle_takeoff(7.0)
    assert result["status"] == "error"
    assert "landed" in result["msg"]
    assert t.mav.takeoff_calls == []


# ---- GUIDED auto-switch ----------------------------------------------

def test_auto_switches_to_guided_when_not_in_guided(monkeypatch):
    """When FCU mode is not GUIDED, handler switches BEFORE preflight."""
    _patch_flight_log(monkeypatch)
    monkeypatch.setattr("time.sleep", lambda _s: None)   # don't actually sleep
    t = _tracker(mode="LOITER")
    result = t._handle_takeoff(7.0)
    assert result["status"] == "ok"
    assert t.mav.guided_calls == 1
    assert t.mav.takeoff_calls == [7.0]


def test_auto_guided_runs_before_preflight(monkeypatch):
    """Regression: auto-GUIDED must precede preflight, since preflight
    gates on the cached mode. Verified by stubbing preflight to fail
    only on 'Flight mode = GUIDED' UNLESS set_mode_guided was called
    first.
    """
    _patch_flight_log(monkeypatch)
    monkeypatch.setattr("time.sleep", lambda _s: None)
    t = _tracker(mode="LOITER")

    # Make preflight fail on GUIDED whenever the FakeMav still reports
    # non-GUIDED at run() time. After set_mode_guided() the FakeMav
    # flips its mode to GUIDED, so a correctly-ordered handler sees
    # preflight pass.
    real_run = t.preflight.run
    def conditional_run():
        if t.mav.get_mode() != "GUIDED":
            return [_PreflightItem("Flight mode = GUIDED", False)]
        return real_run()
    t.preflight.run = conditional_run

    result = t._handle_takeoff(7.0)
    assert result["status"] == "ok", result
    assert t.mav.guided_calls == 1


def test_does_not_set_guided_when_already_guided(monkeypatch):
    _patch_flight_log(monkeypatch)
    t = _tracker(mode="GUIDED")
    t._handle_takeoff(7.0)
    assert t.mav.guided_calls == 0


def test_refuses_when_guided_switch_fails(monkeypatch):
    monkeypatch.setattr("time.sleep", lambda _s: None)
    t = _tracker(mode="LOITER", guided_ok=False)
    result = t._handle_takeoff(7.0)
    assert result["status"] == "error"
    assert "GUIDED" in result["msg"]
    assert t.mav.takeoff_calls == []


# ---- low-altitude warning + happy path -------------------------------

def test_low_altitude_succeeds_but_warns(monkeypatch):
    _patch_flight_log(monkeypatch)
    t = _tracker()
    result = t._handle_takeoff(cfg.MIN_ALT_M - 1.0)   # just below the current floor
    assert result["status"] == "ok"
    assert "WARNING" in result["msg"]
    assert str(cfg.MIN_ALT_M) in result["msg"]


def test_takeoff_emits_flight_log_event(monkeypatch):
    log = _patch_flight_log(monkeypatch)
    t = _tracker()
    t._handle_takeoff(7.0)
    takeoffs = [e for e in log.events if e[0] == "takeoff"]
    assert len(takeoffs) == 1
    assert takeoffs[0][1].get("altitude_m") == 7.0


def test_fcu_no_ack_propagates_error(monkeypatch):
    _patch_flight_log(monkeypatch)
    t = _tracker(takeoff_ok=False)
    result = t._handle_takeoff(7.0)
    assert result["status"] == "error"
    assert "ACK" in result["msg"]


# ---- stdin parsing ---------------------------------------------------

def _make_op(handle_takeoff=None, state=None):
    st = state or TrackerState()
    ctrl = MagicMock(name="ctrl")
    zoom = MagicMock(name="zoom_ctrl"); zoom.enabled = True
    recorder = MagicMock(name="recorder"); recorder.is_recording = False
    grabber = MagicMock(name="grabber"); grabber.frame_w = 1280; grabber.frame_h = 720
    return OperatorInputController(
        state=st, ctrl=ctrl, zoom_ctrl=zoom, stream=None,
        recorder=recorder, grabber=grabber,
        enter_manual=MagicMock(), enter_auto=MagicMock(),
        reset_all=MagicMock(),
        search=MagicMock(), init_scan=MagicMock(),
        expand_search=MagicMock(), lissajous=MagicMock(),
        handle_takeoff=handle_takeoff,
    )


def _drive(monkeypatch, c, lines):
    c._state.running = True
    script = "\n".join(list(lines) + ["q", ""]) + "\n"
    monkeypatch.setattr("sys.stdin", io.StringIO(script))
    c._stdin_loop()


def test_stdin_bare_takeoff_calls_handler_with_none(monkeypatch):
    cb = MagicMock(return_value={"status": "ok", "altitude_m": 7.0, "msg": ""})
    c = _make_op(handle_takeoff=cb)
    _drive(monkeypatch, c, ["takeoff"])
    cb.assert_called_once_with(None)


def test_stdin_takeoff_with_alt_passes_float(monkeypatch):
    cb = MagicMock(return_value={"status": "ok", "altitude_m": 12.5, "msg": ""})
    c = _make_op(handle_takeoff=cb)
    _drive(monkeypatch, c, ["takeoff 12.5"])
    cb.assert_called_once_with(12.5)


def test_stdin_takeoff_with_garbage_alt_prints_usage(monkeypatch, capsys):
    cb = MagicMock()
    c = _make_op(handle_takeoff=cb)
    _drive(monkeypatch, c, ["takeoff abc"])
    out = capsys.readouterr().out
    assert "Usage: takeoff" in out
    assert not cb.called


def test_stdin_takeoff_without_handler_refuses(monkeypatch, capsys):
    c = _make_op(handle_takeoff=None)
    _drive(monkeypatch, c, ["takeoff"])
    out = capsys.readouterr().out
    assert "unavailable" in out and "--drone" in out


def test_stdin_takeoff_refused_response_renders_reason(monkeypatch, capsys):
    cb = MagicMock(return_value={"status": "error", "msg": "FCU not armed"})
    c = _make_op(handle_takeoff=cb)
    _drive(monkeypatch, c, ["takeoff"])
    out = capsys.readouterr().out
    assert "refused" in out
    assert "FCU not armed" in out
