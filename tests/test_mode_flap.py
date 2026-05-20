"""Mode-flapping detector.

Background: real flight log gimbal_track_2026-05-16_16-30-14.log contained
100+ lines like 'GUIDED → Mode(0x4) → Mode(0xc0) → GUIDED' because the RC
pilot's mode switch was fighting the tracker's SET_MODE GUIDED. The
flap detector collapses runaway transitions into a single summary so the
log stays usable.
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from mavlink_client.mavlink_client import MAVLinkClient
from safety.safety import SafetyMonitor


def _client():
    safety = SafetyMonitor()
    return MAVLinkClient(safety=safety, device="/dev/null", baud=9600)


def test_few_transitions_log_individually(capsys):
    """Below the trigger threshold, every transition prints its own line."""
    c = _client()
    c._mode = "STABILIZE"
    c._handle_mode_transition("STABILIZE", "LOITER")
    c._handle_mode_transition("LOITER", "GUIDED")

    out = capsys.readouterr().out
    assert "STABILIZE → LOITER" in out
    assert "LOITER → GUIDED" in out
    assert "flapping" not in out.lower()


def test_runaway_transitions_collapse_into_summary(capsys):
    """Hitting the trigger N once produces one 'flapping detected' line;
    subsequent transitions while flapping is active are silent."""
    c = _client()
    # Push the trigger threshold (default 6 in 10s).
    for _ in range(15):
        c._handle_mode_transition("GUIDED", "LOITER")
        c._handle_mode_transition("LOITER", "GUIDED")

    out = capsys.readouterr().out
    # One flap-start line.
    assert out.count("Mode flapping detected") == 1
    # The per-transition lines should be heavily reduced — at most the few
    # printed before the threshold tripped.
    per_transition = out.count("Flight mode:")
    assert per_transition < c._MODE_FLAP_TRIGGER_N, (
        f"expected <{c._MODE_FLAP_TRIGGER_N} per-transition lines, got "
        f"{per_transition}"
    )


def test_flap_clears_when_window_empties(capsys, monkeypatch):
    """When transitions stop, the next transition outside the window
    should produce a 'Mode flapping ended' summary."""
    import mavlink_client.mavlink_client as mod

    fake_t = [1000.0]
    monkeypatch.setattr(mod.time, "monotonic", lambda: fake_t[0])

    c = _client()
    # Burst of transitions trips flap mode.
    for _ in range(12):
        c._handle_mode_transition("GUIDED", "LOITER")
        c._handle_mode_transition("LOITER", "GUIDED")
        fake_t[0] += 0.1
    assert c._mode_flapping is True

    # Advance well past the flap window; one more transition should
    # find an empty deque and emit the "ended" summary.
    fake_t[0] += c._MODE_FLAP_WINDOW_S + 5.0
    capsys.readouterr()  # clear prior output
    c._handle_mode_transition("LOITER", "GUIDED")

    out = capsys.readouterr().out
    assert "Mode flapping ended" in out
    assert c._mode_flapping is False


def test_left_guided_is_suppressed_during_flap(capsys):
    """The 'Left GUIDED mode' warning is itself a per-transition log; it
    should not fire on every flap cycle, only when we're not in flap mode."""
    c = _client()
    # Get into flap mode quickly.
    for _ in range(12):
        c._handle_mode_transition("GUIDED", "LOITER")
        c._handle_mode_transition("LOITER", "GUIDED")
    out = capsys.readouterr().out

    # The very first GUIDED→LOITER (before flap-trigger) should produce
    # the warning; further occurrences while flapping should not spam.
    assert out.count("Left GUIDED mode") < c._MODE_FLAP_TRIGGER_N

    # And the rc-override latch must still be set regardless of suppression.
    assert c._rc_override_latched is True
