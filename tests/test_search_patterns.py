"""Search-pattern safety and coverage tests."""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from control.search_patterns import (
    ExpandingSquareSearch,
    InitialScanSearch,
    LissajousSearch,
    SectorScanSearch,
)
from tracking.gimbal_state_machine import GimbalStateMachine


# ---------------------------------------------------------------------------
# InitialScanSearch
# ---------------------------------------------------------------------------

def test_initial_scan_pretilt_searches_down():
    scan = InitialScanSearch()
    scan.start()
    _, pitch = scan.get_command()
    assert pitch < 0


def test_initial_scan_return_is_only_upward_move():
    scan = InitialScanSearch()
    scan.start()
    scan._state = "pitch_up"
    _, pitch = scan.get_command()
    assert pitch > 0


# ---------------------------------------------------------------------------
# SectorScanSearch
# ---------------------------------------------------------------------------

def test_sector_pitch_scan_searches_down():
    search = SectorScanSearch()
    search.start()
    search.pitch_scanning = True
    search.pitch_direction = -1
    _, pitch = search.get_command()
    assert pitch < 0


# ---------------------------------------------------------------------------
# ExpandingSquareSearch
# ---------------------------------------------------------------------------

def test_expanding_square_pitch_arm_searches_down():
    search = ExpandingSquareSearch()
    search.start()
    search._arm = 1
    _, pitch = search.get_command()
    assert pitch < 0


def test_expanding_square_arm_length_doubles_every_two_arms():
    """IAMSAR pattern: arm durations are 1,1,2,2,3,3,... × base_time."""
    s = ExpandingSquareSearch()
    s.start()
    durations = [s._arm_duration() for s._arm in range(8)]
    base = cfg.EXP_SQUARE_BASE_TIME
    expected = [base, base, 2*base, 2*base, 3*base, 3*base, 4*base, 4*base]
    assert durations == expected, f"Arm durations {durations} != expected {expected}"


# ---------------------------------------------------------------------------
# LissajousSearch — bidirectional pitch oscillation
# ---------------------------------------------------------------------------

def test_lissajous_pitch_is_bidirectional():
    """Pitch must oscillate both up (>0) and down (<0) over one full period.

    The ground constraint is enforced by _clamp_search_pitch_for_tilt in the
    GSM, not by restricting the command sign. (1-cos)/2 was broken — it only
    commanded downward, causing the gimbal to drift to max depression and
    turn Lissajous into a 1D yaw scan at -80°.)
    """
    search = LissajousSearch()
    search.start()
    w_p = 2.0 * math.pi / cfg.LISSAJOUS_PITCH_PERIOD
    pitches = []
    # Sample across one full pitch period
    for i in range(20):
        t = i * cfg.LISSAJOUS_PITCH_PERIOD / 20.0
        search._t0 = search._t0 - t
        _, pitch = search.get_command()
        search._t0 = search._t0 + t
        pitches.append(pitch)

    assert any(p > 0 for p in pitches), f"No upward pitch commands: {pitches}"
    assert any(p < 0 for p in pitches), f"No downward pitch commands: {pitches}"


def test_lissajous_pitch_amplitude_matches_config():
    """Peak pitch speed must equal LISSAJOUS_PITCH_SPEED."""
    search = LissajousSearch()
    search.start()
    pitches = []
    for i in range(40):
        t = i * cfg.LISSAJOUS_PITCH_PERIOD / 40.0
        search._t0 = search._t0 - t
        _, pitch = search.get_command()
        search._t0 = search._t0 + t
        pitches.append(abs(pitch))
    # Allow ±1 for int() truncation (cos slightly below 1.0 at sampled t values).
    assert max(pitches) >= cfg.LISSAJOUS_PITCH_SPEED - 1, (
        f"Peak |pitch| = {max(pitches)}, expected ~{cfg.LISSAJOUS_PITCH_SPEED}"
    )


def test_lissajous_yaw_is_bidirectional():
    """Yaw must swing both left and right over a full period."""
    search = LissajousSearch()
    search.start()
    yaws = []
    for i in range(20):
        t = i * cfg.LISSAJOUS_YAW_PERIOD / 20.0
        search._t0 = search._t0 - t
        yaw, _ = search.get_command()
        search._t0 = search._t0 + t
        yaws.append(yaw)
    assert any(y > 0 for y in yaws), f"No rightward yaw: {yaws}"
    assert any(y < 0 for y in yaws), f"No leftward yaw: {yaws}"


# ---------------------------------------------------------------------------
# GSM pitch clamp — ground constraint enforced at the clamp, not by sign
# ---------------------------------------------------------------------------

def test_clamp_stops_downward_pitch_at_steep_bound():
    """Downward command at steep limit must be suppressed (gimbal at -80°)."""
    yaw_out, pitch_out = GimbalStateMachine._clamp_search_pitch_for_tilt(
        yaw=10, pitch=-cfg.SEARCH_PITCH_SCAN_SPEED,
        tilt_deg=cfg.SEARCH_PITCH_STEEP_DEG,
    )
    assert pitch_out == 0, f"Expected 0, got {pitch_out} (down at steep bound)"
    assert yaw_out == 10


def test_clamp_stops_upward_pitch_at_shallow_bound():
    """Upward command at shallow limit must be suppressed (gimbal at -12°)."""
    yaw_out, pitch_out = GimbalStateMachine._clamp_search_pitch_for_tilt(
        yaw=5, pitch=cfg.LISSAJOUS_PITCH_SPEED,
        tilt_deg=cfg.SEARCH_PITCH_SHALLOW_DEG,
    )
    assert pitch_out == 0, f"Expected 0, got {pitch_out} (up at shallow bound)"


def test_clamp_allows_downward_within_bounds():
    """Downward pitch is allowed when not at steep limit."""
    _, pitch_out = GimbalStateMachine._clamp_search_pitch_for_tilt(
        yaw=0, pitch=-8, tilt_deg=-45.0,
    )
    assert pitch_out == -8


def test_clamp_allows_upward_within_bounds():
    """Upward pitch is allowed when not at shallow limit."""
    _, pitch_out = GimbalStateMachine._clamp_search_pitch_for_tilt(
        yaw=0, pitch=8, tilt_deg=-45.0,
    )
    assert pitch_out == 8
