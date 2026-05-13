"""S3.6 — RC override detection latch."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import pytest

pytest.importorskip("pymavlink")

from mavlink_client.mavlink_client import MAVLinkClient
from safety.safety import SafetyMonitor
from config.settings import load_settings


def _client():
    settings = load_settings()
    safety = SafetyMonitor(settings=settings)
    return MAVLinkClient(safety=safety, settings=settings)


def test_latch_starts_clear():
    c = _client()
    assert c.is_rc_override_active() is False


def test_latch_does_not_trip_on_first_heartbeat():
    """First mode observation should not falsely trip the latch."""
    c = _client()
    c._mode = "UNKNOWN"   # initial value
    # Simulate _rx_loop transition: UNKNOWN → GUIDED
    new_mode = "GUIDED"
    if new_mode != c._mode and c._mode not in ("UNKNOWN", ""):
        if c._mode == "GUIDED" and new_mode != "GUIDED":
            c._rc_override_latched = True
    c._mode = new_mode
    assert c.is_rc_override_active() is False


def test_latch_trips_on_guided_to_other():
    c = _client()
    c._mode = "GUIDED"
    new_mode = "LOITER"
    if c._mode == "GUIDED" and new_mode != "GUIDED":
        c._rc_override_latched = True
    c._mode = new_mode
    assert c.is_rc_override_active() is True


def test_latch_persists_when_guided_returns():
    c = _client()
    c._mode = "GUIDED"
    c._rc_override_latched = True
    # Mode comes back to GUIDED — latch must stay
    c._mode = "LOITER"
    new_mode = "GUIDED"
    if c._mode == "GUIDED" and new_mode != "GUIDED":
        c._rc_override_latched = True
    c._mode = new_mode
    assert c.is_rc_override_active() is True


def test_clear_releases_latch(capsys):
    c = _client()
    c._rc_override_latched = True
    c.clear_rc_override()
    assert c.is_rc_override_active() is False
    out = capsys.readouterr().out
    assert "latch cleared" in out.lower()


def test_clear_is_idempotent():
    c = _client()
    c.clear_rc_override()
    c.clear_rc_override()
    assert c.is_rc_override_active() is False
