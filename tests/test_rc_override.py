"""S3.6 — RC override detection latch."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

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
    """UNKNOWN→GUIDED transition must not trip the latch.

    _handle_mode_transition is called with prev_mode='UNKNOWN'. The guard
    condition (prev_mode == 'GUIDED') is False, so the latch stays clear.
    """
    c = _client()
    c._handle_mode_transition("UNKNOWN", "GUIDED")
    assert c.is_rc_override_active() is False


def test_latch_trips_on_guided_to_other():
    c = _client()
    c._handle_mode_transition("GUIDED", "LOITER")
    assert c.is_rc_override_active() is True


def test_latch_persists_when_guided_returns():
    """Once latched, returning to GUIDED must NOT auto-clear the latch.

    The operator must explicitly re-enable follow to clear it (S3.6).
    LOITER→GUIDED does not satisfy prev_mode=='GUIDED', so latch stays.
    """
    c = _client()
    c._handle_mode_transition("GUIDED", "LOITER")   # trips latch
    c._handle_mode_transition("LOITER", "GUIDED")   # back to GUIDED — latch must stay
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
