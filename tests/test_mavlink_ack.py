"""tests/test_mavlink_ack.py — send_command_with_ack logic (P1-5 coverage).

Tests the ACK registry without real hardware by directly manipulating the
internal state that _rx_loop would populate.
"""
import sys, pathlib, threading, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from unittest.mock import MagicMock, patch
from mavlink_client.mavlink_client import MAVLinkClient


def _client_with_mock_mav() -> MAVLinkClient:
    """Return a MAVLinkClient with _mav stubbed out, _running=True."""
    from safety.safety import SafetyMonitor
    safety = SafetyMonitor()
    client = MAVLinkClient(safety=safety, device="/dev/null", baud=9600)
    mock_conn = MagicMock()
    mock_conn.target_system    = 1
    mock_conn.target_component = 1
    client._mav     = mock_conn
    client._running = True
    return client


def _simulate_ack(client: MAVLinkClient, command: int, result: int, delay: float = 0.05):
    """Simulate _rx_loop receiving a COMMAND_ACK for the given command."""
    def _deliver():
        time.sleep(delay)
        with client._ack_lock:
            ev = client._ack_events.get(command)
            if ev is not None:
                client._ack_results[command] = result
                ev.set()
    threading.Thread(target=_deliver, daemon=True).start()


def test_ack_accepted_returns_true():
    client = _client_with_mock_mav()
    CMD = 176  # MAV_CMD_DO_SET_MODE
    _simulate_ack(client, CMD, result=0, delay=0.05)
    ok = client.send_command_with_ack(CMD, p1=1, p2=4, timeout=1.0, retries=1)
    assert ok is True


def test_ack_rejected_returns_false():
    client = _client_with_mock_mav()
    CMD = 176
    _simulate_ack(client, CMD, result=4, delay=0.05)  # MAV_RESULT_FAILED = 4
    ok = client.send_command_with_ack(CMD, p1=1, p2=4, timeout=1.0, retries=1)
    assert ok is False


def test_ack_timeout_returns_false():
    client = _client_with_mock_mav()
    CMD = 176
    # No simulated ACK — should timeout
    ok = client.send_command_with_ack(CMD, timeout=0.1, retries=1)
    assert ok is False


def test_ack_retry_then_succeed():
    client = _client_with_mock_mav()
    CMD = 176
    # First attempt times out; second succeeds
    call_count = [0]
    original_send = client._mav.mav.command_long_send

    def patched_send(*args, **kwargs):
        call_count[0] += 1
        if call_count[0] >= 2:
            _simulate_ack(client, CMD, result=0, delay=0.02)

    client._mav.mav.command_long_send = patched_send
    ok = client.send_command_with_ack(CMD, timeout=0.15, retries=3)
    assert ok is True
    assert call_count[0] >= 2


def test_ack_registry_cleaned_up_on_timeout():
    client = _client_with_mock_mav()
    CMD = 999
    client.send_command_with_ack(CMD, timeout=0.05, retries=1)
    with client._ack_lock:
        assert CMD not in client._ack_events


def test_no_mav_returns_false():
    from safety.safety import SafetyMonitor
    client = MAVLinkClient(safety=SafetyMonitor(), device="/dev/null", baud=9600)
    assert client._mav is None
    ok = client.send_command_with_ack(176)
    assert ok is False


# ----------------------------------------------------------------------
#  B1 — actionable messaging when SET_MODE fails to ACK
# ----------------------------------------------------------------------

def test_set_mode_failure_prints_actionable_hint(capsys):
    """Pre-fix the only feedback on SET_MODE failure was a single-line
    'WARNING: GUIDED mode not confirmed by autopilot'. That left the
    operator with no idea what to try next. Now the warning enumerates
    the typical causes and points at the preflight gate for verification."""
    client = _client_with_mock_mav()
    client._mode = "STABILIZE"   # simulated current FCU-reported mode
    # No ACK ever arrives — set_mode_guided() will time out.
    ok = client.set_mode_guided()
    assert ok is False

    out = capsys.readouterr().out
    assert "GUIDED mode not confirmed" in out
    # The improved message includes:
    assert "FCU currently reports: STABILIZE" in out      # diagnostic context
    assert "RC mode switch overriding" in out             # cause #1
    assert "MAVLink link congestion" in out               # cause #2
    assert "FCU busy" in out                              # cause #3
    assert "Preflight verifies" in out                    # next-step hint


def test_set_mode_success_does_not_print_hint(capsys):
    """The actionable hint must only appear on failure — a successful
    SET_MODE should print one clean confirmation line, no warning block."""
    client = _client_with_mock_mav()
    CMD = 176
    _simulate_ack(client, CMD, result=0, delay=0.02)
    ok = client.set_mode_guided()
    assert ok is True

    out = capsys.readouterr().out
    assert "GUIDED mode confirmed" in out
    assert "WARNING" not in out
    assert "Possible causes" not in out
