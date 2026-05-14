"""S1.2 — --ground-test mode must suppress every MAVLink TX path."""
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import pytest

pymavlink = pytest.importorskip("pymavlink")

from mavlink_client.mavlink_client import MAVLinkClient
from safety.safety import SafetyMonitor
from config.settings import load_settings


class _SpyMavlink:
    """Pretends to be the pymavlink connection. Records every TX attempt."""
    def __init__(self):
        self.tx: list[tuple] = []
        self.target_system = 1
        self.target_component = 1

        class _Inner:
            def __init__(self, parent):
                self._p = parent

            def set_position_target_local_ned_send(self, *args):
                self._p.tx.append(("pos_target", args))

            def command_long_send(self, *args):
                self._p.tx.append(("command_long", args))

            def request_data_stream_send(self, *args):
                pass

        self.mav = _Inner(self)


def _client(ground_test: bool) -> tuple[MAVLinkClient, _SpyMavlink]:
    settings = load_settings(ground_test=ground_test)
    safety = SafetyMonitor(settings=settings)
    client = MAVLinkClient(safety=safety, settings=settings)
    spy = _SpyMavlink()
    client._mav = spy
    return client, spy


def test_ground_test_blocks_position_target():
    c, spy = _client(ground_test=True)
    c.send_position_velocity_ned(1.0, 1.0, -10.0, 0.5, 0.5, 0.0)
    assert spy.tx == [], "TX must be suppressed in ground-test mode"


def test_ground_test_blocks_velocity():
    c, spy = _client(ground_test=True)
    c.send_velocity_ned(1.0, 0.0, 0.0)
    assert spy.tx == []


def test_ground_test_blocks_mode_commands():
    c, spy = _client(ground_test=True)
    # send_loiter / send_rtl / set_mode_guided all route through
    # send_command_with_ack; verify none of them emit a TX.
    c.set_mode_guided()
    c.send_loiter()
    c.send_rtl()
    assert spy.tx == []


def test_normal_mode_does_emit_tx():
    """Sanity check the spy + client wiring with ground_test=False."""
    c, spy = _client(ground_test=False)
    c.send_velocity_ned(0.0, 0.0, 0.0)
    kinds = [t[0] for t in spy.tx]
    assert "pos_target" in kinds


def test_live_guard_drops_nonzero_command_when_rc_lost():
    c, spy = _client(ground_test=False)
    c._running = True
    c._hb_time = time.monotonic()
    c._mode = "GUIDED"
    c._armed = True
    c._rc_last_t = 0.0
    c.send_velocity_ned(1.0, 0.0, 0.0)
    assert spy.tx == []


def test_live_guard_drops_nonzero_command_when_rx_loop_stopped():
    c, spy = _client(ground_test=False)
    c._running = False
    c._hb_time = time.monotonic()
    c._mode = "GUIDED"
    c._armed = True
    c._rc_last_t = time.monotonic()
    c._rc_chancount = 8
    c.send_velocity_ned(1.0, 0.0, 0.0)
    assert spy.tx == []


def test_live_guard_allows_zero_hold_when_rc_lost():
    c, spy = _client(ground_test=False)
    c._running = True
    c._hb_time = time.monotonic()
    c._mode = "GUIDED"
    c._armed = True
    c._rc_last_t = 0.0
    c.send_zero_velocity()
    kinds = [t[0] for t in spy.tx]
    assert "pos_target" in kinds


def test_is_ground_test_flag_propagates():
    c, _ = _client(ground_test=True)
    assert c.is_ground_test() is True
    c2, _ = _client(ground_test=False)
    assert c2.is_ground_test() is False
