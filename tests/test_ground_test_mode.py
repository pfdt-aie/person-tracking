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

            def param_request_read_send(self, *args):
                self._p.tx.append(("param_request_read", args))

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


def test_ground_test_DOES_emit_param_request_read():
    """Regression — PARAM_REQUEST_READ is read-only (no effect on the
    vehicle) and must NOT be suppressed in ground-test, otherwise the
    bench operator can't verify ArduPilot params. Pre-fix, fetch_param()
    short-circuited to None whenever ground_test was True, which broke
    preflight on the bench (all 7 safety params reported 'not advertised
    by FCU' in field log gimbal_track_2026-05-16_18-48-17).
    """
    c, spy = _client(ground_test=True)
    # fetch_param will block waiting for PARAM_VALUE that the spy never
    # delivers — use a tiny timeout so the test stays fast.
    val = c.fetch_param("FENCE_ENABLE", timeout=0.05)
    assert val is None, "no real reply was injected, so we expect None"
    kinds = [t[0] for t in spy.tx]
    assert "param_request_read" in kinds, (
        f"PARAM_REQUEST_READ must still be emitted in ground-test; TX log: {spy.tx}"
    )
    # And the existing control-write suppression is unaffected.
    assert "command_long" not in kinds
    assert "pos_target" not in kinds


def test_prefetch_params_sends_one_request_per_uncached_name():
    """prefetch_params fires PARAM_REQUEST_READ for every requested name
    that isn't already cached, in parallel rather than sequentially."""
    c, spy = _client(ground_test=False)
    names = ["FENCE_ENABLE", "FENCE_RADIUS", "RTL_ALT"]
    # tiny timeout — no replies arrive in this test
    result = c.prefetch_params(names, timeout_s=0.05)

    requested = [t[1][2] for t in spy.tx if t[0] == "param_request_read"]
    # param_request_read_send receives name as the 3rd positional arg
    # (target_system, target_component, name, index).
    assert sorted(requested) == sorted(n.encode("ascii") for n in names)
    assert set(result.keys()) == set(names)
    assert all(v is None for v in result.values())   # nothing answered


def test_prefetch_params_returns_cached_without_resending():
    """If a name is already in the cache, no new PARAM_REQUEST_READ is sent."""
    c, spy = _client(ground_test=False)
    c._params["FENCE_ENABLE"] = 1.0      # pre-populate cache
    result = c.prefetch_params(["FENCE_ENABLE", "RTL_ALT"], timeout_s=0.05)

    requested = [t[1][2] for t in spy.tx if t[0] == "param_request_read"]
    assert requested == [b"RTL_ALT"]                  # only the uncached name
    assert result["FENCE_ENABLE"] == 1.0
    assert result["RTL_ALT"] is None


def test_prefetch_params_works_in_ground_test():
    """Like fetch_param, prefetch is read-only and must run in --ground-test
    so the bench operator can pre-warm the param cache before preflight."""
    c, spy = _client(ground_test=True)
    c.prefetch_params(["FENCE_ENABLE"], timeout_s=0.05)
    kinds = [t[0] for t in spy.tx]
    assert "param_request_read" in kinds
    # Vehicle-control writes are still suppressed.
    assert "command_long" not in kinds
    assert "pos_target" not in kinds
