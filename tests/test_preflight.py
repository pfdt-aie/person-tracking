"""S1.3 — preflight checklist + arm-gate semantics."""
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from safety.preflight import PreflightCheck


class _FakeMav:
    def __init__(self, **kw):
        self._kw = {
            "connected": True,
            "hb_age":    0.1,
            "mode":      "GUIDED",
            "armed":     True,
            "home":      True,
            "gps_fix":   3,
            "hdop":      0.8,
            "sats":      14,
            "sensors":   True,
            "fence":     False,
            "voltage":   16.0,
        }
        self._kw.update(kw)

    def is_connected(self):           return self._kw["connected"]
    def get_last_heartbeat_time(self): return time.monotonic() - self._kw["hb_age"]
    def get_mode(self):                return self._kw["mode"]
    def is_armed(self):                return self._kw["armed"]
    def is_home_set(self):             return self._kw["home"]
    def is_gps_ok(self):
        return (self._kw["gps_fix"] >= cfg.GPS_MIN_FIX_TYPE
                and self._kw["hdop"] <= cfg.GPS_MAX_HDOP
                and self._kw["sats"] >= cfg.GPS_MIN_SATS)
    def get_gps_fix(self):             return self._kw["gps_fix"]
    def get_gps_hdop(self):            return self._kw["hdop"]
    def get_sat_count(self):           return self._kw["sats"]
    def is_sensors_healthy(self):      return self._kw["sensors"]
    def is_fence_breached(self):       return self._kw["fence"]
    def get_battery_voltage(self):     return self._kw["voltage"]


class _FakeSafety:
    def watchdog_heartbeat(self, t):
        return (time.monotonic() - t) <= cfg.HEARTBEAT_WATCHDOG_S
    def is_battery_critical(self, v):
        # critical if v/cell < CELL_CRITICAL_MV/1000
        return v <= (cfg.CELL_CRITICAL_MV / 1000.0) * cfg.DEFAULT_CELLS


class _OkVerifier:
    """Stub that always reports all-params-OK for tests not focused on S2.3."""
    def run(self):
        from mavlink_client.param_verifier import ParamCheck
        return [ParamCheck("STUB", True, 1.0, "OK")]


class _FailVerifier:
    def run(self):
        from mavlink_client.param_verifier import ParamCheck
        return [ParamCheck("FENCE_ENABLE", False, 0.0, "0.0 == 1.0")]


def _pf(mav_kwargs=None, ground_test=False, verifier=None):
    mav_kwargs = mav_kwargs or {}
    return PreflightCheck(
        _FakeMav(**mav_kwargs), _FakeSafety(),
        ground_test=ground_test,
        param_verifier=verifier or _OkVerifier(),
    )


def test_all_green_passes():
    pf = _pf()
    assert pf.all_pass() is True
    assert all(c.ok for c in pf.run())


def test_disconnected_mavlink_fails():
    pf = _pf({"connected": False, "hb_age": 99.0})
    items = pf.run()
    assert not items[0].ok   # MAVLink connected
    assert not pf.all_pass()


def test_non_guided_mode_fails():
    pf = _pf({"mode": "LOITER"})
    failing = [c for c in pf.run() if not c.ok]
    assert any("GUIDED" in c.name for c in failing)


def test_disarmed_fails():
    pf = _pf({"armed": False})
    failing = [c for c in pf.run() if not c.ok]
    assert any("ARMED" in c.name for c in failing)


def test_ground_test_relaxes_guided_and_armed():
    """In dry-run mode the operator is bench-testing without flying."""
    pf = _pf({"mode": "STABILIZE", "armed": False}, ground_test=True)
    items = {c.name: c for c in pf.run()}
    assert items["Flight mode = GUIDED"].ok
    assert items["Vehicle ARMED"].ok


def test_bad_gps_fix_fails():
    pf = _pf({"gps_fix": 2})
    failing = [c for c in pf.run() if not c.ok]
    assert any("GPS" in c.name for c in failing)


def test_no_voltage_fails_battery_check():
    pf = _pf({"voltage": 0.0})
    failing = [c for c in pf.run() if not c.ok]
    assert any("Battery" in c.name for c in failing)


def test_low_voltage_fails_battery_check():
    # Below per-cell critical
    bad_v = (cfg.CELL_CRITICAL_MV - 100) / 1000.0 * cfg.DEFAULT_CELLS
    pf = _pf({"voltage": bad_v})
    failing = [c for c in pf.run() if not c.ok]
    assert any("Battery" in c.name for c in failing)


def test_to_dict_shape():
    pf = _pf()
    for item in pf.run():
        d = item.to_dict()
        assert set(d.keys()) == {"name", "ok", "message"}
        assert isinstance(d["ok"], bool)


def test_param_failure_blocks_all_pass():
    """S2.3 — wired verifier returning failures must block preflight."""
    pf = _pf(verifier=_FailVerifier())
    assert pf.all_pass() is False
    items = {c.name: c for c in pf.run()}
    assert items["ArduPilot params correct"].ok is False


def test_missing_verifier_fails_unless_ground_test():
    pf = PreflightCheck(_FakeMav(), _FakeSafety(), param_verifier=None)
    items = {c.name: c for c in pf.run()}
    assert items["ArduPilot params correct"].ok is False
    pf_dry = PreflightCheck(_FakeMav(), _FakeSafety(),
                            ground_test=True, param_verifier=None)
    items_dry = {c.name: c for c in pf_dry.run()}
    assert items_dry["ArduPilot params correct"].ok is True
