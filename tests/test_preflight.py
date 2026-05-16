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
            "rc_connected": True,
            "rc_count":     8,
            "rc_rssi":      210,
            "rc_age":       0.2,
            "global_fresh":  True,
            "gps_raw_fresh": True,
            "ekf_fresh":     True,
            "sys_fresh":     True,
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
                and self._kw["sats"] >= cfg.GPS_MIN_SATS
                and self._kw["global_fresh"]
                and self._kw["gps_raw_fresh"]
                and self._kw["ekf_fresh"])
    def get_gps_fix(self):             return self._kw["gps_fix"]
    def get_gps_hdop(self):            return self._kw["hdop"]
    def get_sat_count(self):           return self._kw["sats"]
    def is_global_position_fresh(self): return self._kw["global_fresh"]
    def is_gps_raw_fresh(self):         return self._kw["gps_raw_fresh"]
    def is_ekf_status_fresh(self):      return self._kw["ekf_fresh"]
    def is_sys_status_fresh(self):      return self._kw["sys_fresh"]
    def is_sensors_healthy(self):      return self._kw["sensors"]
    def is_fence_breached(self):       return self._kw["fence"]
    def get_battery_voltage(self):     return self._kw["voltage"]
    def is_rc_connected(self):         return self._kw["rc_connected"]
    def get_rc_age_s(self):            return self._kw["rc_age"]
    def get_rc_channel_count(self):    return self._kw["rc_count"]
    def get_rc_rssi(self):             return self._kw["rc_rssi"]


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


def test_stale_gps_telemetry_blocks_arm():
    pf = _pf({"global_fresh": False})
    items = {c.name: c for c in pf.run()}
    gps = items["GPS fix OK"]
    assert gps.ok is False
    assert "GLOBAL_POSITION_INT stale/missing" in gps.message


def test_stale_sys_status_blocks_arm():
    pf = _pf({"sys_fresh": False, "sensors": False})
    items = {c.name: c for c in pf.run()}
    sensors = items["IMU / mag / baro healthy"]
    assert sensors.ok is False
    assert "SYS_STATUS stale/missing" in sensors.message


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


def test_rc_disconnected_blocks_arm():
    pf = _pf({"rc_connected": False, "rc_age": 5.0, "rc_count": 0})
    items = {c.name: c for c in pf.run()}
    rc = items["RC transmitter connected"]
    assert rc.ok is False
    assert "no RC_CHANNELS" in rc.message
    assert pf.all_pass() is False


def test_rc_never_seen_shows_explicit_message():
    pf = _pf({"rc_connected": False, "rc_age": float("inf"), "rc_count": 0})
    items = {c.name: c for c in pf.run()}
    rc = items["RC transmitter connected"]
    assert rc.ok is False
    assert "never received" in rc.message
    # A5 — the "never received" case should also point at the ArduPilot
    # SR*_RC_CHAN param, because the operator's first instinct is to
    # debug the radio rather than the streaming-rate config.
    assert "SR" in rc.message and "_RC_CHAN" in rc.message


class _MultiFailVerifier:
    """Returns 4 failing params + 1 OK — matches the field scenario where
    the preflight summary used to say 'FENCE_ENABLE: ... (+3 more)' and
    left the operator guessing which 3."""
    def run(self):
        from mavlink_client.param_verifier import ParamCheck
        return [
            ParamCheck("FENCE_ENABLE", False, 0.0, "0.0 == 1.0"),
            ParamCheck("SR1_RC_CHAN", False, 0.0, "0.0 == 5.0"),
            ParamCheck("LAND_SPEED", False, 30.0, "30.0 == 50.0"),
            ParamCheck("RTL_ALT", False, 1500.0, "1500.0 == 3000.0"),
            ParamCheck("BATT_MONITOR", True, 4.0, "OK"),
        ]


def test_param_failure_enumerates_every_failing_name():
    """A1 — operator must see all failing params, not just the first + count."""
    pf = _pf(verifier=_MultiFailVerifier())
    items = {c.name: c for c in pf.run()}
    msg = items["ArduPilot params correct"].message
    # All four failing names appear in the message
    for name in ("FENCE_ENABLE", "SR1_RC_CHAN", "LAND_SPEED", "RTL_ALT"):
        assert name in msg, f"expected {name} in preflight message, got: {msg}"
    # The OK one does not appear
    assert "BATT_MONITOR" not in msg
    # The truncation phrase is gone
    assert "+3 more" not in msg
    assert "+1 more" not in msg


def test_rc_connected_shows_channel_and_rssi():
    pf = _pf({"rc_connected": True, "rc_count": 8, "rc_rssi": 210})
    items = {c.name: c for c in pf.run()}
    rc = items["RC transmitter connected"]
    assert rc.ok is True
    assert "8 channels" in rc.message
    assert "RSSI 210" in rc.message


def test_ground_test_skips_rc_check():
    """Bench rehearsal does not require a powered RC transmitter."""
    pf = _pf({"rc_connected": False, "rc_count": 0}, ground_test=True)
    items = {c.name: c for c in pf.run()}
    assert items["RC transmitter connected"].ok is True


def test_missing_verifier_fails_unless_ground_test():
    pf = PreflightCheck(_FakeMav(), _FakeSafety(), param_verifier=None)
    items = {c.name: c for c in pf.run()}
    assert items["ArduPilot params correct"].ok is False
    pf_dry = PreflightCheck(_FakeMav(), _FakeSafety(),
                            ground_test=True, param_verifier=None)
    items_dry = {c.name: c for c in pf_dry.run()}
    assert items_dry["ArduPilot params correct"].ok is True
