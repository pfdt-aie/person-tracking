"""S2.3 — ArduPilot parameter verifier."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from mavlink_client.param_verifier import (
    ParamVerifier, Requirement, default_requirements,
)


class _FakeMav:
    """Returns canned values keyed by param name; None for missing."""
    def __init__(self, values: dict):
        self._values = values

    def fetch_param(self, name, timeout=2.0):
        return self._values.get(name)


def _good_values():
    return {
        "FENCE_ENABLE":     1.0,
        "FENCE_RADIUS":     1000.0,
        "FENCE_ALT_MAX":    120.0,
        "RTL_ALT":          3000.0,
        "BATT_FS_LOW_ACT":  2.0,
        "FS_GCS_ENABLE":    1.0,
        "GUID_TIMEOUT":     5.0,
    }


def test_all_params_correct_passes():
    v = ParamVerifier(_FakeMav(_good_values()), timeout_per_param=0.0)
    assert v.all_pass() is True


def test_fence_disabled_fails():
    bad = _good_values()
    bad["FENCE_ENABLE"] = 0.0
    v = ParamVerifier(_FakeMav(bad), timeout_per_param=0.0)
    results = {c.name: c for c in v.run()}
    assert results["FENCE_ENABLE"].ok is False


def test_radius_too_small_fails():
    bad = _good_values()
    bad["FENCE_RADIUS"] = 100.0  # less than default GEOFENCE_RADIUS_M=500
    v = ParamVerifier(_FakeMav(bad), timeout_per_param=0.0)
    assert v.all_pass() is False


def test_battery_failsafe_action_too_low_fails():
    bad = _good_values()
    bad["BATT_FS_LOW_ACT"] = 1.0   # WARN only, not RTL/LAND
    v = ParamVerifier(_FakeMav(bad), timeout_per_param=0.0)
    assert v.all_pass() is False


def test_missing_param_treated_as_fail():
    bad = _good_values()
    del bad["FENCE_ENABLE"]
    v = ParamVerifier(_FakeMav(bad), timeout_per_param=0.0)
    results = {c.name: c for c in v.run()}
    assert results["FENCE_ENABLE"].ok is False
    assert "not advertised" in results["FENCE_ENABLE"].message


def test_guid_timeout_zero_fails():
    bad = _good_values()
    bad["GUID_TIMEOUT"] = 0.0
    v = ParamVerifier(_FakeMav(bad), timeout_per_param=0.0)
    results = {c.name: c for c in v.run()}
    assert results["GUID_TIMEOUT"].ok is False


def test_custom_requirements_used():
    """Caller can override the requirement set for component tests."""
    custom = [Requirement("MY_PARAM", "==", 42.0)]
    v = ParamVerifier(_FakeMav({"MY_PARAM": 42.0}), timeout_per_param=0.0,
                      requirements=custom)
    assert v.all_pass() is True


def test_bitmask_rule():
    req = Requirement("FENCE_TYPE", "bit", 0x1)   # require bit 0
    ok, _ = req.evaluate(0x3)
    assert ok
    ok, _ = req.evaluate(0x2)
    assert not ok


def test_default_requirements_lists_known_params():
    names = [r.name for r in default_requirements()]
    for expected in ("FENCE_ENABLE", "RTL_ALT", "BATT_FS_LOW_ACT",
                     "FS_GCS_ENABLE", "GUID_TIMEOUT"):
        assert expected in names
