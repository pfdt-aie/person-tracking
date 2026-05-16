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


class _BatchingFakeMav:
    """Tracks prefetch + fetch calls so we can assert the verifier calls
    prefetch_params once with the full list, then per-name fetch_param
    after, hitting the cache."""
    def __init__(self, values: dict):
        self._values = dict(values)
        self.prefetch_calls: list[list[str]] = []
        self.fetch_calls:    list[str] = []

    def prefetch_params(self, names: list, timeout_s: float = 5.0):
        self.prefetch_calls.append(list(names))
        return {n: self._values.get(n) for n in names}

    def fetch_param(self, name, timeout=2.0):
        self.fetch_calls.append(name)
        return self._values.get(name)


def test_verifier_calls_prefetch_once_with_all_param_names():
    """The batch prefetch must include every requirement name and fire
    exactly once per run() — sequential fetches would re-introduce the
    cold-FCU timeout problem the prefetch is solving."""
    mav = _BatchingFakeMav(_good_values())
    v = ParamVerifier(mav, timeout_per_param=0.05)
    v.run()
    assert len(mav.prefetch_calls) == 1
    assert set(mav.prefetch_calls[0]) == set(_good_values().keys())


def test_verifier_falls_back_to_fetch_param_when_prefetch_missing():
    """A mav stub without prefetch_params (e.g. unit-test fakes) must
    still work — the verifier should silently skip the batch step and
    proceed to per-name fetch."""
    class _LegacyMav:
        def fetch_param(self, name, timeout=2.0):
            return _good_values().get(name)
    v = ParamVerifier(_LegacyMav(), timeout_per_param=0.0)
    assert v.all_pass() is True


def test_verifier_tolerates_prefetch_exception():
    """If prefetch raises (e.g. transport glitch), per-name fetch_param
    must still run so the report isn't lost."""
    class _BrokenPrefetchMav(_BatchingFakeMav):
        def prefetch_params(self, names, timeout_s=5.0):
            raise RuntimeError("link hiccup")
    mav = _BrokenPrefetchMav(_good_values())
    v = ParamVerifier(mav, timeout_per_param=0.0)
    results = v.run()
    # We still got per-param results from the fallback path.
    assert len(results) == len(_good_values())
    assert all(c.ok for c in results)
