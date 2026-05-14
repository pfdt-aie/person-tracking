"""RC transmitter link health — preflight gate for takeoff."""
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import pytest

pytest.importorskip("pymavlink")

import config as cfg
from mavlink_client.mavlink_client import MAVLinkClient
from safety.safety import SafetyMonitor
from config.settings import load_settings


def _client():
    settings = load_settings()
    safety = SafetyMonitor(settings=settings)
    return MAVLinkClient(safety=safety, settings=settings)


def test_rc_not_connected_before_first_message():
    c = _client()
    assert c.is_rc_connected() is False


def test_rc_age_is_inf_before_first_message():
    c = _client()
    assert c.get_rc_age_s() == float("inf")


def test_rc_connected_after_recent_message():
    c = _client()
    c._rc_last_t    = time.monotonic()
    c._rc_chancount = 8
    assert c.is_rc_connected() is True


def test_rc_disconnected_when_stale():
    c = _client()
    c._rc_last_t    = time.monotonic() - (cfg.RC_WATCHDOG_S + 1.0)
    c._rc_chancount = 8
    assert c.is_rc_connected() is False


def test_rc_disconnected_when_too_few_channels():
    c = _client()
    c._rc_last_t    = time.monotonic()
    c._rc_chancount = max(0, cfg.RC_MIN_CHANNELS - 1)
    assert c.is_rc_connected() is False


def test_rc_age_reports_real_value():
    c = _client()
    c._rc_last_t = time.monotonic() - 1.0
    age = c.get_rc_age_s()
    assert 0.5 < age < 2.0


def test_rc_rssi_default_zero():
    c = _client()
    assert c.get_rc_rssi() == 0


def test_rc_rssi_reflects_message():
    c = _client()
    c._rc_rssi = 210
    assert c.get_rc_rssi() == 210


def test_watchdog_constant_is_realistic():
    assert 0.5 <= cfg.RC_WATCHDOG_S <= 5.0
    assert cfg.RC_MIN_CHANNELS >= 4
