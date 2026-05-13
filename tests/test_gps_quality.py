"""S2.1 — GPS quality gate (fix type + HDOP + sats + EKF variance)."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import pytest

pytest.importorskip("pymavlink")

import config as cfg
from mavlink_client.mavlink_client import MAVLinkClient
from safety.safety import SafetyMonitor
from config.settings import load_settings


def _client(**state) -> MAVLinkClient:
    settings = load_settings()
    safety = SafetyMonitor(settings=settings)
    c = MAVLinkClient(safety=safety, settings=settings)
    # Bypass connect() — directly seed the telemetry cache.
    c._gps_fix       = state.get("fix",  3)
    c._gps_hdop      = state.get("hdop", 0.8)
    c._n_sats        = state.get("sats", 14)
    c._ekf_pos_var   = state.get("ekf_var",  0.2)
    c._ekf_status_seen = state.get("ekf_seen", True)
    return c


def test_all_good_passes():
    assert _client().is_gps_ok() is True


def test_bad_fix_type_fails():
    assert _client(fix=2).is_gps_ok() is False


def test_high_hdop_fails():
    assert _client(hdop=cfg.GPS_MAX_HDOP + 0.5).is_gps_ok() is False


def test_too_few_sats_fails():
    assert _client(sats=cfg.GPS_MIN_SATS - 1).is_gps_ok() is False


def test_high_ekf_variance_fails():
    assert _client(ekf_var=cfg.EKF_MAX_VARIANCE + 0.5).is_gps_ok() is False


def test_ekf_variance_ignored_when_unreported():
    # ekf_status_seen=False, variance can be huge — gate should not enforce it.
    assert _client(ekf_var=9.0, ekf_seen=False).is_gps_ok() is True


def test_boundary_hdop_exactly_at_threshold_passes():
    assert _client(hdop=cfg.GPS_MAX_HDOP).is_gps_ok() is True


def test_boundary_sats_exactly_at_threshold_passes():
    assert _client(sats=cfg.GPS_MIN_SATS).is_gps_ok() is True
