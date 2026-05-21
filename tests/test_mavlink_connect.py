"""MAVLink connection lifecycle edge cases."""

import pathlib
import sys
from unittest.mock import MagicMock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import mavlink_client.mavlink_client as mavlink_mod
from mavlink_client.mavlink_client import MAVLinkClient
from safety.safety import SafetyMonitor


def test_connect_fails_closed_when_wait_heartbeat_times_out(monkeypatch):
    fake_conn = MagicMock()
    fake_conn.wait_heartbeat.return_value = None
    fake_mavutil = MagicMock()
    fake_mavutil.mavlink_connection.return_value = fake_conn

    monkeypatch.setattr(mavlink_mod, "_PYMAVLINK_AVAILABLE", True)
    monkeypatch.setattr(mavlink_mod, "mavutil", fake_mavutil, raising=False)

    client = MAVLinkClient(safety=SafetyMonitor(), device="/dev/null", baud=9600)

    assert client.connect() is False
    assert client._mav is None
    assert client._running is False
    fake_conn.close.assert_called_once()
