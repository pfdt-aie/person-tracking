"""FCU disarm requests must be gated by landed-state telemetry."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import pytest

pymavlink = pytest.importorskip("pymavlink")
from pymavlink import mavutil

from config.settings import load_settings
from mavlink_client.mavlink_client import MAVLinkClient
from safety.safety import SafetyMonitor


def _client() -> MAVLinkClient:
    settings = load_settings()
    return MAVLinkClient(safety=SafetyMonitor(settings=settings), settings=settings)


def test_disarm_refused_when_armed_and_landed_state_unknown():
    c = _client()
    c._armed = True
    c._landed_state = None
    seen = []
    c.send_command_with_ack = lambda *a, **kw: seen.append((a, kw)) or True
    assert c.send_disarm_if_landed() is False
    assert seen == []


def test_disarm_allowed_when_landed_state_on_ground():
    c = _client()
    c._armed = True
    c._landed_state = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    seen = []
    c.send_command_with_ack = lambda *a, **kw: seen.append((a, kw)) or True
    assert c.send_disarm_if_landed() is True
    assert seen[0][0][0] == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    assert seen[0][1]["p1"] == 0
