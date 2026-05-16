"""HEARTBEAT component filtering.

Modern ArduPilot FCUs (e.g. CubeOrangePlus) emit heartbeats from multiple
components on the same link — the autopilot itself, the IO MCU, sometimes
gimbal drivers / cameras. Only the autopilot's heartbeat carries valid
mode and armed state.

Pre-fix (field log gimbal_track_2026-05-16_18-59-34): the rx loop accepted
heartbeats from all of them, producing a phantom "STABILIZE → Mode(0x4) →
STABILIZE" flap at ~1 Hz on a bench with no RC, because the IOMCU's
heartbeats had a different base_mode and confused mode_string_v10().
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import pytest

pytest.importorskip("pymavlink")

from pymavlink import mavutil
from unittest.mock import MagicMock

from mavlink_client.mavlink_client import MAVLinkClient
from safety.safety import SafetyMonitor


def _client_with_mock_mav(initial_compid: int = 0):
    """Return a MAVLinkClient with _mav stubbed; mimic the state right after
    wait_heartbeat() picked some `initial_compid`."""
    safety = SafetyMonitor()
    client = MAVLinkClient(safety=safety, device="/dev/null", baud=9600)
    mock_conn = MagicMock()
    mock_conn.target_system    = 1
    mock_conn.target_component = initial_compid
    client._mav = mock_conn
    return client


def _hb(autopilot_field: int, custom_mode: int = 0, src_comp: int = 1,
        base_mode_extra: int = 0):
    """Fabricate a minimal HEARTBEAT-like object."""
    msg = MagicMock()
    msg.autopilot = autopilot_field
    msg.base_mode = (
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        | base_mode_extra
    )
    msg.custom_mode = custom_mode
    msg.type = mavutil.mavlink.MAV_TYPE_QUADROTOR
    msg.get_srcComponent.return_value = src_comp
    msg.get_type.return_value = "HEARTBEAT"
    return msg


def test_heartbeat_from_iomcu_is_dropped():
    """An IO-MCU style heartbeat (autopilot=INVALID) must NOT update
    _mode, _armed, or _hb_time. Pre-fix this produced phantom flapping."""
    c = _client_with_mock_mav(initial_compid=0)
    c._mode = "STABILIZE"
    c._armed = False
    c._hb_time = 0.0

    junk = _hb(
        autopilot_field=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        custom_mode=4,                        # would be GUIDED if accepted
        src_comp=0,
        base_mode_extra=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
    )
    c._process_heartbeat(junk)

    assert c._mode == "STABILIZE", "mode must not change on peripheral HB"
    assert c._armed is False,     "armed must not change on peripheral HB"
    assert c._hb_time == 0.0,     "_hb_time must not advance on peripheral HB"


def test_heartbeat_from_autopilot_updates_state():
    """A real ArduPilot autopilot heartbeat (autopilot=ARDUPILOTMEGA)
    updates mode, armed, and the heartbeat timestamp."""
    c = _client_with_mock_mav(initial_compid=1)
    c._mode = "STABILIZE"
    c._armed = False

    hb = _hb(
        autopilot_field=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        custom_mode=4,    # GUIDED
        src_comp=1,
        base_mode_extra=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
    )
    c._process_heartbeat(hb)

    assert c._mode == "GUIDED"
    assert c._armed is True
    assert c._hb_time > 0.0


def test_target_component_adopts_autopilot_when_initial_was_wrong(capsys):
    """If wait_heartbeat() captured a peripheral compid (e.g. compid=0,
    the IO MCU), the first real autopilot heartbeat must re-point our
    target to the autopilot so subsequent SET_MODE / arm / takeoff TX
    actually reach it."""
    c = _client_with_mock_mav(initial_compid=0)        # peripheral
    autopilot_hb = _hb(
        autopilot_field=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        custom_mode=4,
        src_comp=1,                                    # autopilot
    )
    c._process_heartbeat(autopilot_hb)

    assert c._mav.target_component == 1
    out = capsys.readouterr().out
    assert "Adopting autopilot compid=1" in out


def test_mixed_stream_does_not_flap():
    """Realistic scenario: peripheral HBs and autopilot HBs interleave on
    the link. Pre-fix, this triggered the mode-flap collapser. Post-fix,
    only the autopilot HBs reach the mode-transition path."""
    c = _client_with_mock_mav(initial_compid=1)
    c._mode = "GUIDED"

    # Alternate 20 heartbeats: peripheral / autopilot / peripheral / autopilot ...
    for i in range(20):
        if i % 2 == 0:
            hb = _hb(
                autopilot_field=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                custom_mode=999, src_comp=0,    # garbage values
            )
        else:
            hb = _hb(
                autopilot_field=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                custom_mode=4,                  # GUIDED — no transition
                src_comp=1,
            )
        c._process_heartbeat(hb)

    # No mode flapping should have been triggered, because autopilot HBs
    # all carried the same mode and peripheral HBs were dropped.
    assert c._mode_flapping is False
    assert c._mode == "GUIDED"
