"""Tests for SIYI controller helpers that do not require hardware."""

import struct
import threading

import config as cfg
from gimbal.siyi_controller import SIYIController


def _controller_stub():
    c = SIYIController.__new__(SIYIController)
    c.lock = threading.Lock()
    c._last_cmd_time = 0.0
    c._last_yaw = 0
    c._last_pitch = 0
    c._last_recv_time = 0.0
    c._att_yaw_deg = 0.0
    c._att_pitch_deg = 0.0
    c._att_roll_deg = 0.0
    c._att_time = 0.0
    c._cmd_yaw_deg = 0.0
    c._cmd_pitch_deg = 0.0
    c._last_angle_est_time = 0.0
    return c


def test_commanded_attitude_integrates_previous_speed_with_dt_cap():
    c = _controller_stub()
    c._last_yaw = 100
    c._last_pitch = -100
    c._last_angle_est_time = 10.0

    c._integrate_commanded_attitude(10.5)

    expected = cfg.GIMBAL_SPEED_FULL_SCALE_DEG_S * cfg.GIMBAL_CMD_EST_MAX_DT_S
    assert c._cmd_yaw_deg == expected
    assert c._cmd_pitch_deg == -expected


def test_commanded_attitude_clamps_to_mechanical_limits():
    c = _controller_stub()
    c._cmd_pitch_deg = cfg.GIMBAL_TILT_MIN_DEG + 1.0
    c._last_pitch = -100
    c._last_angle_est_time = 10.0

    c._integrate_commanded_attitude(10.5)

    assert c._cmd_pitch_deg == cfg.GIMBAL_TILT_MIN_DEG


def test_attitude_packet_seeds_commanded_fallback():
    c = _controller_stub()
    payload = struct.pack("<hhh", 123, -450, 0)
    packet = (
        b"\x55\x66\x01"
        + struct.pack("<H", len(payload))
        + b"\x00\x00"
        + b"\x0d"
        + payload
        + b"\x00\x00"
    )

    c._parse_packet(packet)

    assert c._cmd_yaw_deg == 12.3
    assert c._cmd_pitch_deg == -45.0
