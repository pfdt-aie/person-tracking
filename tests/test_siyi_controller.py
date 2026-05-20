"""Tests for SIYI controller helpers that do not require hardware."""

import struct
import threading
import time

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
    c._battery_mv = 0
    c._current_ma = 0
    return c


def _attitude_packet(yaw_01deg: int, pitch_01deg: int, roll_01deg: int = 0) -> bytes:
    """Build a minimal valid 0x0D attitude response packet."""
    payload = struct.pack("<hhh", yaw_01deg, pitch_01deg, roll_01deg)
    return (
        b"\x55\x66\x01"
        + struct.pack("<H", len(payload))
        + b"\x00\x00"
        + b"\x0d"
        + payload
        + b"\x00\x00"
    )


def _battery_packet(mv: int, ma: int) -> bytes:
    """Build a minimal valid 0x0A battery response packet."""
    payload = struct.pack("<HH", mv, ma)
    return (
        b"\x55\x66\x01"
        + struct.pack("<H", len(payload))
        + b"\x00\x00"
        + b"\x0a"
        + payload
        + b"\x00\x00"
    )


# ---------------------------------------------------------------------------
# CRC
# ---------------------------------------------------------------------------

def test_crc16_standard_xmodem_vector():
    """CRC-16/XMODEM standard test vector: b'123456789' → 0x31C3."""
    c = _controller_stub()
    assert c._crc16(b"123456789") == 0x31C3


def test_crc16_empty_input():
    c = _controller_stub()
    assert c._crc16(b"") == 0x0000


# ---------------------------------------------------------------------------
# _integrate_commanded_attitude (existing tests, kept for regression)
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Battery packet parsing (0x0A)
# ---------------------------------------------------------------------------

def test_battery_packet_updates_voltage_and_current():
    c = _controller_stub()
    c._parse_packet(_battery_packet(mv=12600, ma=2000))
    assert c._battery_mv == 12600
    assert c._current_ma == 2000


def test_battery_voltage_property_converts_mv_to_v():
    c = _controller_stub()
    c._parse_packet(_battery_packet(mv=16800, ma=1500))
    assert abs(c.battery_voltage - 16.8) < 0.001


def test_power_watts_is_v_times_a():
    c = _controller_stub()
    c._battery_mv = 12000
    c._current_ma = 2000
    expected = (12000 / 1000.0) * (2000 / 1000.0)  # 12V × 2A = 24W
    assert abs(c.power_watts - expected) < 0.001


def test_battery_zero_before_first_packet():
    c = _controller_stub()
    assert c.battery_voltage == 0.0
    assert c.current_ma == 0


# ---------------------------------------------------------------------------
# set_speed — clamping, rate limit, force flag
# ---------------------------------------------------------------------------

def test_set_speed_clamps_yaw_to_100():
    c = _controller_stub()
    c._last_cmd_time = 0.0       # old → time gate open
    # Can't send (no sock), but we verify clamping via _last_yaw
    # Use a stub that captures the clamped value
    sent = []

    def fake_send(cmd_id, data):
        sent.append(struct.unpack("bb", data))

    c._send = fake_send
    c.set_speed(200, -300, force=True)
    assert sent[-1] == (100, -100), f"Expected (100,-100) got {sent[-1]}"


def test_set_speed_rate_limit_suppresses_duplicate():
    c = _controller_stub()
    c._last_cmd_time = time.time()   # command just sent
    c._last_yaw   = 50
    c._last_pitch = 20
    sent = []
    c._send = lambda cmd, data: sent.append(data)

    result = c.set_speed(50, 20, force=False)   # same values, within interval

    assert result is False, "Duplicate within rate-limit window must be suppressed"
    assert sent == []


def test_set_speed_allows_large_change_within_interval():
    """Significant speed change (≥3 units) must bypass the rate limiter."""
    c = _controller_stub()
    c._last_cmd_time = time.time()   # within interval
    c._last_yaw   = 0
    c._last_pitch = 0
    sent = []
    c._send = lambda cmd, data: sent.append(data)

    result = c.set_speed(50, 0, force=False)   # change = 50 >> 3

    assert result is True
    assert len(sent) == 1


def test_set_speed_force_bypasses_rate_limit():
    c = _controller_stub()
    c._last_cmd_time = time.time()   # within interval
    c._last_yaw   = 0
    c._last_pitch = 0
    sent = []
    c._send = lambda cmd, data: sent.append(data)

    result = c.set_speed(0, 0, force=True)

    assert result is True
    assert len(sent) == 1


# ---------------------------------------------------------------------------
# gimbal_pan_deg / gimbal_tilt_deg — fresh vs stale fallback
# ---------------------------------------------------------------------------

def test_gimbal_pan_deg_returns_telemetry_when_fresh():
    c = _controller_stub()
    c._att_time     = time.monotonic()  # just received
    c._att_yaw_deg  = 45.0
    c._cmd_yaw_deg  = 10.0             # different commanded estimate

    assert c.gimbal_pan_deg == 45.0, "Fresh telemetry must override commanded estimate"


def test_gimbal_pan_deg_falls_back_to_commanded_when_stale():
    c = _controller_stub()
    c._att_time    = time.monotonic() - cfg.GIMBAL_ATTITUDE_STALE_S - 1.0
    c._att_yaw_deg = 45.0
    c._cmd_yaw_deg = 10.0

    assert c.gimbal_pan_deg == 10.0, "Stale telemetry must fall back to commanded angle"


def test_gimbal_tilt_deg_returns_telemetry_when_fresh():
    c = _controller_stub()
    c._att_time      = time.monotonic()
    c._att_pitch_deg = -30.0
    c._cmd_pitch_deg = -5.0

    assert c.gimbal_tilt_deg == -30.0


def test_gimbal_pan_deg_uses_commanded_when_no_telemetry_ever_received():
    c = _controller_stub()
    # _att_time = 0.0 (never received), _cmd_yaw_deg = 0.0
    assert c.gimbal_pan_deg == 0.0


# ---------------------------------------------------------------------------
# attitude_is_fresh
# ---------------------------------------------------------------------------

def test_attitude_is_fresh_true_for_recent_telemetry():
    c = _controller_stub()
    c._att_time = time.monotonic()
    assert c.attitude_is_fresh() is True


def test_attitude_is_fresh_false_when_stale():
    c = _controller_stub()
    c._att_time = time.monotonic() - cfg.GIMBAL_COAST_S - 1.0
    assert c.attitude_is_fresh() is False


def test_attitude_is_fresh_false_before_first_packet():
    c = _controller_stub()
    c._att_time = 0.0   # never received
    assert c.attitude_is_fresh() is False


# ---------------------------------------------------------------------------
# is_responding
# ---------------------------------------------------------------------------

def test_is_responding_true_before_first_packet():
    """Startup grace: no packet yet → must report as responding."""
    c = _controller_stub()
    c._last_recv_time = 0.0
    assert c.is_responding is True


def test_is_responding_true_after_recent_packet():
    c = _controller_stub()
    c._last_recv_time = time.monotonic()
    assert c.is_responding is True


def test_is_responding_false_after_watchdog_timeout():
    c = _controller_stub()
    c._last_recv_time = time.monotonic() - cfg.GIMBAL_WATCHDOG_S - 1.0
    assert c.is_responding is False


# ---------------------------------------------------------------------------
# zoom_absolute packet encoding
# ---------------------------------------------------------------------------

def test_zoom_absolute_encodes_integer_and_decimal_parts():
    c = _controller_stub()
    sent = []
    c._send = lambda cmd, data: sent.append((cmd, data))

    c.zoom_absolute(2.5)

    assert len(sent) == 1
    cmd, data = sent[0]
    assert cmd == 0x0F
    int_part, dec_part = struct.unpack("BB", data)
    assert int_part == 2
    assert dec_part == 5


def test_zoom_absolute_clamps_to_zoom_max():
    c = _controller_stub()
    sent = []
    c._send = lambda cmd, data: sent.append(data)

    c.zoom_absolute(cfg.ZOOM_MAX + 10.0)

    int_part, dec_part = struct.unpack("BB", sent[0])
    assert int_part + dec_part / 10.0 <= cfg.ZOOM_MAX + 0.1
