"""S3.3 — session-time RTL.

Verifies the controller starts a per-session timer when armed, fires RTL
exactly once on expiry, and resets cleanly when disarmed.
"""
import math
import pathlib
import sys
import threading
import time
from collections import deque

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from config.settings import load_settings
from control.drone_controller import DroneController


class _FakeMav:
    def __init__(self):
        self.rtl_calls = 0
        self.loiter_calls = 0
        self.last_velocity = None
        self.rc_connected = True

    # Health gates always pass so update() reaches the session check.
    def is_connected(self):           return True
    def get_last_heartbeat_time(self): return time.monotonic()
    def get_mode(self):                return "GUIDED"
    def is_armed(self):                return True
    def is_home_set(self):             return True
    def is_sensors_healthy(self):      return True
    def get_battery_voltage(self):     return 16.0
    def is_gps_ok(self):               return True
    def get_gps(self):                 return (51.5, -0.1, 30.0)
    def get_home_position(self):       return (51.5, -0.1, 0.0)
    def is_fence_breached(self):       return False
    def is_rc_override_active(self):   return False
    def is_rc_connected(self):         return self.rc_connected
    def is_ground_test(self):          return False
    # Capture commands
    def send_rtl(self):
        self.rtl_calls += 1
        return True
    def send_loiter(self):
        self.loiter_calls += 1
        return True
    def send_zero_velocity(self):      self.last_velocity = (0, 0, 0)
    def send_velocity_ned(self, *a):   self.last_velocity = a
    def send_position_velocity_ned(self, *a): self.last_velocity = a
    def get_position_ned(self):        return (0.0, 0.0, -30.0)
    def get_velocity_ned(self):        return (0.0, 0.0, 0.0)
    def get_attitude(self):            return (0.0, 0.0, 0.0)


class _FakeEKF:
    is_valid = False
    def predict(self, dt): pass
    def reset(self): pass
    def get_position_ned(self):  return (0.0, 0.0)
    def get_velocity_ned(self):  return (0.0, 0.0)


class _FakeSafety:
    def watchdog_heartbeat(self, t):   return True
    def is_battery_critical(self, v):  return False
    def check_geofence(self, *a):      return True
    def geofence_contains(self, *a):   return True
    def check_altitude(self, a):       return a
    def check_velocity(self, n, e, d): return n, e, d
    def check_home_keepout(self, *a):  return True
    @property
    def is_home_set(self):             return True
    def get_fence_centre(self):        return (51.5, -0.1)


def _controller(mav, safety):
    c = DroneController.__new__(DroneController)
    c._s = load_settings()
    c._mav = mav
    c._safety = safety
    c._ekf = _FakeEKF()
    c._last_update = 0.0
    c._last_detection = time.monotonic()
    c._loiter_issued = False
    c._loiter_t = 0.0
    c._rtl_issued = False
    c._alert_issued = False
    c._rtl_attempts = 0
    c._rtl_last_t = 0.0
    c._rtl_confirmed = False
    c._mode_warn_t = 0.0
    c._sensor_warn_issued = False
    c._target_fence_warn_t = 0.0
    c._origin_lat = 51.5
    c._origin_lon = -0.1
    c._origin_set = True
    c._gimbal_pan_rad = 0.0
    c._gimbal_tilt_rad = -math.pi / 4
    c._lock = threading.Lock()
    c._confirm_count = 0
    c._retreating = False
    c._vel_above_t = -1.0
    c._bearing_rad = 0.0
    c._bearing_init = False
    c._session_start_t = -1.0
    c._session_rtl_issued = False
    c._rc_loss_loiter_issued = False
    c._fps_times = deque(maxlen=cfg.FPS_WINDOW_SIZE)
    c._fps_warned = False
    c._ema_vn = c._ema_ve = 0.0
    c._prev_vn = c._prev_ve = 0.0
    c._prev_an = c._prev_ae = 0.0
    return c


def test_disarmed_keeps_timer_inactive():
    mav = _FakeMav()
    c = _controller(mav, _FakeSafety())
    c.update(0.0, -45.0, None, drone_tracking_enabled=False)
    assert c._session_start_t < 0
    assert mav.rtl_calls == 0


def test_armed_starts_timer():
    """First armed update primes the session start timestamp."""
    mav = _FakeMav()
    c = _controller(mav, _FakeSafety())
    c.update(0.0, -45.0, None, drone_tracking_enabled=True)
    assert c._session_start_t > 0


def test_rtl_fires_after_max_flight_time():
    mav = _FakeMav()
    c = _controller(mav, _FakeSafety())
    c.update(0.0, -45.0, None, drone_tracking_enabled=True)
    # Backdate the session start past the limit
    c._session_start_t -= cfg.MAX_FLIGHT_TIME_S + 1.0
    c.update(0.0, -45.0, None, drone_tracking_enabled=True)
    assert mav.rtl_calls == 1
    assert c._session_rtl_issued is True


def test_rtl_fires_only_once_per_session():
    mav = _FakeMav()
    c = _controller(mav, _FakeSafety())
    c.update(0.0, -45.0, None, drone_tracking_enabled=True)
    c._session_start_t -= cfg.MAX_FLIGHT_TIME_S + 5.0
    for _ in range(5):
        c.update(0.0, -45.0, None, drone_tracking_enabled=True)
    assert mav.rtl_calls == 1


def test_disarm_clears_timer_and_latch():
    mav = _FakeMav()
    c = _controller(mav, _FakeSafety())
    c.update(0.0, -45.0, None, drone_tracking_enabled=True)
    c._session_start_t -= cfg.MAX_FLIGHT_TIME_S + 1.0
    c.update(0.0, -45.0, None, drone_tracking_enabled=True)
    assert c._session_rtl_issued is True
    # Operator disarms — state should reset
    c.update(0.0, -45.0, None, drone_tracking_enabled=False)
    assert c._session_start_t < 0
    assert c._session_rtl_issued is False


def test_runtime_rc_loss_issues_loiter_once():
    mav = _FakeMav()
    mav.rc_connected = False
    c = _controller(mav, _FakeSafety())
    c.update(0.0, -45.0, None, drone_tracking_enabled=True)
    c.update(0.0, -45.0, None, drone_tracking_enabled=True)
    assert mav.loiter_calls == 1
    assert mav.last_velocity == (0, 0, 0)
