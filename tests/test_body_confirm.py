"""S1.5 — time-based body-movement confirmation before drone body movement."""
import pathlib
import sys
import threading
import time
from collections import deque

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from config.settings import load_settings
from control.drone_controller import DroneController


def _controller() -> DroneController:
    c = DroneController.__new__(DroneController)
    c._s = load_settings()
    c._lock = threading.Lock()
    c._last_detection = 0.0   # 0.0 → age is huge → not confirmed
    c._loiter_issued = False
    c._alert_issued = False
    c._fps_times = deque(maxlen=cfg.FPS_WINDOW_SIZE)
    return c


def test_confirm_starts_unconfirmed():
    c = _controller()
    # _last_detection=0.0 means age = huge >> window → not confirmed
    assert c.is_body_confirmed() is False


def test_confirmed_after_detection():
    c = _controller()
    c.notify_detection(True)
    assert c.is_body_confirmed() is True


def test_not_confirmed_if_stale():
    c = _controller()
    c.notify_detection(True)
    # Backdate _last_detection so it's older than window
    with c._lock:
        c._last_detection = time.monotonic() - c._s.body_confirm_window_s - 0.1
    assert c.is_body_confirmed() is False


def test_confirmed_stays_true_within_window():
    c = _controller()
    c.notify_detection(True)
    # Still within window
    assert c.is_body_confirmed() is True


def test_false_detection_does_not_break_confirm():
    c = _controller()
    c.notify_detection(True)
    # Several misses — confirm stays True while within window
    for _ in range(6):
        c.notify_detection(False)
    assert c.is_body_confirmed() is True


def test_visual_reacquire_does_not_full_reset_follow_state():
    class _Mav:
        def get_velocity_ned(self):
            return 0.4, -0.2, 0.0

    class _Ekf:
        def __init__(self):
            self.reset_count = 0
        def reset(self):
            self.reset_count += 1

    c = _controller()
    c._mav = _Mav()
    c._ekf = _Ekf()
    c._last_detection = time.monotonic()   # confirmed
    c._session_start_t = 123.0
    c._retreating = True
    c._bearing_init = True

    c.on_target_reacquired()

    # EKF not reset (on_target_reacquired is lightweight)
    assert c._session_start_t == 123.0
    assert c._retreating is True
    assert c._bearing_init is True
    assert c._ekf.reset_count == 0
    assert c._ema_vn == 0.4
    assert c._ema_ve == -0.2
