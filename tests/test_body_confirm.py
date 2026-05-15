"""S1.5 — N-frame detection confirmation before drone body movement."""
import pathlib
import sys
import threading
from collections import deque

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from config.settings import load_settings
from control.drone_controller import DroneController


def _controller() -> DroneController:
    c = DroneController.__new__(DroneController)
    c._s = load_settings()
    c._confirm_count = 0
    c._lock = threading.Lock()
    c._last_detection = 0.0
    c._loiter_issued = False
    c._alert_issued = False
    c._fps_times = deque(maxlen=cfg.FPS_WINDOW_SIZE)
    return c


def test_confirm_starts_unconfirmed():
    c = _controller()
    assert c.is_body_confirmed() is False


def test_confirm_after_n_consecutive_true():
    c = _controller()
    for _ in range(cfg.BODY_MOVE_CONFIRM_FRAMES):
        c.notify_detection(True)
    assert c.is_body_confirmed() is True


def test_confirm_resets_on_false():
    c = _controller()
    for _ in range(cfg.BODY_MOVE_CONFIRM_FRAMES - 1):
        c.notify_detection(True)
    c.notify_detection(False)
    assert c.is_body_confirmed() is False
    # Must restart count from zero
    c.notify_detection(True)
    assert c.is_body_confirmed() is False


def test_confirm_count_does_not_overflow():
    c = _controller()
    for _ in range(cfg.BODY_MOVE_CONFIRM_FRAMES * 5):
        c.notify_detection(True)
    assert c._confirm_count == cfg.BODY_MOVE_CONFIRM_FRAMES
