"""S3.5 — detection FPS floor."""
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
    c._fps_times = deque(maxlen=cfg.FPS_WINDOW_SIZE)
    c._lock = threading.Lock()
    c._last_detection = 0.0
    c._loiter_issued = False
    c._alert_issued  = False
    c._confirm_count = 0
    c._fps_warned    = False
    return c


def test_fps_zero_when_no_samples():
    c = _controller()
    assert c.effective_fps() == 0.0


def test_fps_zero_with_single_sample():
    c = _controller()
    c.notify_detection(True)
    assert c.effective_fps() == 0.0


def test_fps_estimated_from_window():
    c = _controller()
    base = time.monotonic()
    # Inject samples at 30 Hz directly into the deque (avoid sleep).
    for i in range(10):
        c._fps_times.append(base + i / 30.0)
    fps = c.effective_fps()
    assert abs(fps - 30.0) < 1e-6


def test_low_fps_window():
    c = _controller()
    base = time.monotonic()
    # 5 Hz — far below MIN_TRACKING_FPS=8
    for i in range(10):
        c._fps_times.append(base + i / 5.0)
    assert c.effective_fps() < cfg.MIN_TRACKING_FPS


def test_deque_caps_at_window_size():
    c = _controller()
    base = time.monotonic()
    for i in range(cfg.FPS_WINDOW_SIZE + 5):
        c._fps_times.append(base + i)
    assert len(c._fps_times) == cfg.FPS_WINDOW_SIZE
