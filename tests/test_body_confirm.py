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
from tracking.target_detection import TargetDetection


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


def test_visual_detection_can_skip_body_confirmation():
    c = _controller()

    c.notify_detection(True, confirm_body=False)

    assert c.is_body_confirmed() is False
    assert len(c._fps_times) == 1


class _Geo:
    def __init__(self, result=(0.0, 0.00001)):
        self.result = result

    def project(self, *args):
        return self.result


class _Mav:
    def get_gps(self):
        return 0.0, 0.0, 7.0

    def get_attitude(self):
        return 0.0, 0.0, 0.0


class _AcceptingEkf:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.updates = []

    def gps_to_ned(self, lat, lon, origin_lat, origin_lon):
        return lat * 1000.0, lon * 1000.0

    def update(self, meas_n, meas_e):
        self.updates.append((meas_n, meas_e))
        return self.accepted


def _target():
    return TargetDetection(
        cx=100.0, cy=120.0,
        x1=80.0, y1=90.0, x2=120.0, y2=150.0,
        conf=0.9, track_id=1,
    )


def test_accepted_ekf_measurement_confirms_body_motion():
    c = _controller()
    c._origin_set = True
    c._origin_lat = 0.0
    c._origin_lon = 0.0
    c._frame_w = 1280
    c._frame_h = 720
    c._gimbal_pan_rad = 0.0
    c._gimbal_tilt_rad = -0.7
    c._mav = _Mav()
    c._geo = _Geo()
    c._ekf = _AcceptingEkf(accepted=True)

    assert c._update_ekf_from_detection(_target(), time.monotonic()) is True

    assert c.is_body_confirmed() is True
    assert c._loiter_issued is False
    assert c._alert_issued is False


def test_rejected_ekf_measurement_does_not_confirm_body_motion():
    c = _controller()
    c._origin_set = True
    c._origin_lat = 0.0
    c._origin_lon = 0.0
    c._frame_w = 1280
    c._frame_h = 720
    c._gimbal_pan_rad = 0.0
    c._gimbal_tilt_rad = -0.7
    c._mav = _Mav()
    c._geo = _Geo()
    c._ekf = _AcceptingEkf(accepted=False)

    assert c._update_ekf_from_detection(_target(), time.monotonic()) is False

    assert c.is_body_confirmed() is False


def test_failed_projection_does_not_confirm_body_motion():
    c = _controller()
    c._origin_set = True
    c._origin_lat = 0.0
    c._origin_lon = 0.0
    c._frame_w = 1280
    c._frame_h = 720
    c._gimbal_pan_rad = 0.0
    c._gimbal_tilt_rad = -0.7
    c._mav = _Mav()
    c._geo = _Geo(result=None)
    c._ekf = _AcceptingEkf(accepted=True)

    assert c._update_ekf_from_detection(_target(), time.monotonic()) is False

    assert c.is_body_confirmed() is False


# ---------------------------------------------------------------------------
# _prediction_window_active — EKF prediction phase bypasses body confirm
# ---------------------------------------------------------------------------

class _FakeEKF:
    def __init__(self, valid: bool) -> None:
        self.is_valid = valid


def _controller_with_ekf(valid: bool) -> "DroneController":
    c = _controller()
    c._ekf = _FakeEKF(valid=valid)
    return c


def test_prediction_window_active_when_ekf_valid_and_within_hover():
    """EKF valid, loss within hover window → prediction active → body confirm exempt."""
    c = _controller_with_ekf(valid=True)
    dt_lost = c._s.tracking_loss_hover_s * 0.4   # 40 % — well within 2 s
    assert c._prediction_window_active(dt_lost) is True


def test_prediction_window_inactive_when_ekf_invalid():
    """EKF not yet initialised → startup phase → body confirm still required."""
    c = _controller_with_ekf(valid=False)
    assert c._prediction_window_active(0.5) is False


def test_prediction_window_inactive_when_hover_expired():
    """dt_lost >= tracking_loss_hover_s → prediction phase over → body confirm applies."""
    c = _controller_with_ekf(valid=True)
    dt_lost = c._s.tracking_loss_hover_s + 0.1   # just past 2 s
    assert c._prediction_window_active(dt_lost) is False


def test_prediction_window_boundary_exclusive():
    """Exactly at hover boundary → not active (tracking_loss_hover_s is exclusive)."""
    c = _controller_with_ekf(valid=True)
    assert c._prediction_window_active(c._s.tracking_loss_hover_s) is False


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
