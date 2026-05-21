"""Tests for tracker-side drone outer-loop dispatch."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from tracker import PersonGimbalTracker
from tracking.state_machine import State
from tracking.target_detection import TargetDetection
from tracking.tracker_state import TrackerState


class _Ctrl:
    def __init__(self, fresh: bool):
        self._fresh = fresh
        self.gimbal_pan_deg = 12.0
        self.gimbal_tilt_deg = -35.0
        self.speed_calls = []

    def attitude_is_fresh(self):
        return self._fresh

    def set_speed(self, yaw, pitch):
        self.speed_calls.append((yaw, pitch))


class _DroneCtrl:
    def __init__(self, correction=0.0):
        self.notify_calls = []
        self.angle_calls = []
        self.update_calls = []
        self._correction = correction

    def notify_detection(self, detected):
        self.notify_calls.append(detected)

    def set_gimbal_angles(self, pan_rad, tilt_rad):
        self.angle_calls.append((pan_rad, tilt_rad))

    def update(self, **kwargs):
        self.update_calls.append(kwargs)
        return self._correction


def _tracker_for_outer_loop(*, fresh: bool, correction: float = 0.0):
    t = PersonGimbalTracker.__new__(PersonGimbalTracker)
    t._drone_enabled = True
    t._last_drone_cmd = 0.0
    t._drone_cmd_interval = 0.1
    t._ts = TrackerState(
        tracking_enabled=True,
        drone_following=True,
        mode="AUTO",
        state=State.TRACKING,
        telem_yaw_cmd=4,
        telem_pitch_cmd=-2,
    )
    t.ctrl = _Ctrl(fresh=fresh)
    t.drone_ctrl = _DroneCtrl(correction=correction)
    return t


def _target():
    return TargetDetection(
        cx=100.0, cy=120.0,
        x1=80.0, y1=90.0, x2=120.0, y2=150.0,
        conf=0.9, track_id=1,
    )


def test_drone_outer_loop_runs_update_when_attitude_stale():
    target = _target()
    t = _tracker_for_outer_loop(fresh=False)
    t._ts.lock_id = target.track_id   # H4: must be locked for drone to follow

    t._update_drone_outer_loop(1.0, person_detected=True, target_info=target)

    assert t.drone_ctrl.notify_calls == [True]
    assert t.drone_ctrl.angle_calls == []
    assert len(t.drone_ctrl.update_calls) == 1
    call = t.drone_ctrl.update_calls[0]
    assert call["gimbal_pan_deg"] == 12.0
    assert call["gimbal_tilt_deg"] == -35.0
    assert call["target_info"] is None   # stale attitude → None even with lock
    assert call["drone_tracking_enabled"] is True


def test_drone_outer_loop_uses_target_when_attitude_fresh():
    target = _target()
    t = _tracker_for_outer_loop(fresh=True, correction=0.1)
    t._ts.lock_id = target.track_id   # H4: must be locked for drone to follow

    t._update_drone_outer_loop(1.0, person_detected=True, target_info=target)

    assert len(t.drone_ctrl.angle_calls) == 1
    assert t.drone_ctrl.update_calls[0]["target_info"] is target
    assert t.ctrl.speed_calls


def test_drone_outer_loop_treats_grace_target_as_not_detected():
    target = _target()
    target.is_fresh = False
    t = _tracker_for_outer_loop(fresh=True)

    t._update_drone_outer_loop(1.0, person_detected=True, target_info=target)

    assert t.drone_ctrl.notify_calls == [False]
    assert t.drone_ctrl.update_calls[0]["target_info"] is None


# ---------------------------------------------------------------------------
# Regression: _compute_yaw_correction call-site signature
# ---------------------------------------------------------------------------

class _DroneCtrlWithYaw(_DroneCtrl):
    """Stub that records _compute_yaw_correction() calls so we can verify
    the call site passes exactly one positional argument (pan_deg only).
    F13 removed the dead `dt` parameter; the search-state block in
    _update_drone_outer_loop was left calling with two args and would
    crash with TypeError whenever the drone entered a search state
    with pan > GIMBAL_PAN_SOFT_DEG."""

    def __init__(self, correction=0.0):
        super().__init__(correction=correction)
        self.yaw_corr_calls = []

    def _compute_yaw_correction(self, pan_deg):
        self.yaw_corr_calls.append(pan_deg)
        return self._correction


def _tracker_in_search(pan_deg: float, correction: float = 0.1):
    import config as cfg
    t = PersonGimbalTracker.__new__(PersonGimbalTracker)
    t._drone_enabled = True
    t._last_drone_cmd = 0.0
    t._drone_cmd_interval = 0.1
    t._ts = TrackerState(
        tracking_enabled=True,
        drone_following=True,
        mode="AUTO",
        state=State.SEARCHING,
        telem_yaw_cmd=0,
        telem_pitch_cmd=0,
    )
    ctrl = _Ctrl(fresh=True)
    ctrl.gimbal_pan_deg = pan_deg
    t.ctrl = ctrl
    t.drone_ctrl = _DroneCtrlWithYaw(correction=correction)
    return t


def test_search_state_yaw_correction_no_extra_arg():
    """_compute_yaw_correction must be called with a single pan_deg argument.

    If the call site passes a second argument (old dt=0.1), the method raises
    TypeError and the drone crashes mid-search — regression for F13 fix."""
    import config as cfg
    pan = cfg.GIMBAL_PAN_SOFT_DEG + 20.0
    t = _tracker_in_search(pan_deg=pan, correction=0.1)

    # Must not raise TypeError
    t._update_drone_outer_loop(1.0, person_detected=False, target_info=None)

    assert t.drone_ctrl.yaw_corr_calls, "yaw correction should have been called"
    assert t.drone_ctrl.yaw_corr_calls[0] == pan


def test_predicting_state_applies_gimbal_compensation():
    """Gimbal yaw compensation must fire in PREDICTING state, not just TRACKING.

    During brief person loss (PREDICTING), the drone may still be yawing to
    recentre the gimbal.  If compensation is only applied in TRACKING the
    predicted path drifts visually while the drone rotates."""
    t = _tracker_for_outer_loop(fresh=True, correction=0.05)
    t._ts.state = State.PREDICTING

    t._update_drone_outer_loop(1.0, person_detected=False, target_info=None)

    assert t.ctrl.speed_calls, (
        "gimbal compensation must be applied during PREDICTING state"
    )
