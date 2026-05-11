"""Regression tests for TargetDetection-backed web click handling via WebControlAdapter."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gcs.web_control_adapter import WebControlAdapter
from tracking.tracker_state import TrackerState
from tracking.target_detection import TargetDetection


class _Grabber:
    frame_w = 100
    frame_h = 100


class _NullCtrl:
    def stop(self): pass
    def zoom_in(self): pass
    def zoom_out(self): pass


class _NullZoom:
    enabled = True


def _adapter_with_detection() -> tuple[WebControlAdapter, TrackerState]:
    state = TrackerState()
    state.detected_ids = {
        7: TargetDetection(
            cx=50, cy=50,
            x1=40, y1=40, x2=60, y2=60,
            conf=0.91, track_id=7,
        )
    }
    adapter = WebControlAdapter(
        state=state,
        ctrl=_NullCtrl(),
        zoom_ctrl=_NullZoom(),
        grabber=_Grabber(),
        enter_manual=lambda: None,
        enter_auto=lambda: None,
    )
    return adapter, state


def test_web_status_handles_target_detection_values():
    adapter, _ = _adapter_with_detection()
    assert adapter.handle_click(None, None) == {'lock_id': None, 'ids': [7]}


def test_web_click_locks_target_detection_value():
    adapter, state = _adapter_with_detection()
    result = adapter.handle_click(0.5, 0.5)
    assert result == {'status': 'locked', 'id': 7, 'lock_id': 7}
    assert state.lock_id == 7


def test_web_click_miss_does_not_crash():
    adapter, state = _adapter_with_detection()
    result = adapter.handle_click(0.1, 0.1)
    assert result['status'] == 'miss'
    assert state.lock_id is None


def test_web_click_unlock_sentinel():
    adapter, state = _adapter_with_detection()
    state.lock_id = 7
    result = adapter.handle_click(-1.0, -1.0)
    assert result == {'status': 'unlocked', 'lock_id': None}
    assert state.lock_id is None


def test_web_gimbal_requires_manual_mode():
    adapter, state = _adapter_with_detection()
    assert state.mode == 'AUTO'
    result = adapter.handle_gimbal('left')
    assert result['status'] == 'error'


def test_web_gimbal_in_manual_mode():
    adapter, state = _adapter_with_detection()
    state.mode = 'MANUAL'
    result = adapter.handle_gimbal('right')
    assert result == {'status': 'ok', 'direction': 'right'}
    assert state.manual_yaw_speed > 0


def test_web_mode_query():
    adapter, state = _adapter_with_detection()
    assert adapter.handle_mode() == {'mode': 'AUTO'}


def test_web_mode_switch_calls_enter_manual():
    called = []
    adapter = WebControlAdapter(
        state=TrackerState(),
        ctrl=_NullCtrl(),
        zoom_ctrl=_NullZoom(),
        grabber=_Grabber(),
        enter_manual=lambda: called.append('manual'),
        enter_auto=lambda: called.append('auto'),
    )
    adapter.handle_mode('manual')
    assert 'manual' in called
