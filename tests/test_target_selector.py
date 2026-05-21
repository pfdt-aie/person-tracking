"""Tests for TargetSelector — YOLO result parsing and target selection."""
import pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import numpy as np

from tracking.target_selector import TargetSelector
from tracking.tracker_state import TrackerState
from tracking.person_registry import PersonRegistry


class _T:
    """Minimal tensor-like mock: wraps a numpy array and exposes
    .cpu()/.numpy()/.item() so TargetSelector can call the same
    interface it uses on real PyTorch tensors."""

    def __init__(self, data):
        self._d = np.asarray(data, dtype=np.float32)

    def __getitem__(self, i):
        return _T(self._d[i])

    def __len__(self):
        return len(self._d)

    def cpu(self):
        return self

    def numpy(self):
        return self._d

    def item(self):
        return float(self._d.flat[0])


def _make_state(**kw):
    return TrackerState(**kw)


def test_empty_results_clears_detected_ids():
    sel = TargetSelector(PersonRegistry())
    state = _make_state()
    result = sel.select([], state)
    assert result is None
    assert state.detected_ids == {}


def test_empty_boxes_clears_detected_ids():
    class _Boxes:
        id = None
        def __len__(self): return 0
    class _R:
        boxes = _Boxes()
    sel = TargetSelector(PersonRegistry())
    state = _make_state()
    result = sel.select([_R()], state)
    assert result is None
    assert state.detected_ids == {}


def test_lock_id_none_returns_largest():
    """Without a lock, selector returns the detection with the largest bbox."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    class _Boxes:
        xyxy = _T([[10, 10, 50, 50], [100, 100, 400, 400]])
        conf = _T([0.9, 0.8])
        id   = None
        def __len__(self): return 2

    class _Result:
        boxes    = _Boxes()
        orig_img = frame

    sel = TargetSelector(PersonRegistry())
    state = _make_state()
    target = sel.select([_Result()], state)

    assert target is not None
    # Largest bbox wins — area(10,10→50,50)=1600 vs area(100,100→400,400)=90000
    assert target.x1 == 100.0
    assert len(state.detected_ids) >= 1


def test_unlocked_preview_keeps_same_person_even_if_rival_gets_larger(monkeypatch):
    """Before explicit lock, preview should not jump between visible people."""
    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame1[100:300, 100:200] = (0, 0, 255)
    frame1[120:240, 300:360] = (255, 0, 0)
    frame2 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2[100:300, 100:200] = (0, 0, 255)
    frame2[80:360, 260:520] = (255, 0, 0)

    class _Boxes1:
        xyxy = _T([[100, 100, 200, 300], [300, 120, 360, 240]])
        conf = _T([0.9, 0.88])
        id   = _T([7, 8])
        def __len__(self): return 2

    class _Boxes2:
        xyxy = _T([[100, 100, 200, 300], [260, 80, 520, 360]])
        conf = _T([0.9, 0.88])
        id   = _T([7, 8])
        def __len__(self): return 2

    class _Result1:
        boxes = _Boxes1()
        orig_img = frame1

    class _Result2:
        boxes = _Boxes2()
        orig_img = frame2

    sel = TargetSelector(PersonRegistry())
    state = _make_state()

    first = sel.select([_Result1()], state)
    now[0] += 0.1
    second = sel.select([_Result2()], state)

    assert first is not None
    assert second is not None
    assert second.track_id == first.track_id
    assert second.x1 == first.x1


def test_unlocked_preview_holds_brief_occlusion_instead_of_switching(monkeypatch):
    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame1[100:300, 100:200] = (0, 0, 255)
    frame1[100:300, 300:400] = (255, 0, 0)
    frame2 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2[100:300, 300:400] = (255, 0, 0)

    class _Boxes1:
        xyxy = _T([[100, 100, 200, 300], [300, 100, 400, 300]])
        conf = _T([0.9, 0.88])
        id   = _T([7, 8])
        def __len__(self): return 2

    class _Boxes2:
        xyxy = _T([[300, 100, 400, 300]])
        conf = _T([0.88])
        id   = _T([8])
        def __len__(self): return 1

    class _Result1:
        boxes = _Boxes1()
        orig_img = frame1

    class _Result2:
        boxes = _Boxes2()
        orig_img = frame2

    sel = TargetSelector(PersonRegistry(), lock_grace_s=0.5)
    state = _make_state()

    first = sel.select([_Result1()], state)
    now[0] += 0.2
    held = sel.select([_Result2()], state)
    now[0] += 0.5
    switched = sel.select([_Result2()], state)

    assert first is not None
    assert held is None
    assert switched is not None
    assert switched.track_id != first.track_id


def test_lock_id_returns_locked_person():
    """With lock_id set, selector returns that detection or None."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    class _Boxes:
        xyxy = _T([[10, 10, 50, 50]])
        conf = _T([0.9])
        id   = _T([7])
        def __len__(self): return 1

    class _Result:
        boxes    = _Boxes()
        orig_img = frame

    sel = TargetSelector(PersonRegistry())
    state = _make_state()
    # First call to populate registry
    sel.select([_Result()], state)

    pid = list(state.detected_ids.keys())[0]

    # Lock onto that ID
    state.lock_id = pid
    target = sel.select([_Result()], state)
    assert target is not None
    assert target.track_id == pid

    # Lock on non-existent ID returns None (and grace hasn't started yet for 9999)
    state.lock_id = 9999
    target = sel.select([_Result()], state)
    assert target is None


def test_locked_target_survives_brief_empty_result(monkeypatch):
    """A locked person should not be declared lost for one short detector dropout."""
    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    class _Boxes:
        xyxy = _T([[100, 100, 200, 300]])
        conf = _T([0.9])
        id   = _T([7])
        def __len__(self): return 1

    class _Result:
        boxes    = _Boxes()
        orig_img = frame

    sel = TargetSelector(PersonRegistry(), lock_grace_s=0.5)
    state = _make_state()
    sel.select([_Result()], state)
    pid = list(state.detected_ids.keys())[0]
    state.lock_id = pid
    locked = sel.select([_Result()], state)

    now[0] += 0.2
    stale = sel.select([], state)

    assert stale is not None
    assert stale.track_id == locked.track_id
    assert stale.is_fresh is False


def test_locked_target_grace_expires(monkeypatch):
    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    class _Boxes:
        xyxy = _T([[100, 100, 200, 300]])
        conf = _T([0.9])
        id   = _T([7])
        def __len__(self): return 1

    class _Result:
        boxes    = _Boxes()
        orig_img = frame

    sel = TargetSelector(PersonRegistry(), lock_grace_s=0.5)
    state = _make_state()
    sel.select([_Result()], state)
    pid = list(state.detected_ids.keys())[0]
    state.lock_id = pid
    sel.select([_Result()], state)

    now[0] += 0.8

    assert sel.select([], state) is None


def test_locked_target_can_reacquire_nearby_new_id(monkeypatch):
    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame1[100:300, 100:200] = (0, 0, 255)
    frame2 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2[105:305, 108:208] = (0, 0, 255)

    class _Boxes1:
        xyxy = _T([[100, 100, 200, 300]])
        conf = _T([0.9])
        id   = _T([7])
        def __len__(self): return 1

    class _Boxes2:
        xyxy = _T([[108, 105, 208, 305]])
        conf = _T([0.88])
        id   = _T([8])
        def __len__(self): return 1

    class _Result1:
        boxes    = _Boxes1()
        orig_img = frame1

    class _Result2:
        boxes    = _Boxes2()
        orig_img = frame2

    sel = TargetSelector(PersonRegistry(), lock_grace_s=0.5, reacquire_center_ratio=0.25)
    state = _make_state()
    sel.select([_Result1()], state)
    first_pid = list(state.detected_ids.keys())[0]
    state.lock_id = first_pid
    sel.select([_Result1()], state)

    target = sel.select([_Result2()], state)

    assert target is not None
    assert state.lock_id == target.track_id
    assert target.cx == 158.0
    assert state.target_status == "locked_visible"


def test_locked_target_rejects_nearby_different_appearance(monkeypatch):
    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame1[100:300, 100:200] = (0, 0, 255)
    frame2 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2[105:305, 108:208] = (255, 0, 0)

    class _Boxes1:
        xyxy = _T([[100, 100, 200, 300]])
        conf = _T([0.9])
        id   = _T([7])
        def __len__(self): return 1

    class _Boxes2:
        xyxy = _T([[108, 105, 208, 305]])
        conf = _T([0.88])
        id   = _T([8])
        def __len__(self): return 1

    class _Result1:
        boxes    = _Boxes1()
        orig_img = frame1

    class _Result2:
        boxes    = _Boxes2()
        orig_img = frame2

    sel = TargetSelector(PersonRegistry(), lock_grace_s=0.5, reacquire_center_ratio=0.25)
    state = _make_state()
    sel.select([_Result1()], state)
    first_pid = list(state.detected_ids.keys())[0]
    state.lock_id = first_pid
    locked = sel.select([_Result1()], state)

    target = sel.select([_Result2()], state)

    assert target is not None
    assert target.track_id == locked.track_id
    assert target.is_fresh is False
    assert state.lock_id == first_pid
    assert state.target_status == "locked_grace"


def test_locked_target_holds_when_two_candidates_are_ambiguous(monkeypatch):
    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame1[100:300, 100:200] = (0, 0, 255)
    frame2 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2[102:302, 98:198] = (0, 0, 255)
    frame2[103:303, 112:212] = (0, 0, 255)

    class _Boxes1:
        xyxy = _T([[100, 100, 200, 300]])
        conf = _T([0.9])
        id   = _T([7])
        def __len__(self): return 1

    class _Boxes2:
        xyxy = _T([[98, 102, 198, 302], [112, 103, 212, 303]])
        conf = _T([0.88, 0.87])
        id   = _T([8, 9])
        def __len__(self): return 2

    class _Result1:
        boxes    = _Boxes1()
        orig_img = frame1

    class _Result2:
        boxes    = _Boxes2()
        orig_img = frame2

    sel = TargetSelector(
        PersonRegistry(),
        lock_grace_s=0.5,
        reacquire_center_ratio=0.25,
        ambiguity_margin=0.2,
    )
    state = _make_state()
    sel.select([_Result1()], state)
    first_pid = list(state.detected_ids.keys())[0]
    state.lock_id = first_pid
    locked = sel.select([_Result1()], state)

    target = sel.select([_Result2()], state)

    assert target is not None
    assert target.track_id == locked.track_id
    assert target.is_fresh is False
    assert state.lock_id == first_pid
    assert state.target_status == "locked_grace"


def test_visible_locked_id_rejected_after_implausible_jump(monkeypatch):
    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame1[100:300, 100:200] = (0, 0, 255)
    frame2 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2[100:300, 500:600] = (0, 0, 255)

    class _Boxes1:
        xyxy = _T([[100, 100, 200, 300]])
        conf = _T([0.9])
        id   = _T([7])
        def __len__(self): return 1

    class _Boxes2:
        xyxy = _T([[500, 100, 600, 300]])
        conf = _T([0.88])
        id   = _T([7])
        def __len__(self): return 1

    class _Result1:
        boxes    = _Boxes1()
        orig_img = frame1

    class _Result2:
        boxes    = _Boxes2()
        orig_img = frame2

    sel = TargetSelector(PersonRegistry(), lock_grace_s=0.5)
    state = _make_state()
    sel.select([_Result1()], state)
    first_pid = list(state.detected_ids.keys())[0]
    state.lock_id = first_pid
    locked = sel.select([_Result1()], state)

    target = sel.select([_Result2()], state)

    assert target is not None
    assert target.track_id == locked.track_id
    assert target.is_fresh is False
    assert state.lock_id == first_pid


def test_locked_target_rejects_far_new_id(monkeypatch):
    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2[100:300, 470:570] = (0, 0, 255)

    class _Boxes1:
        xyxy = _T([[100, 100, 200, 300]])
        conf = _T([0.9])
        id   = _T([7])
        def __len__(self): return 1

    class _Boxes2:
        xyxy = _T([[470, 100, 570, 300]])
        conf = _T([0.88])
        id   = _T([8])
        def __len__(self): return 1

    class _Result1:
        boxes    = _Boxes1()
        orig_img = frame1

    class _Result2:
        boxes    = _Boxes2()
        orig_img = frame2

    sel = TargetSelector(PersonRegistry(), lock_grace_s=0.5, reacquire_center_ratio=0.25)
    state = _make_state()
    sel.select([_Result1()], state)
    first_pid = list(state.detected_ids.keys())[0]
    state.lock_id = first_pid
    locked = sel.select([_Result1()], state)

    target = sel.select([_Result2()], state)

    assert target is not None
    assert target.track_id == locked.track_id
    assert target.is_fresh is False
    assert state.lock_id == first_pid


def test_locked_target_rejects_no_overlap_outside_strict_center(monkeypatch):
    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2[100:300, 220:320] = (0, 0, 255)

    class _Boxes1:
        xyxy = _T([[100, 100, 200, 300]])
        conf = _T([0.9])
        id   = _T([7])
        def __len__(self): return 1

    class _Boxes2:
        xyxy = _T([[220, 100, 320, 300]])
        conf = _T([0.88])
        id   = _T([8])
        def __len__(self): return 1

    class _Result1:
        boxes    = _Boxes1()
        orig_img = frame1

    class _Result2:
        boxes    = _Boxes2()
        orig_img = frame2

    sel = TargetSelector(
        PersonRegistry(),
        lock_grace_s=0.5,
        reacquire_center_ratio=0.25,
        strict_center_ratio=0.05,
        min_reacquire_iou=0.05,
    )
    state = _make_state()
    sel.select([_Result1()], state)
    first_pid = list(state.detected_ids.keys())[0]
    state.lock_id = first_pid
    locked = sel.select([_Result1()], state)

    target = sel.select([_Result2()], state)

    assert target is not None
    assert target.track_id == locked.track_id
    assert target.is_fresh is False
    assert state.lock_id == first_pid


def test_detect_only_fallback_reidentifies_by_appearance_not_index():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[100:300, 100:200] = (0, 0, 255)

    class _Boxes:
        xyxy = _T([[100, 100, 200, 300]])
        conf = _T([0.9])
        id   = None
        def __len__(self): return 1

    class _Result:
        boxes    = _Boxes()
        orig_img = frame

    sel = TargetSelector(PersonRegistry())
    state = _make_state()

    first  = sel.select([_Result()], state)
    second = sel.select([_Result()], state)

    assert first is not None
    assert second is not None
    assert second.track_id == first.track_id


def test_same_frame_similar_people_do_not_collapse_to_one_pid():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[100:300, 100:200] = (0, 0, 255)
    frame[100:300, 300:400] = (0, 0, 255)

    class _Boxes:
        xyxy = _T([[100, 100, 200, 300], [300, 100, 400, 300]])
        conf = _T([0.9, 0.88])
        id   = _T([7, 8])
        def __len__(self): return 2

    class _Result:
        boxes    = _Boxes()
        orig_img = frame

    sel = TargetSelector(PersonRegistry())
    state = _make_state()

    sel.select([_Result()], state)

    assert len(state.detected_ids) == 2
