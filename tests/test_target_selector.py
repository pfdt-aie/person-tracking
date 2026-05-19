"""Tests for TargetSelector — YOLO result parsing and target selection."""
import pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from tracking.target_selector import TargetSelector
from tracking.tracker_state import TrackerState
from tracking.person_registry import PersonRegistry


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
    import numpy as np
    from tracking.target_detection import TargetDetection

    class _Boxes:
        id = None
        xyxy = None
        conf = None
        def __len__(self): return 2

    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    import torch
    boxes = _Boxes()
    # Two boxes: small one and large one
    boxes.xyxy = torch.tensor([[10, 10, 50, 50], [100, 100, 400, 400]], dtype=torch.float32)
    boxes.conf = torch.tensor([0.9, 0.8])
    boxes.id   = None  # no ByteTrack IDs → fallback to index

    class _Result:
        pass

    r = _Result()
    r.boxes = boxes
    r.orig_img = frame

    sel = TargetSelector(PersonRegistry())
    state = _make_state()
    target = sel.select([r], state)

    assert target is not None
    # Largest bbox wins — area(10,10→50,50)=1600 vs area(100,100→400,400)=90000
    assert target.x1 == 100.0
    assert len(state.detected_ids) >= 1  # registry may merge identical crops


def test_lock_id_returns_locked_person():
    """With lock_id set, selector returns that detection or None."""
    import numpy as np
    import torch

    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    class _Boxes:
        xyxy = torch.tensor([[10, 10, 50, 50]], dtype=torch.float32)
        conf = torch.tensor([0.9])
        id   = torch.tensor([7], dtype=torch.float32)
        def __len__(self): return 1

    class _Result:
        boxes = _Boxes()
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

    # Lock on non-existent ID returns None
    state.lock_id = 9999
    target = sel.select([_Result()], state)
    assert target is None


def test_locked_target_survives_brief_empty_result(monkeypatch):
    """A locked person should not be declared lost for one short detector dropout."""
    import numpy as np
    import torch

    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    class _Boxes:
        xyxy = torch.tensor([[100, 100, 200, 300]], dtype=torch.float32)
        conf = torch.tensor([0.9])
        id = torch.tensor([7], dtype=torch.float32)
        def __len__(self): return 1

    class _Result:
        boxes = _Boxes()
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
    import numpy as np
    import torch

    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    class _Boxes:
        xyxy = torch.tensor([[100, 100, 200, 300]], dtype=torch.float32)
        conf = torch.tensor([0.9])
        id = torch.tensor([7], dtype=torch.float32)
        def __len__(self): return 1

    class _Result:
        boxes = _Boxes()
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
    import numpy as np
    import torch

    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2[105:305, 108:208] = (0, 0, 255)

    class _Boxes1:
        xyxy = torch.tensor([[100, 100, 200, 300]], dtype=torch.float32)
        conf = torch.tensor([0.9])
        id = torch.tensor([7], dtype=torch.float32)
        def __len__(self): return 1

    class _Boxes2:
        xyxy = torch.tensor([[108, 105, 208, 305]], dtype=torch.float32)
        conf = torch.tensor([0.88])
        id = torch.tensor([8], dtype=torch.float32)
        def __len__(self): return 1

    class _Result1:
        boxes = _Boxes1()
        orig_img = frame1

    class _Result2:
        boxes = _Boxes2()
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


def test_locked_target_rejects_far_new_id(monkeypatch):
    import numpy as np
    import torch

    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2[100:300, 470:570] = (0, 0, 255)

    class _Boxes1:
        xyxy = torch.tensor([[100, 100, 200, 300]], dtype=torch.float32)
        conf = torch.tensor([0.9])
        id = torch.tensor([7], dtype=torch.float32)
        def __len__(self): return 1

    class _Boxes2:
        xyxy = torch.tensor([[470, 100, 570, 300]], dtype=torch.float32)
        conf = torch.tensor([0.88])
        id = torch.tensor([8], dtype=torch.float32)
        def __len__(self): return 1

    class _Result1:
        boxes = _Boxes1()
        orig_img = frame1

    class _Result2:
        boxes = _Boxes2()
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
    import numpy as np
    import torch

    now = [100.0]
    monkeypatch.setattr("tracking.target_selector.time.time", lambda: now[0])

    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2[100:300, 220:320] = (0, 0, 255)

    class _Boxes1:
        xyxy = torch.tensor([[100, 100, 200, 300]], dtype=torch.float32)
        conf = torch.tensor([0.9])
        id = torch.tensor([7], dtype=torch.float32)
        def __len__(self): return 1

    class _Boxes2:
        xyxy = torch.tensor([[220, 100, 320, 300]], dtype=torch.float32)
        conf = torch.tensor([0.88])
        id = torch.tensor([8], dtype=torch.float32)
        def __len__(self): return 1

    class _Result1:
        boxes = _Boxes1()
        orig_img = frame1

    class _Result2:
        boxes = _Boxes2()
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
    import numpy as np
    import torch

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[100:300, 100:200] = (0, 0, 255)

    class _Boxes:
        xyxy = torch.tensor([[100, 100, 200, 300]], dtype=torch.float32)
        conf = torch.tensor([0.9])
        id = None
        def __len__(self): return 1

    class _Result:
        boxes = _Boxes()
        orig_img = frame

    sel = TargetSelector(PersonRegistry())
    state = _make_state()

    first = sel.select([_Result()], state)
    second = sel.select([_Result()], state)

    assert first is not None
    assert second is not None
    assert second.track_id == first.track_id


def test_same_frame_similar_people_do_not_collapse_to_one_pid():
    import numpy as np
    import torch

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[100:300, 100:200] = (0, 0, 255)
    frame[100:300, 300:400] = (0, 0, 255)

    class _Boxes:
        xyxy = torch.tensor(
            [[100, 100, 200, 300], [300, 100, 400, 300]],
            dtype=torch.float32,
        )
        conf = torch.tensor([0.9, 0.88])
        id = torch.tensor([7, 8], dtype=torch.float32)
        def __len__(self): return 2

    class _Result:
        boxes = _Boxes()
        orig_img = frame

    sel = TargetSelector(PersonRegistry())
    state = _make_state()

    sel.select([_Result()], state)

    assert len(state.detected_ids) == 2
