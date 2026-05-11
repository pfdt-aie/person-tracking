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
