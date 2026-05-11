"""tests/test_target_detection.py — TargetDetection dataclass."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from tracking.target_detection import TargetDetection


def _det(**kw) -> TargetDetection:
    defaults = dict(cx=100.0, cy=200.0, x1=50.0, y1=150.0,
                    x2=150.0, y2=250.0, conf=0.9, track_id=1)
    defaults.update(kw)
    return TargetDetection(**defaults)


def test_field_access():
    d = _det()
    assert d.cx == 100.0
    assert d.cy == 200.0
    assert d.x1 == 50.0
    assert d.y1 == 150.0
    assert d.x2 == 150.0
    assert d.y2 == 250.0
    assert d.conf == 0.9
    assert d.track_id == 1


def test_bbox_h():
    d = _det(y1=100.0, y2=300.0)
    assert d.bbox_h == 200.0


def test_bbox_w():
    d = _det(x1=10.0, x2=90.0)
    assert d.bbox_w == 80.0


def test_bbox_area():
    d = _det(x1=0.0, y1=0.0, x2=100.0, y2=50.0)
    assert d.bbox_area == 5000.0


def test_equality():
    a = _det(track_id=5)
    b = _det(track_id=5)
    assert a == b


def test_inequality():
    a = _det(track_id=1)
    b = _det(track_id=2)
    assert a != b


def test_always_truthy():
    """A TargetDetection instance is always truthy — guards must use 'is not None'."""
    d = _det(cx=0.0, cy=0.0, conf=0.0, track_id=0)
    assert bool(d)
