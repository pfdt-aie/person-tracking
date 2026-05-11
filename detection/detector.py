"""
detection/detector.py : YOLO TensorRT inference wrapper.

Encapsulates model loading, device selection, and the track/detect
try-except fallback so tracker.py never calls Ultralytics APIs directly.

Person re-identification is handled by PersonRegistry (HSV histogram, CPU)
in tracking/person_registry.py — no GPU ReID backbone is loaded here.
"""

import logging

from ultralytics import YOLO

from config.settings import Settings, load_settings

_log = logging.getLogger(__name__)


def _resolve_device(pref: str) -> str:
    """Resolve "auto" to "0" (CUDA) or "cpu" based on torch availability."""
    if pref != "auto":
        return pref
    try:
        import torch
        return "0" if torch.cuda.is_available() else "cpu"
    except ImportError:
        return "cpu"


class Detector:
    """YOLO TensorRT engine with ByteTrack and graceful fallback.

    Args:
        model_path: Path to TensorRT engine or .pt weights file.
    """

    def __init__(self, model_path: str | None = None, settings: Settings | None = None) -> None:
        self._s = settings or load_settings()
        self.device = _resolve_device(self._s.detect_device)
        print(f"[Model] Loading YOLO TensorRT engine on device={self.device!r}...")
        self.model = YOLO(model_path or self._s.model_path)
        print("[Model] Ready")

    def detect_and_track(self, frame, conf: float, imgsz: int, persist: bool = True):
        """Run inference with ByteTrack, falling back to detect-only on error.

        Args:
            frame:   BGR numpy array from the camera.
            conf:    Confidence threshold.
            imgsz:   Inference image size.
            persist: Keep ByteTrack state across frames.

        Returns:
            Ultralytics Results list (same as model.track / model()).
        """
        try:
            return self.model.track(
                frame,
                classes=0,
                conf=conf,
                imgsz=imgsz,
                verbose=False,
                device=self.device,
                persist=persist,
                tracker="bytetrack.yaml",
            )
        except Exception as exc:
            _log.warning("[Detect] model.track failed (%s), retrying without tracker", exc)
            return self.model(
                frame,
                classes=0,
                conf=conf,
                imgsz=imgsz,
                verbose=False,
                device=self.device,
            )
