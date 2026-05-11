"""
mission/recording_manager.py — Background recording thread extracted from tracker.py.

RecordingManager owns the _recorder_loop logic.  It takes explicit dependencies
so it can be tested and reused without a full tracker instance.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

import config as cfg
from tracking.target_detection import TargetDetection


class RecordingManager:
    """Writes annotated frames to disk at true RTSP FPS in a background thread.

    Args:
        grabber:    FrameGrabber providing frame_ready Condition and raw frames.
        recorder:   VideoRecorder that accepts write() calls.
        draw_fn:    Callable(frame, target_info) that draws HUD overlay in-place.
        get_target: Callable() → Optional[TargetDetection] returning the latest target.
        is_running: Callable() → bool; loop exits when this returns False.
    """

    def __init__(
        self,
        grabber,
        recorder,
        draw_fn:    Callable,
        get_target: Callable[[], Optional[TargetDetection]],
        is_running: Callable[[], bool],
    ) -> None:
        self._grabber    = grabber
        self._recorder   = recorder
        self._draw       = draw_fn
        self._get_target = get_target
        self._running    = is_running

    def loop(self) -> None:
        """Entry point for the recorder background thread."""
        last_id = -1

        while self._running():
            if not self._recorder.is_recording:
                time.sleep(0.005)
                continue

            with self._grabber.frame_ready:
                while self._grabber.frame_id == last_id and self._running():
                    self._grabber.frame_ready.wait(timeout=0.1)
                if self._grabber.frame is None:
                    continue
                last_id   = self._grabber.frame_id
                frame_ref = self._grabber.frame

            frame = frame_ref.copy()

            if cfg.RECORD_OVERLAY:
                self._draw(frame, self._get_target())
            self._recorder.write(frame)
