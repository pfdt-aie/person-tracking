"""
mission/stream_adapter.py — Background stream-push thread extracted from tracker.py.

StreamAdapter owns the _stream_loop logic.  It takes explicit dependencies
so it can be tested and reused without a full tracker instance.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

import cv2

import config as cfg
from gcs.stream_server import StreamServer
from tracking.target_detection import TargetDetection

# Stream output resolutions (must match tracker.py / stream_server.py)
_STREAM_W_HI = 1280
_STREAM_H_HI = 720
_STREAM_W_LO = 854
_STREAM_H_LO = 480


class StreamAdapter:
    """Pushes annotated frames to web clients at STREAM_MAX_FPS in a background thread.

    Grabs raw frames from the grabber's Condition, draws HUD overlay, and
    pushes dual-resolution JPEG to the StreamServer.  Independent of YOLO speed.

    Args:
        grabber:    FrameGrabber providing frame_ready Condition and raw frames.
        stream:     StreamServer receiving push_frame() calls.
        draw_fn:    Callable(frame, target_info) that draws HUD overlay in-place.
        get_target: Callable() → Optional[TargetDetection] returning latest target.
        is_running: Callable() → bool; loop exits when this returns False.
    """

    def __init__(
        self,
        grabber,
        stream:     StreamServer,
        draw_fn:    Callable,
        get_target: Callable[[], Optional[TargetDetection]],
        is_running: Callable[[], bool],
    ) -> None:
        self._grabber    = grabber
        self._stream     = stream
        self._draw       = draw_fn
        self._get_target = get_target
        self._running    = is_running

    def loop(self) -> None:
        """Entry point for the stream-push background thread."""
        last_id    = -1
        _min_dt    = 1.0 / max(1, cfg.STREAM_MAX_FPS)
        _last_push = 0.0

        while self._running():
            if not self._stream or not self._stream.is_active:
                time.sleep(0.005)
                continue

            with self._grabber.frame_ready:
                while self._grabber.frame_id == last_id and self._running():
                    self._grabber.frame_ready.wait(timeout=0.1)
                if self._grabber.frame is None:
                    continue
                last_id   = self._grabber.frame_id
                frame_ref = self._grabber.frame

            now = time.time()
            if now - _last_push < _min_dt:
                continue
            _last_push = now

            frame   = frame_ref.copy()
            disp_hi = cv2.resize(frame, (_STREAM_W_HI, _STREAM_H_HI),
                                 interpolation=cv2.INTER_LINEAR)
            self._draw(disp_hi, self._get_target())
            disp_lo = cv2.resize(disp_hi, (_STREAM_W_LO, _STREAM_H_LO),
                                 interpolation=cv2.INTER_AREA)
            self._stream.push_frame(disp_hi, disp_lo)
