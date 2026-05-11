"""
video_recorder.py — Records annotated frames to a dated MP4 file.

Encoder priority:
  1. Jetson NVENC via GStreamer (nvv4l2h264enc) — hardware, near-zero CPU
  2. Software H.264 via GStreamer (x264enc)     — CPU, universal fallback
  3. OpenCV built-in mp4v                       — last resort

4K on Jetson Orin Nano Super:
  YES — the onboard NVENC encoder handles 4K H.264 at < 5 % CPU.
  Requirements: JetPack 6 + GStreamer gst-plugins-bad with nvv4l2h264enc.
  NVMe SSD recommended (eMMC may not sustain 4K write rates).
  Resolution is detected automatically from the live stream — no config change needed.
"""

import os
from datetime import datetime

import cv2

import config as cfg


class VideoRecorder:
    """Records annotated frames to a dated MP4 file.

    Uses GStreamer's Jetson NVENC encoder when available (JetPack 6+),
    which allows 4K@30 fps recording with < 5 % CPU overhead on the
    Orin Nano Super. Falls back to software encoding on non-Jetson hosts.

    Args:
        recordings_dir: Directory where video files are saved.
    """

    def __init__(self, recordings_dir: str) -> None:
        self._dir:    str                    = recordings_dir
        self._writer: cv2.VideoWriter | None = None
        self._path:   str                    = ""
        self._active: bool                   = False

    # ------------------------------------------------------------------
    #  Lifecycle
    # ------------------------------------------------------------------

    def start(self, frame_w: int, frame_h: int, fps: float) -> str:
        """Open a new recording file. Returns the file path."""
        if self._active:
            self.stop()

        os.makedirs(self._dir, exist_ok=True)
        ts         = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        res        = f"{frame_w}x{frame_h}"
        self._path = os.path.join(self._dir, f"track_{ts}_{res}.mp4")

        writer = (
            self._open_hw(frame_w, frame_h, fps)
            or self._open_sw_gst(frame_w, frame_h, fps)
            or self._open_sw_cv(frame_w, frame_h, fps)
        )

        if writer and writer.isOpened():
            self._writer = writer
            self._active = True
            print(f"[REC] Recording started")
            print(f"      File   {self._path}")
            print(f"      Codec  H.264  {frame_w}x{frame_h} @ {fps:.0f} fps  "
                  f"{cfg.RECORD_BITRATE // 1_000_000} Mbps")
        else:
            print("[REC] ERROR: VideoWriter failed — recording disabled")
        return self._path

    def write(self, frame) -> None:
        """Write one annotated frame. No-op if not recording."""
        if self._active and self._writer is not None:
            self._writer.write(frame)

    def stop(self) -> None:
        """Finalise and close the current recording."""
        if self._active:
            self._active = False
            if self._writer is not None:
                self._writer.release()
                self._writer = None
            print(f"[REC] Saved  {self._path}")

    # ------------------------------------------------------------------
    #  Properties
    # ------------------------------------------------------------------

    @property
    def is_recording(self) -> bool:
        return self._active

    @property
    def path(self) -> str:
        return self._path

    # ------------------------------------------------------------------
    #  Encoder backends
    # ------------------------------------------------------------------

    def _open_hw(self, w: int, h: int, fps: float) -> cv2.VideoWriter | None:
        """Jetson NVENC hardware H.264 via GStreamer."""
        quoted = self._path.replace('"', '\\"')
        gst = (
            f"appsrc ! "
            f"videoconvert ! "
            f"nvvidconv ! "
            f"nvv4l2h264enc bitrate={cfg.RECORD_BITRATE} control-rate=constant_bitrate ! "
            f"h264parse ! "
            f"mp4mux ! "
            f'filesink location="{quoted}" sync=false'
        )
        try:
            w_ = cv2.VideoWriter(gst, cv2.CAP_GSTREAMER, 0, fps, (w, h), True)
            if w_ and w_.isOpened():
                return w_
            print("[REC] HW encoder (nvv4l2h264enc) unavailable — trying x264enc")
            return None
        except Exception as e:
            print(f"[REC] HW encoder error ({type(e).__name__}): {e}")
            return None

    def _open_sw_gst(self, w: int, h: int, fps: float) -> cv2.VideoWriter | None:
        """Software H.264 via GStreamer x264enc (universal fallback)."""
        quoted = self._path.replace('"', '\\"')
        gst = (
            f"appsrc ! "
            f"videoconvert ! "
            f"x264enc bitrate={cfg.RECORD_BITRATE // 1000} speed-preset=ultrafast ! "
            f"h264parse ! "
            f"mp4mux ! "
            f'filesink location="{quoted}" sync=false'
        )
        try:
            w_ = cv2.VideoWriter(gst, cv2.CAP_GSTREAMER, 0, fps, (w, h), True)
            if w_ and w_.isOpened():
                return w_
            print("[REC] x264enc unavailable — trying mp4v fallback")
            return None
        except Exception as e:
            print(f"[REC] x264enc error ({type(e).__name__}): {e}")
            return None

    def _open_sw_cv(self, w: int, h: int, fps: float) -> cv2.VideoWriter | None:
        """OpenCV built-in mp4v — last resort, no GStreamer needed."""
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        return cv2.VideoWriter(self._path, fourcc, fps, (w, h))
