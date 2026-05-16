"""
frame_grabber.py — Zero-latency threaded RTSP frame capture for SIYI A8 mini.

Pipeline priority:
  1. GStreamer + Jetson NVDEC (nvv4l2decoder) — hardware H.264 decode,
     rtspsrc latency=0, appsink drop=true  → typically < 30 ms glass-to-RAM
  2. GStreamer + software avdec_h264         — same low-latency pipeline, SW decode
  3. OpenCV / FFmpeg                         — last resort, higher latency

The shared frame slot uses a threading.Condition so stream/recorder threads
block (zero CPU) until the grabber signals a new frame, then wake immediately
— no polling, no sleep().
"""

import threading
import time

import cv2

import config as cfg
from config.settings import Settings, load_settings


class FrameGrabber:
    """Continuous RTSP grab loop with auto-reconnect and Condition-based signaling.

    The frame_ready Condition is shared with the stream and recorder threads so
    they can block cheaply until a new frame arrives.

    Args:
        rtsp_url: Full RTSP URL of the camera stream.
    """

    def __init__(self, rtsp_url: str | None = None, settings: Settings | None = None) -> None:
        _s = settings or load_settings()
        self.rtsp_url   = rtsp_url or _s.rtsp_url
        self.cap        = None
        self.frame      = None
        self.frame_id   = 0
        self.last_read_id = -1
        # Condition IS-A Lock: consumers wait on it, grabber notifies on every frame.
        self.frame_ready = threading.Condition()
        self.running    = False
        self.thread     = None
        self.frame_w    = 0
        self.frame_h    = 0
        self.grab_fps   = 0.0
        self.stream_fps = 25.0
        self._connected  = False
        self._hw_decode  = False   # True when NVDEC pipeline is active

    # ------------------------------------------------------------------
    #  Stream open — tries GStreamer HW → GStreamer SW → FFmpeg
    # ------------------------------------------------------------------

    def _gst_hw_pipeline(self) -> str:
        """Jetson NVDEC hardware H.264 decode, zero-latency RTSP."""
        return (
            f"rtspsrc location={self.rtsp_url} latency=0 drop-on-latency=true "
            f"buffer-mode=none ! "
            f"rtph264depay ! h264parse ! "
            f"nvv4l2decoder enable-max-performance=1 ! "
            f"nvvidconv ! video/x-raw,format=BGRx ! "
            f"videoconvert ! video/x-raw,format=BGR ! "
            f"appsink max-buffers=1 drop=true sync=false name=sink"
        )

    def _gst_sw_pipeline(self) -> str:
        """Software H.264 decode, zero-latency RTSP (non-Jetson fallback)."""
        return (
            f"rtspsrc location={self.rtsp_url} latency=0 drop-on-latency=true "
            f"buffer-mode=none ! "
            f"rtph264depay ! h264parse ! "
            f"avdec_h264 max-threads=2 ! "
            f"videoconvert ! video/x-raw,format=BGR ! "
            f"appsink max-buffers=1 drop=true sync=false name=sink"
        )

    def _open_stream(self) -> cv2.VideoCapture:
        # 1. Try Jetson hardware pipeline
        try:
            cap = cv2.VideoCapture(self._gst_hw_pipeline(), cv2.CAP_GSTREAMER)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    print("[GRAB] GStreamer HW (nvv4l2decoder) pipeline OK")
                    self._hw_decode = True
                    return cap
            cap.release()
        except Exception as e:
            import logging as _log
            _log.warning("[GRAB] GStreamer HW pipeline failed: %s", e)

        # 2. GStreamer software pipeline
        try:
            cap = cv2.VideoCapture(self._gst_sw_pipeline(), cv2.CAP_GSTREAMER)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    print("[GRAB] GStreamer SW (avdec_h264) pipeline OK")
                    self._hw_decode = False
                    return cap
            cap.release()
        except Exception as e:
            import logging as _log
            _log.warning("[GRAB] GStreamer SW pipeline failed: %s", e)

        # 3. FFmpeg fallback — higher latency but universal
        print("[GRAB] WARN: GStreamer unavailable — using FFmpeg (higher latency)")
        print("[GRAB]       Fix: sudo apt install -y gstreamer1.0-plugins-good "
              "gstreamer1.0-plugins-bad gstreamer1.0-libav python3-gst-1.0")
        print("[GRAB]       Then verify: python3 -c \"import gi; gi.require_version('Gst','1.0'); "
              "from gi.repository import Gst; print(Gst.version())\"")
        cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000)
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000)
        self._hw_decode = False
        return cap

    # ------------------------------------------------------------------
    #  Stream lifecycle
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Open stream and start background grab thread."""
        self.cap = self._open_stream()
        if not self.cap.isOpened():
            print("[GRAB] ERROR: Cannot open RTSP stream — check camera IP/network")
            return False
        reported = self.cap.get(cv2.CAP_PROP_FPS)
        self.stream_fps = reported if 1.0 < reported <= 60.0 else 25.0
        ret, frame = self.cap.read()
        if ret:
            self.frame_h, self.frame_w = frame.shape[:2]
            with self.frame_ready:
                self.frame    = frame
                self.frame_id = 1
                self.frame_ready.notify_all()
            decode_tag = "HW-NVDEC" if self._hw_decode else "SW"
            print(f"[GRAB] {self.frame_w}x{self.frame_h}  @ {self.stream_fps:.0f} fps  [{decode_tag}]")
        else:
            self.frame_w, self.frame_h = 1280, 720
        self._connected = True
        self.running    = True
        self.thread = threading.Thread(target=self._loop, daemon=True, name="GrabThread")
        self.thread.start()
        return True

    def stop(self) -> None:
        """Stop grab loop and release camera resources."""
        self.running = False
        with self.frame_ready:
            self.frame_ready.notify_all()
        if self.thread:
            self.thread.join(timeout=3.0)
        if self.cap:
            self.cap.release()

    # ------------------------------------------------------------------
    #  Background loop
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        """Continuous grab loop; reconnects automatically on stream loss."""
        fc: int = 0
        ft: float = time.time()
        reconnect_count: int = 0

        while self.running:
            if not self._connected:
                reconnect_count += 1
                time.sleep(5.0 if reconnect_count > 20 else 1.0)
                try:
                    if self.cap:
                        self.cap.release()
                    self.cap = self._open_stream()
                    if self.cap.isOpened():
                        ret, test = self.cap.read()
                        if ret and test is not None:
                            h, w = test.shape[:2]
                            if w > 0 and h > 0:
                                self._connected    = True
                                self.frame_h, self.frame_w = h, w
                                reconnect_count    = 0
                                print(f"[GRAB] Reconnected  {self.frame_w}x{self.frame_h}")
                            else:
                                print(f"[GRAB] WARN: Zero-size frame ({w}x{h}) — retrying")
                except Exception as e:
                    print(f"[GRAB] ERROR: Reconnect  {type(e).__name__}: {e}")
                continue

            ret, frame = self.cap.read()
            if not ret:
                self._connected = False
                continue

            # Signal all waiting consumers immediately — no polling needed.
            with self.frame_ready:
                self.frame     = frame
                self.frame_id += 1
                self.frame_ready.notify_all()

            fc += 1
            if fc >= 30:
                el = time.time() - ft
                self.grab_fps = fc / el if el > 0 else 0.0
                fc, ft = 0, time.time()

    # ------------------------------------------------------------------
    #  Frame access
    # ------------------------------------------------------------------

    def get_frame(self) -> tuple:
        """Return the latest frame and whether it is new since last call."""
        with self.frame_ready:
            if self.frame is None:
                return None, False
            is_new = self.frame_id != self.last_read_id
            self.last_read_id = self.frame_id
            return self.frame.copy(), is_new

    def is_connected(self) -> bool:
        return self._connected
