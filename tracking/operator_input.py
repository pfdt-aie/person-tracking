"""
operator_input.py — Keyboard and terminal (stdin) input handler for the tracker.

Extracts _stdin_reader, _handle_manual_key, and _handle_key from
PersonGimbalTracker so those ~100 lines of operator-control logic live in one
focused class instead of in the main integration file.
"""

from __future__ import annotations

import sys
import threading
import time
from typing import Callable, Optional

import cv2

import config as cfg
from tracking.state_machine import State
from tracking.tracker_state import TrackerState


# Window for the E-STOP double-tap escalation in the SSH stdin loop.
# First `estop` press sends BRAKE; a second press within this window
# escalates to LAND. State is per-channel: the browser's E-STOP in
# StreamServer keeps its own copy so SSH and UI can't desync each other.
_ESTOP_DOUBLE_TAP_S: float = 3.0


# Single source of truth for stdin command help text. Also serves the
# `help` command — every verb the SSH operator can type must appear
# here so docs and code can never drift.
_HELP: list[tuple[str, str]] = [
    ("help",                       "show this command list"),
    ("status [json]",              "current telemetry snapshot (json = raw dict)"),
    ("preflight",                  "run the preflight checklist"),
    ("ids",                        "list detected person IDs"),
    ("track <id>",                 "lock onto a numeric person ID"),
    ("track on|off",               "enable/disable autonomous tracking"),
    ("unlock",                     "release current person lock"),
    ("mode",                       "print current tracker mode"),
    ("mode auto|manual",           "set tracker mode"),
    ("mode brake|land|rtl",        "request FCU safety mode (--drone required)"),
    ("arm",                        "arm drone-body tracker after preflight (--drone)"),
    ("disarm",                     "stop following (tracker only; FCU mode unchanged)"),
    ("estop",                      "BRAKE; press again within 3s for LAND"),
    ("pan <-100..100>",            "manual gimbal pan speed (MANUAL mode)"),
    ("tilt <-100..100>",           "manual gimbal tilt speed (MANUAL mode)"),
    ("stop",                       "stop manual gimbal motion"),
    ("zoom in|out",                "0.5 s zoom pulse (also disables auto-zoom)"),
    ("rec [on|off]",               "toggle/start/stop recording (no arg = toggle)"),
    ("stream on|off",              "start/stop live MJPEG stream server"),
    ("search on|off|restart",      "toggle search or restart initial acquisition scan"),
    ("center",                     "center gimbal and reset zoom to 1x"),
    ("autozoom on|off",            "toggle auto-zoom"),
    ("q",                          "quit program"),
]


class OperatorInputController:
    """Handle all keyboard and terminal stdin commands.

    Args:
        state:         Shared TrackerState.
        ctrl:          SIYIController for gimbal stop/zoom/center.
        zoom_ctrl:     AutoZoomController.
        stream:        Optional StreamServer for toggle (may be None).
        recorder:      VideoRecorder for toggle.
        grabber:       FrameGrabber — provides frame dimensions for recorder start.
        enter_manual:  Callable that switches the tracker to MANUAL mode.
        enter_auto:    Callable that switches the tracker to AUTO mode.
        request_land:   Optional callable that asks the FCU to enter LAND mode.
        request_rtl:    Optional callable that asks the FCU to enter RTL mode.
        request_brake:  Optional callable that asks the FCU to enter BRAKE mode.
        reset_all:     Callable that resets all state (from GimbalStateMachine).
        search, init_scan, expand_search, lissajous:
                       Search pattern instances for key 's' / 'i' commands.
    """

    def __init__(
        self,
        state:        TrackerState,
        ctrl,
        zoom_ctrl,
        stream,
        recorder,
        grabber,
        enter_manual: Callable[[], None],
        enter_auto:   Callable[[], None],
        reset_all:    Callable[[], None],
        search,
        init_scan,
        expand_search,
        lissajous,
        request_land:  Optional[Callable[[], dict]] = None,
        request_rtl:   Optional[Callable[[], dict]] = None,
        request_brake: Optional[Callable[[], dict]] = None,
        handle_estop:     Optional[Callable[[str], dict]]       = None,
        handle_arm:       Optional[Callable[[Optional[bool]], dict]] = None,
        handle_preflight: Optional[Callable[[], list]]          = None,
        handle_telemetry: Optional[Callable[[], dict]]          = None,
    ) -> None:
        self._state        = state
        self._ctrl         = ctrl
        self._zoom         = zoom_ctrl
        self._stream       = stream
        self._recorder     = recorder
        self._grabber      = grabber
        self._enter_manual = enter_manual
        self._enter_auto   = enter_auto
        self._request_land = request_land
        self._request_rtl  = request_rtl
        self._request_brake = request_brake
        self._reset_all    = reset_all
        self._search       = search
        self._init_scan    = init_scan
        self._expand       = expand_search
        self._lissajous    = lissajous

        # SSH-parity callbacks (browser-equivalent actions for stdin use).
        self._handle_estop     = handle_estop
        self._handle_arm       = handle_arm
        self._handle_preflight = handle_preflight
        self._handle_telemetry = handle_telemetry

        # E-STOP double-tap state. Independent of StreamServer's copy so
        # the SSH channel can escalate BRAKE -> LAND without depending on
        # whether the browser ever pressed the button.
        self._estop_last_t:      float = 0.0
        self._estop_last_action: str   = ""

    # ------------------------------------------------------------------
    #  Thread lifecycle
    # ------------------------------------------------------------------

    def start(self) -> threading.Thread:
        """Spawn and return the background stdin reader thread."""
        t = threading.Thread(target=self._stdin_loop, daemon=True, name="StdinCmd")
        t.start()
        return t

    # ------------------------------------------------------------------
    #  Stdin loop
    # ------------------------------------------------------------------

    def _stdin_loop(self) -> None:
        st = self._state
        print("[Cmd] Terminal commands: 'track <id>' | 'unlock' | 'ids' | "
              "'mode auto' | 'mode manual' | 'mode brake' | 'mode land' | 'mode rtl' | "
              "'pan <spd>' | 'tilt <spd>' | 'stop' | 'q'")
        while st.running:
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                line = line.strip().lower()
                if not line:
                    continue
                if line.startswith("track "):
                    parts = line.split()
                    if len(parts) == 2 and parts[1].isdigit():
                        st.lock_id = int(parts[1])
                        print(f"[Target] Locked onto ID {st.lock_id}. Type 'unlock' to release.")
                    else:
                        print("[Cmd] Usage: track <number>  (e.g. track 2)")
                elif line == "unlock":
                    st.lock_id = None
                    print("[Target] Lock released — tracking largest person")
                elif line == "ids":
                    snap = dict(st.detected_ids)
                    if snap:
                        for tid, d in snap.items():
                            marker = " ← LOCKED" if tid == st.lock_id else ""
                            print(f"  ID {tid:3d}  center=({d.cx:.0f},{d.cy:.0f})"
                                  f"  conf={d.conf:.2f}{marker}")
                    else:
                        print("[IDs] No persons currently detected")
                elif line.startswith("mode "):
                    parts = line.split()
                    if len(parts) == 2 and parts[1] in ("auto", "manual", "brake", "land", "rtl"):
                        if parts[1] == "manual":
                            self._enter_manual()
                        elif parts[1] == "auto":
                            self._enter_auto()
                        elif parts[1] == "brake":
                            self._handle_safety_mode("BRAKE", self._request_brake)
                        elif parts[1] == "land":
                            self._handle_safety_mode("LAND", self._request_land)
                        else:
                            self._handle_safety_mode("RTL", self._request_rtl)
                    else:
                        print("[Cmd] Usage: mode auto | mode manual | mode brake | mode land | mode rtl")
                elif line.startswith("pan "):
                    if st.mode != "MANUAL":
                        print("[Cmd] Switch to MANUAL mode first ('mode manual')")
                    else:
                        try:
                            spd = max(-100, min(100, int(line.split()[1])))
                            st.manual_yaw_speed = spd
                            st.manual_key_t     = time.time() + 86400.0
                            print(f"[Manual] Pan speed: {spd:+d}  (type 'stop' to halt)")
                        except (IndexError, ValueError):
                            print("[Cmd] Usage: pan <-100..100>")
                elif line.startswith("tilt "):
                    if st.mode != "MANUAL":
                        print("[Cmd] Switch to MANUAL mode first ('mode manual')")
                    else:
                        try:
                            spd = max(-100, min(100, int(line.split()[1])))
                            st.manual_pitch_speed = spd
                            st.manual_key_t       = time.time() + 86400.0
                            print(f"[Manual] Tilt speed: {spd:+d}  (type 'stop' to halt)")
                        except (IndexError, ValueError):
                            print("[Cmd] Usage: tilt <-100..100>")
                elif line == "stop":
                    st.manual_yaw_speed   = 0
                    st.manual_pitch_speed = 0
                    st.manual_key_t       = 0.0
                    self._ctrl.stop()
                    print("[Manual] Gimbal stopped")
                elif line == "q":
                    st.running = False
                    break
            except Exception:
                break

    def _handle_safety_mode(self, label: str, cb: Optional[Callable[[], dict]]) -> None:
        if cb is None:
            print(f"[Cmd] {label} unavailable — tracker launched without MAVLink handler")
            return
        result = cb()
        status = result.get("status", "error")
        msg = result.get("msg", "")
        if status == "ok":
            print(f"[Cmd] {label} requested")
        else:
            print(f"[Cmd] {label} failed: {msg}")

    # ------------------------------------------------------------------
    #  Arrow-key handler (raw cv2.waitKey value)
    # ------------------------------------------------------------------

    def handle_manual_key(self, raw_key: int) -> None:
        """Process arrow keys for gimbal direction in MANUAL mode.

        Sets a 150 ms hold timer — gimbal auto-stops if key is released.
        """
        if raw_key < 0:
            return
        spd = cfg.MANUAL_GIMBAL_SPEED
        now = time.time()
        st  = self._state
        if raw_key in (81, 65361):    # Left
            st.manual_yaw_speed, st.manual_pitch_speed = -spd, 0
            st.manual_key_t = now + 0.15
        elif raw_key in (83, 65363):  # Right
            st.manual_yaw_speed, st.manual_pitch_speed =  spd, 0
            st.manual_key_t = now + 0.15
        elif raw_key in (82, 65362):  # Up
            st.manual_yaw_speed, st.manual_pitch_speed = 0,  spd
            st.manual_key_t = now + 0.15
        elif raw_key in (84, 65364):  # Down
            st.manual_yaw_speed, st.manual_pitch_speed = 0, -spd
            st.manual_key_t = now + 0.15

    # ------------------------------------------------------------------
    #  Full key handler (masked cv2.waitKey & 0xFF value)
    # ------------------------------------------------------------------

    def handle_key(self, key: int) -> None:
        if key == 255:
            return
        st = self._state
        if key == ord("m"):
            if st.mode == "AUTO":
                self._enter_manual()
            else:
                self._enter_auto()
        elif key == ord("t"):
            st.tracking_enabled = not st.tracking_enabled
            if not st.tracking_enabled:
                self._reset_all()
            print(f"[Track] {'ON' if st.tracking_enabled else 'OFF'}")
        elif key == ord("r"):
            self._ctrl.center()
            self._reset_all()
            self._ctrl.zoom_absolute(1.0)
            print("[Reset] Center + Zoom 1×")
        elif key == ord("s"):
            st.search_enabled = not st.search_enabled
            if not st.search_enabled:
                self._search.stop(); self._init_scan.stop()
                self._expand.stop(); self._lissajous.stop()
                if st.state in State.SEARCH_STATES:
                    st.state = State.WAITING
                    self._ctrl.stop()
            print(f"[Search] {'ON' if st.search_enabled else 'OFF'}")
        elif key == ord("i"):
            self._search.stop(); self._expand.stop(); self._lissajous.stop()
            st.state = State.INITIAL_SCAN
            self._init_scan.start()
            print("[Search] Restarting initial acquisition scan")
        elif key == ord("d"):
            # show_display is still on PersonGimbalTracker — handled there
            pass
        elif key == ord("z"):
            self._ctrl.zoom_in()
            st.manual_zoom_stop_at = time.time() + 0.5
        elif key == ord("x"):
            self._ctrl.zoom_out()
            st.manual_zoom_stop_at = time.time() + 0.5
        elif key == ord("a"):
            self._zoom.enabled = not self._zoom.enabled
            if not self._zoom.enabled:
                self._zoom.stop()
            print(f"[AutoZoom] {'ON' if self._zoom.enabled else 'OFF'}")
        elif key == ord("l"):
            if self._stream:
                if self._stream.is_active:
                    self._stream.stop()
                    print("[Stream] Stopped")
                else:
                    self._stream.start()
        elif key in (ord("+"), ord("=")):
            cfg.ADAPT_KP_MIN += 2; cfg.ADAPT_KP_MAX += 2
            print(f"[Adapt] Kp range: {cfg.ADAPT_KP_MIN:.0f}-{cfg.ADAPT_KP_MAX:.0f}")
        elif key == ord("-"):
            cfg.ADAPT_KP_MIN = max(5, cfg.ADAPT_KP_MIN - 2)
            cfg.ADAPT_KP_MAX = max(10, cfg.ADAPT_KP_MAX - 2)
            print(f"[Adapt] Kp range: {cfg.ADAPT_KP_MIN:.0f}-{cfg.ADAPT_KP_MAX:.0f}")
        elif key == ord("]"):
            cfg.ADAPT_SPEED_MAX = min(100, cfg.ADAPT_SPEED_MAX + 5)
            print(f"[Adapt] Speed max: {cfg.ADAPT_SPEED_MAX}")
        elif key == ord("["):
            cfg.ADAPT_SPEED_MAX = max(cfg.ADAPT_SPEED_MIN + 5, cfg.ADAPT_SPEED_MAX - 5)
        elif key == ord("v"):
            if self._recorder.is_recording:
                self._recorder.stop()
            else:
                self._recorder.start(
                    self._grabber.frame_w, self._grabber.frame_h, cfg.RECORD_FPS
                )
            print(f"[Rec] {'ON' if self._recorder.is_recording else 'OFF'}")
