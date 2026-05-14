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
        print("[Cmd] Stdin command loop ready — type 'help' for the full list. "
              "Quick: track <id> | unlock | ids | status | preflight | "
              "mode auto|manual | arm | disarm | estop | q")
        while st.running:
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                line = line.strip().lower()
                if not line:
                    continue

                # --- Info / discovery ---
                if line == "help":
                    self._print_help()
                elif line == "status":
                    self._print_status()
                elif line == "status json":
                    self._print_status(json_mode=True)
                elif line == "preflight":
                    self._print_preflight()

                # --- Tracker / FCU mode ---
                elif line == "mode":
                    print(f"[Cmd] mode={st.mode}")
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
                        print("[Cmd] Usage: mode | mode auto|manual|brake|land|rtl")

                # --- Drone-body arm + software E-STOP ---
                elif line == "arm":
                    self._handle_stdin_arm(True)
                elif line == "disarm":
                    self._handle_stdin_arm(False)
                elif line == "estop":
                    self._handle_stdin_estop()

                # --- Target / lock ---
                elif line.startswith("track "):
                    parts = line.split()
                    if len(parts) == 2 and parts[1].isdigit():
                        st.lock_id = int(parts[1])
                        print(f"[Target] Locked onto ID {st.lock_id}. Type 'unlock' to release.")
                    elif len(parts) == 2 and parts[1] in ("on", "off"):
                        self._toggle_tracking(state=(parts[1] == "on"))
                    else:
                        print("[Cmd] Usage: track <number>  |  track on|off")
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

                # --- Gimbal pulses + center ---
                elif line.startswith("zoom "):
                    parts = line.split()
                    if len(parts) == 2 and parts[1] in ("in", "out"):
                        self._zoom_pulse(parts[1])
                    else:
                        print("[Cmd] Usage: zoom in|out")
                elif line == "center":
                    self._center_and_reset()

                # --- Recording / streaming ---
                elif line == "rec":
                    self._toggle_recording()
                elif line == "rec on":
                    self._toggle_recording(state=True)
                elif line == "rec off":
                    self._toggle_recording(state=False)
                elif line == "stream on":
                    self._toggle_stream(state=True)
                elif line == "stream off":
                    self._toggle_stream(state=False)

                # --- Search / autozoom ---
                elif line == "search on":
                    self._toggle_search(state=True)
                elif line == "search off":
                    self._toggle_search(state=False)
                elif line == "search restart":
                    self._restart_initial_scan()
                elif line == "autozoom on":
                    self._toggle_autozoom(state=True)
                elif line == "autozoom off":
                    self._toggle_autozoom(state=False)

                # --- Manual gimbal speed ---
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

                # --- Quit ---
                elif line == "q":
                    st.running = False
                    break

                # --- Unknown ---
                else:
                    self._print_unknown(line)
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

    def _handle_stdin_arm(self, on: bool) -> None:
        """Stdin wrapper around the handle_arm callback.

        Mirrors what the web UI's Arm/Stop-Follow buttons do but prints
        an operator-friendly summary instead of returning JSON. The
        underlying handler emits its own flight-log event ('arm' /
        'disarm'); we do not log here to avoid double-entries.
        """
        verb = "arm" if on else "disarm"
        cb   = self._handle_arm
        if cb is None:
            print(f"[Cmd] {verb} unavailable — handler not wired "
                  "(launched without --drone?)")
            return
        try:
            result = cb(on) or {}
        except Exception as exc:
            print(f"[Cmd] {verb} error: {exc}")
            return
        status = result.get("status", "error")
        msg    = result.get("msg", "")
        armed  = result.get("armed", False)
        if status == "ok":
            label = "ARMED" if armed else "DISARMED"
            print(f"[Cmd] {label}" + (f" — {msg}" if msg else ""))
        else:
            print(f"[Cmd] {verb} refused: {msg or 'unknown error'}")

    def _handle_stdin_estop(self) -> None:
        """Software E-STOP for the SSH operator.

        First press → handle_estop('brake').  A second press within
        _ESTOP_DOUBLE_TAP_S escalates to handle_estop('land').
        State is per-channel: this counter is independent of the
        browser's copy inside StreamServer, so the SSH operator can
        always escalate without depending on what the UI did.
        """
        cb = self._handle_estop
        if cb is None:
            print("[Cmd] estop unavailable — handler not wired")
            return
        now = time.monotonic()
        if ((now - self._estop_last_t) < _ESTOP_DOUBLE_TAP_S
                and self._estop_last_action == "brake"):
            action = "land"
        else:
            action = "brake"
        self._estop_last_t      = now
        self._estop_last_action = action
        try:
            result = cb(action) or {}
        except Exception as exc:
            print(f"[Cmd] estop error: {exc}")
            return
        status = result.get("status", "error")
        msg    = result.get("msg", "")
        actual = str(result.get("action", action)).upper()
        if status == "ok":
            print(f"[Cmd] E-STOP -> {actual}")
        else:
            print(f"[Cmd] E-STOP -> {actual} FAILED: {msg or 'unknown error'}")

    # ------------------------------------------------------------------
    #  Stdin-only printers (SSH parity surface)
    #
    #  Render structured output for the operator's terminal. They never
    #  mutate tracker state — callers above (the stdin loop) decide
    #  when to invoke them. All three rely on optional callbacks the
    #  tracker may not have wired (gimbal-only mode), so each refuses
    #  gracefully when its callback is None.
    # ------------------------------------------------------------------

    def _print_help(self) -> None:
        """Render the _HELP table to stdout, column-aligned."""
        width = max(len(usage) for usage, _ in _HELP)
        print("[Cmd] Available commands:")
        for usage, doc in _HELP:
            print(f"  {usage.ljust(width)}  {doc}")

    def _print_status(self, json_mode: bool = False) -> None:
        """Print the live telemetry snapshot.

        json_mode=True dumps the raw dict on a single line so the
        operator can pipe `status json` through jq or save a log line.
        """
        cb = self._handle_telemetry
        if cb is None:
            print("[Cmd] status unavailable — telemetry callback not wired")
            return
        try:
            data = cb() or {}
        except Exception as exc:
            print(f"[Cmd] status error: {exc}")
            return
        if json_mode:
            import json as _json
            print(f"[Cmd] {_json.dumps(data, default=str)}")
            return

        def fmt(v, suffix: str = "") -> str:
            return "—" if v is None else f"{v}{suffix}"

        def yn(v) -> str:
            return "yes" if v else "no"

        pp        = data.get("person_protection") or {}
        rc_rssi   = data.get("rc_rssi", 0)
        rssi_str  = f"·{rc_rssi}" if rc_rssi else ""
        dry_run   = " DRY-RUN" if data.get("ground_test") else ""

        print("[Cmd] status:")
        print(f"  tracker={data.get('mode_tracker', '—')}  "
              f"drone={'on' if data.get('drone_enabled') else 'off'}  "
              f"armed=tracker:{yn(data.get('drone_armed'))} "
              f"fcu:{yn(data.get('armed_fcu'))}")
        print(f"  mavlink={'up' if data.get('mavlink') else 'down'}  "
              f"mode_fcu={fmt(data.get('mode_fcu'))}  "
              f"rc={'on' if data.get('rc_connected') else 'off'}{rssi_str}")
        print(f"  gps=fix{fmt(data.get('gps_fix'))} "
              f"sats{fmt(data.get('gps_sats'))} "
              f"hdop{fmt(data.get('gps_hdop'))}  "
              f"ekf_var={fmt(data.get('ekf_var'))}")
        print(f"  battery={fmt(data.get('battery_v'), 'V')}  "
              f"fence={'breach' if data.get('fence_breach') else 'ok'}  "
              f"home={'set' if data.get('home_set') else 'unset'}  "
              f"landed={yn(data.get('landed_fcu'))}")
        print(f"  tracking_loss={fmt(data.get('tracking_loss_s'), 's')}  "
              f"fps={fmt(data.get('fps'))}  "
              f"body_confirmed={yn(data.get('body_confirmed'))}")
        print(f"  person_sep={fmt(pp.get('person_sep_m'), 'm')}  "
              f"vert_clr={fmt(pp.get('vertical_clearance_m'), 'm')}  "
              f"retreating={yn(pp.get('retreating'))}"
              f"{dry_run}")

    def _print_preflight(self) -> None:
        """Print each preflight check on its own line plus a summary."""
        cb = self._handle_preflight
        if cb is None:
            print("[Cmd] preflight unavailable — preflight callback not wired")
            return
        try:
            items = cb() or []
        except Exception as exc:
            print(f"[Cmd] preflight error: {exc}")
            return
        if not items:
            print("[Cmd] preflight: (no checks reported)")
            return
        print("[Cmd] preflight:")
        passed = 0
        for it in items:
            ok   = bool(it.get("ok"))
            tag  = "[OK]  " if ok else "[FAIL]"
            name = it.get("name", "?")
            msg  = it.get("message", "")
            passed += int(ok)
            line = f"  {tag} {name}"
            if msg:
                line += f"  — {msg}"
            print(line)
        print(f"  passed={passed}/{len(items)}")

    def _print_unknown(self, line: str) -> None:
        """Print an 'unknown command' hint and log it to the flight log."""
        print(f"[Cmd] unknown: {line!r} — type 'help' for the list")
        try:
            from utils.flight_log import get_flight_log
            get_flight_log().event("ssh_unknown_command", line=line)
        except Exception:
            pass

    # ------------------------------------------------------------------
    #  Shared action helpers
    #
    #  Each method below replaces a single-key body in handle_key() and
    #  is also called from the stdin loop (Phase 4). Keep them
    #  side-effect-equivalent to the previous inline bodies so keyboard
    #  behavior is unchanged; new behavior (e.g. _zoom_pulse disabling
    #  auto-zoom) is called out in the docstring.
    # ------------------------------------------------------------------

    def _toggle_tracking(self, state: Optional[bool] = None) -> None:
        """Toggle (state=None) or set autonomous person tracking on/off."""
        st = self._state
        target = (not st.tracking_enabled) if state is None else bool(state)
        if target == st.tracking_enabled:
            print(f"[Track] {'ON' if target else 'OFF'} (unchanged)")
            return
        st.tracking_enabled = target
        if not target:
            self._reset_all()
        print(f"[Track] {'ON' if target else 'OFF'}")

    def _toggle_search(self, state: Optional[bool] = None) -> None:
        """Toggle (state=None) or set the search subsystem on/off.

        When turning off, every running search pattern is stopped and
        the FSM is parked in WAITING if it was in a search state.
        """
        st = self._state
        target = (not st.search_enabled) if state is None else bool(state)
        st.search_enabled = target
        if not target:
            self._search.stop(); self._init_scan.stop()
            self._expand.stop(); self._lissajous.stop()
            if st.state in State.SEARCH_STATES:
                st.state = State.WAITING
                self._ctrl.stop()
        print(f"[Search] {'ON' if target else 'OFF'}")

    def _restart_initial_scan(self) -> None:
        """Cancel any in-flight search and start the initial acquisition scan."""
        self._search.stop(); self._expand.stop(); self._lissajous.stop()
        self._state.state = State.INITIAL_SCAN
        self._init_scan.start()
        print("[Search] Restarting initial acquisition scan")

    def _toggle_recording(self, state: Optional[bool] = None) -> None:
        """Toggle (state=None) or set the video recorder on/off."""
        rec = self._recorder
        target = (not rec.is_recording) if state is None else bool(state)
        if target == rec.is_recording:
            print(f"[Rec] {'ON' if target else 'OFF'} (unchanged)")
            return
        if target:
            rec.start(self._grabber.frame_w, self._grabber.frame_h, cfg.RECORD_FPS)
        else:
            rec.stop()
        print(f"[Rec] {'ON' if rec.is_recording else 'OFF'}")

    def _toggle_stream(self, state: Optional[bool] = None) -> None:
        """Toggle (state=None) or set the live MJPEG stream on/off.

        Refuses gracefully when the stream was disabled at launch
        (`STREAM_ENABLED=False` → `self._stream is None`).
        """
        if self._stream is None:
            print("[Stream] disabled at launch (STREAM_ENABLED=False)")
            return
        active = self._stream.is_active
        target = (not active) if state is None else bool(state)
        if target == active:
            print(f"[Stream] {'ON' if target else 'OFF'} (unchanged)")
            return
        if target:
            self._stream.start()   # StreamServer.start() prints its own banner
        else:
            self._stream.stop()
            print("[Stream] Stopped")

    def _toggle_autozoom(self, state: Optional[bool] = None) -> None:
        """Toggle (state=None) or set the auto-zoom controller on/off."""
        target = (not self._zoom.enabled) if state is None else bool(state)
        self._zoom.enabled = target
        if not target:
            self._zoom.stop()
        print(f"[AutoZoom] {'ON' if target else 'OFF'}")

    def _zoom_pulse(self, direction: str) -> None:
        """Send a 0.5 s manual zoom pulse and disable auto-zoom.

        Matches the web UI behaviour (web_control_adapter.handle_zoom)
        so SSH and browser zoom commands have identical side effects.
        Direction must be 'in' or 'out'; anything else is ignored.
        """
        if direction not in ("in", "out"):
            print(f"[Zoom] unknown direction: {direction!r} (expected in|out)")
            return
        if direction == "in":
            self._ctrl.zoom_in()
        else:
            self._ctrl.zoom_out()
        self._zoom.enabled = False
        self._state.manual_zoom_stop_at = time.time() + 0.5
        print(f"[Zoom] {direction.upper()}")

    def _center_and_reset(self) -> None:
        """Center the gimbal, reset tracker FSM state, snap zoom to 1x."""
        self._ctrl.center()
        self._reset_all()
        self._ctrl.zoom_absolute(1.0)
        print("[Reset] Center + Zoom 1x")

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
            self._toggle_tracking()
        elif key == ord("r"):
            self._center_and_reset()
        elif key == ord("s"):
            self._toggle_search()
        elif key == ord("i"):
            self._restart_initial_scan()
        elif key == ord("d"):
            # show_display is still on PersonGimbalTracker — handled there
            pass
        elif key == ord("z"):
            self._zoom_pulse("in")
        elif key == ord("x"):
            self._zoom_pulse("out")
        elif key == ord("a"):
            self._toggle_autozoom()
        elif key == ord("l"):
            self._toggle_stream()
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
            self._toggle_recording()
