<!-- ARCHIVED — superseded by DRONE_PROJECT_TECHNICAL_AUDIT_REPORT_2026-05-08.md -->
# Technical Implementation Plan
# Person Tracking Drone — Audit Remediation

> Historical note: this plan was based on the 2026-05-07 audit. The current
> status is tracked in `DRONE_PROJECT_TECHNICAL_AUDIT_REPORT_2026-05-08.md`;
> multiple Phase 1 items in this file have already been implemented.

Source: PROJECT_AUDIT_REPORT.md (2026-05-07)  
Cross-checked: 2026-05-07 — all code bugs from cross-check incorporated below.  
Approach: Fix high-severity runtime bugs first, then refactor for testability, then deployment hygiene.

---

## Cross-Check Corrections Applied To This Plan

The following bugs were found in the first draft of this plan and have been corrected in-place:

| # | Bug | Where it was | Correction applied |
|---|-----|-------------|-------------------|
| 1 | `person_ned[2]` IndexError — EKF returns 2-tuple not 3-tuple | P1-3 | Fixed: use `-(cfg.FOLLOW_ALTITUDE_M)` |
| 2 | Wrong `None` guard on EKF — `get_position_ned()` never returns `None` | P1-3 | Fixed: use `self._ekf.is_valid` flag |
| 3 | `self.conn` does not exist — attribute is `self._mav` | P1-5 ACK helper | Fixed throughout |
| 4 | `MAV_CMD_NAV_LOITER_UNLIM` not ACK-able the same way — use `MAV_CMD_DO_SET_MODE` | P1-5 | Fixed: loiter/RTL now use mode-switch pattern matching `set_mode_guided()` |
| 5 | `center()` at siyi_controller.py:318-319 resets `_cmd_yaw_deg`/`_cmd_pitch_deg` — removing them breaks `center()` | P1-2 | Fixed: keep attributes, add `attitude_is_fresh()` and `GIMBAL_COAST_S` instead |
| 6 | `math.radians(None)` crash — None guard in tracker.py must wrap entire block including radians calls | P1-2 | Fixed: gate whole block, not just `set_gimbal_angles()` |
| 7 | `_resolve_device` placed in `tracker.py` but P3-2 moves it to `Detector` — double work | P1-6 | Fixed: put in `detection/detector.py` from the start |
| 8 | `self._thread` already exists in StreamServer — plan re-described existing code as missing | P2-6 | Clarified: delta is only `server_close()` + `join()` + idempotent guard |
| 9 | `@dataclass(slots=True)` requires Python 3.10+ — Jetson may run 3.8 | P3-1 | Fixed: dropped `slots=True` |
| 10 | Empty `.git/` must be removed before `git init` | P4-5 | Added `rm -rf .git` pre-step |
| Q1 | Design: stale gimbal — freeze drone or coast? | P1-2 | Decision: coast on last-known angles for `GIMBAL_COAST_S=0.5 s`, then suppress |
| Q2 | Design: standoff bearing — velocity direction or drone-to-person? | P1-3 | Decision: velocity direction when EKF speed > 0.3 m/s, else drone-to-person |
| Q3 | Design: STREAM_HOST default | P1-4 | Decision: `127.0.0.1`; require token when host is not loopback |
| Q4 | Clarification: what does recorder thread actually record? | P1-1 | Confirmed: `_draw_overlay()` draws all bounding boxes — safe to remove main-loop write |
| 11 | P1-2 code snippet only gated `set_gimbal_angles()` — `update()` on line 1141-1146 also passes the same angles and must be inside the same gate | P1-2 | Fixed: gate covers entire block lines 1137-1153 including `update()` and pan correction |
| 12 | P2-2 fix was described as adding an `else` branch — but `_manual_key_t` line is unconditional (not in any branch), so the else must be inserted before it, not after | P2-2 | Fixed: explicit `else: return False` inserted before the timer line |
| 13 | P3-2 `_parse_results` used `for box in enumerate(boxes)` (object iteration) and `float(box.conf[0])` — existing code uses index-based access with `.cpu().item()` to handle CUDA tensors | P3-2 | Fixed: rewritten with index-based access matching tracker.py:202-214 pattern |

---

## Phase 1 — Fix Immediate Runtime Risks

Priority order matches the audit's H1–H6 severity ranking. Each item is self-contained; they can be done in parallel by separate engineers, except where the coupling note appears.

> **Coupling note — P1-2 and P1-3 must land together.** P1-2 adds `attitude_is_fresh()` and gates the gimbal-angle block in `tracker.py`. That same block calls `set_gimbal_angles()` which feeds the EKF used by the standoff math in P1-3. If P1-2 lands alone without P1-3, the tracker still works. If P1-3 lands alone without P1-2, stale gimbal angles silently corrupt the standoff bearing. Merge them in the same PR.

---

### P1-1 · Remove Duplicate Recorder Writes (H1)

**Problem:** Two paths call `self.recorder.write()` on the same `VideoRecorder` instance:
- Dedicated recorder thread: `tracker.py:756`
- Main loop: `tracker.py:1163-1167`

OpenCV `VideoWriter` is not thread-safe. Concurrent writes cause duplicate frames, timing corruption, and excess CPU load.

**Why the recorder thread is the right owner:**  
Confirmed by reading `_recorder_loop` (lines 731-756): the thread copies `self.grabber.frame`, then calls `_draw_overlay(frame, self._latest_target_info)`. `_draw_overlay` draws every visual element — bounding rectangles (lines 807, 818), target circle (line 827), ID labels, crosshair, dead-zone rect, telemetry text, battery bar, velocity arrow. There is no `results.plot()` call or separate box drawing in the main detection block. The recorder thread produces identical output to the main loop write.

**Files touched:** `tracker.py`

**Steps:**

1. Delete `tracker.py:1163-1167` (the `rec_frame = frame.copy()` block and `recorder.write(rec_frame)` call).
2. No other changes needed. The recorder thread already handles overlay via `cfg.RECORD_OVERLAY`.

**Verification:**  
```bash
grep -n "recorder.write" tracker.py
# Must return exactly ONE match (inside _recorder_loop, line ~756)
```
Run with `AUTO_RECORD=True`; record 10 seconds; confirm no duplicate frames in output video (check with `ffprobe -show_frames recording.mp4 | grep -c pkt_dts`).

---

### P1-2 · Fix Gimbal Angle Fallback (H2)

**Problem:** When SIYI attitude telemetry is stale, `gimbal_pan_deg` / `gimbal_tilt_deg` return `_cmd_yaw_deg` / `_cmd_pitch_deg` (both permanently 0.0 because `set_speed()` never integrates into them). The drone EKF projection and yaw recentering use these 0° values, silently corrupting geolocation.

**Design decision (Q1):** Do not completely freeze the drone when a single UDP packet is late. Instead, coast on the last-known valid telemetry angles for up to `GIMBAL_COAST_S` seconds. After that, suppress EKF updates until fresh telemetry arrives. This tolerates normal UDP jitter while still failing safe on sustained loss.

**Files touched:** `config/config.py`, `gimbal/siyi_controller.py`, `tracker.py`

**Steps:**

**config/config.py** — add after `GIMBAL_ATTITUDE_STALE_S`:
```python
GIMBAL_COAST_S: float = 0.5  # suppress EKF update if telemetry stale longer than this
```

**gimbal/siyi_controller.py** — keep `_cmd_yaw_deg` and `_cmd_pitch_deg` attributes unchanged (do not remove them — `center()` at line 318-319 resets them, removing them would raise `AttributeError` there). Add only a public freshness helper after the `gimbal_tilt_deg` property:

```python
def attitude_is_fresh(self) -> bool:
    """Return True if gimbal attitude telemetry is within GIMBAL_COAST_S."""
    with self.lock:
        return (time.monotonic() - self._att_time) < cfg.GIMBAL_COAST_S
```

**tracker.py lines 1137-1153** — the actual block spans both `set_gimbal_angles()` and `update()`:
```python
# CURRENT (lines 1137-1153):
self.drone_ctrl.set_gimbal_angles(
    pan_rad  = math.radians(self.ctrl.gimbal_pan_deg),   # line 1138
    tilt_rad = math.radians(self.ctrl.gimbal_tilt_deg),  # line 1139
)
pan_correction = self.drone_ctrl.update(
    gimbal_pan_deg         = self.ctrl.gimbal_pan_deg,   # line 1142
    gimbal_tilt_deg        = self.ctrl.gimbal_tilt_deg,  # line 1143
    target_info            = target_info,
    drone_tracking_enabled = self.tracking_enabled and self.mode == "AUTO",
)
if pan_correction != 0.0 and self.state == State.TRACKING:
    correction_speed = int(pan_correction * (180.0 / math.pi))
    self.ctrl.set_speed(...)
```

`gimbal_pan_deg` / `gimbal_tilt_deg` are accessed **four times** across both calls (lines 1138, 1139, 1142, 1143). The freshness gate must wrap the entire block from `set_gimbal_angles` through `pan_correction`:

```python
# REPLACEMENT:
if self.ctrl.attitude_is_fresh():
    self.drone_ctrl.set_gimbal_angles(
        pan_rad  = math.radians(self.ctrl.gimbal_pan_deg),
        tilt_rad = math.radians(self.ctrl.gimbal_tilt_deg),
    )
    pan_correction = self.drone_ctrl.update(
        gimbal_pan_deg         = self.ctrl.gimbal_pan_deg,
        gimbal_tilt_deg        = self.ctrl.gimbal_tilt_deg,
        target_info            = target_info,
        drone_tracking_enabled = self.tracking_enabled and self.mode == "AUTO",
    )
    if pan_correction != 0.0 and self.state == State.TRACKING:
        correction_speed = int(pan_correction * (180.0 / math.pi))
        self.ctrl.set_speed(
            max(-100, min(100, self.telem_yaw_cmd + correction_speed)),
            self.telem_pitch_cmd,
        )
else:
    # Telemetry stale beyond GIMBAL_COAST_S — skip EKF update and yaw correction
    pass  # drone continues coasting on its own velocity; no new position target sent
```

Do not move the freshness check inside `set_gimbal_angles()` — the `math.radians()` calls at lines 1138-1139 execute before entering the method and would silently use stale 0.0 values if not gated here.

**Verification:** Disconnect SIYI UDP. Within `GIMBAL_COAST_S` seconds, the log should still show drone updates. After `GIMBAL_COAST_S` seconds, confirm `"skip EKF update"` log message appears and no position target is sent.

---

### P1-3 · Implement Horizontal Standoff (H3)

> Must be merged together with P1-2 (see coupling note above).

**Problem:** `DroneController.update()` lines 278-283 command the raw EKF person position as the drone target. `FOLLOW_STANDOFF_M` is referenced in the README but not in `config/config.py` and has no effect.

**Design decision (Q2 — standoff bearing direction):** Use the person's EKF velocity direction when the estimated speed exceeds 0.3 m/s (person is moving — their heading is meaningful). Fall back to drone-to-person bearing when the person is slow or stationary (velocity direction is noise-dominated). `get_velocity_ned()` is confirmed available at `person_geolocation.py:313`.

**Files touched:** `config/config.py`, `control/drone_controller.py`

**config/config.py** — add under the "Drone Body Control" section:
```python
FOLLOW_STANDOFF_M: float     = 8.0   # horizontal distance to hold behind/away from person
MIN_PERSON_DRONE_SEP_M: float = 4.0  # hard minimum; drone hovers if closer than this
STANDOFF_VEL_THRESHOLD_MS: float = 0.3  # use person velocity direction above this speed
```

**control/drone_controller.py lines 275-293** — replace the current target block with:
```python
pN, pE       = self._ekf.get_position_ned()          # 2-tuple, never None
drone_pN, drone_pE, _ = self._mav.get_position_ned() # 3-tuple

# EKF validity must be confirmed before using pN/pE
# (caller already checked self._ekf.is_valid before calling _compute_follow_velocity,
#  but this block is also reached from update() directly — guard here too)
if not self._ekf.is_valid or not self._origin_set:
    self._mav.send_zero_velocity()
    return 0.0

# Bearing for standoff offset
vN, vE = self._ekf.get_velocity_ned()
speed  = math.hypot(vN, vE)
if speed > cfg.STANDOFF_VEL_THRESHOLD_MS:
    # Use person's velocity direction: standoff is "behind" their direction of travel
    bearing = math.atan2(vE, vN)
else:
    # Person is slow/stationary: stand off along drone-to-person vector
    delta_n = pN - drone_pN
    delta_e = pE - drone_pE
    bearing = math.atan2(delta_e, delta_n) if math.hypot(delta_n, delta_e) > 0.1 else 0.0

# Hard minimum separation guard — hover if already too close
sep = math.hypot(pN - drone_pN, pE - drone_pE)
if sep < cfg.MIN_PERSON_DRONE_SEP_M:
    self._mav.send_zero_velocity()
    return 0.0

# Desired position: FOLLOW_STANDOFF_M behind person along bearing
target_pN = pN - math.cos(bearing) * cfg.FOLLOW_STANDOFF_M
target_pE = pE - math.sin(bearing) * cfg.FOLLOW_STANDOFF_M
target_pD = -(cfg.FOLLOW_ALTITUDE_M)  # NED down is negative; altitude from config only
                                       # NOTE: pN/pE are 2D — EKF has no altitude component

# Altitude safety clamp (already exists)
safe_alt  = self._safety.check_altitude(cfg.FOLLOW_ALTITUDE_M)
target_pD = -safe_alt

self._mav.send_position_velocity_ned(
    target_pN, target_pE, target_pD,
    vN, vE, 0.0,
)
return pan_correction_rads
```

**Verification:** In SITL, start drone at origin, place person at 20 m N. Drone should converge to ~12 m N (20 - 8 m standoff). Walking person north: drone should follow at 8 m offset. Place drone at 2 m from person: drone should hover, not approach.

---

### P1-4 · Restrict Web Control Server Access (H4)

**Problem:** `gcs/stream_server.py:497` binds to `"0.0.0.0"`. Control endpoints (`/click`, `/unlock`, `/mode`, `/gimbal`, `/zoom_in`, `/zoom_out`) require no authentication.

**Design decision (Q3):** Default to `STREAM_HOST=127.0.0.1`. When the host is not loopback, require `STREAM_TOKEN` to be non-empty (operators deploying over LAN/Tailscale must set both). The MJPEG `/stream` endpoint is read-only and exempt from token auth so browser `<img>` tags work without credentials.

**Files touched:** `config/config.py`, `gcs/stream_server.py`, `main.py`

**config/config.py** — add under "Live Streaming":
```python
STREAM_HOST: str  = "127.0.0.1"  # "0.0.0.0" for LAN/Tailscale access
STREAM_TOKEN: str = ""            # required (non-empty) when STREAM_HOST != "127.0.0.1"
```

**gcs/stream_server.py** — add `import urllib.parse` at top if not present.

In `start()` (line 497) change the bind address:
```python
self._server = _ThreadingHTTPServer((cfg.STREAM_HOST, self._port), _Handler)
```

Add a token-check helper inside `_Handler` (before `do_GET`):
```python
_CONTROL_PATHS = {'/click', '/unlock', '/mode', '/gimbal', '/zoom_in', '/zoom_out'}

def _check_token(self) -> bool:
    # /stream and /status are read-only — always allowed
    path = urllib.parse.urlparse(self.path).path
    if path not in _CONTROL_PATHS:
        return True
    if not cfg.STREAM_TOKEN:
        # No token configured: allow only from loopback
        return self.client_address[0] in ('127.0.0.1', '::1')
    params = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
    return params.get('token', [''])[0] == cfg.STREAM_TOKEN
```

At the top of `do_GET`, before routing:
```python
if not self._check_token():
    self.send_response(403)
    self.send_header('Content-Type', 'text/plain')
    self.end_headers()
    self.wfile.write(b'Forbidden')
    return
```

**main.py** — add to CLI args:
```python
parser.add_argument('--stream-host',  default=None, help='Bind address for stream server')
parser.add_argument('--stream-token', default=None, help='Auth token for control endpoints')
```
Write back to `cfg` after parsing, before constructing the tracker.

**Startup warning** — in `start()`, after the bind line, add:
```python
if cfg.STREAM_HOST != '127.0.0.1' and not cfg.STREAM_TOKEN:
    print("[Stream] WARNING: server is exposed on network without a token — set STREAM_TOKEN")
```

**Verification:**  
- Default: `curl http://127.0.0.1:8080/mode?mode=MANUAL` → 200; from another host → 403.  
- With token: `curl "http://host:8080/mode?mode=MANUAL&token=secret"` → 200; without token → 403.  
- Stream: `curl http://host:8080/stream` → MJPEG response, no token needed.

---

### P1-5 · Add COMMAND_ACK Confirmation for Safety Mode Commands (H5)

**Problem:** `send_loiter()` (line 528) and `send_rtl()` (line 547) send commands and return without waiting for `COMMAND_ACK`. A rejected or lost command is silent.

**Corrections from cross-check:**
- Plan originally used `self.conn` — correct attribute is `self._mav`.
- `MAV_CMD_NAV_LOITER_UNLIM` (17) is a NAV waypoint command; ArduPilot's ACK behaviour for it inside GUIDED mode is firmware-version-dependent. `MAV_CMD_NAV_RETURN_TO_LAUNCH` (20) similarly. The ACK-reliable pattern is `MAV_CMD_DO_SET_MODE` (176), which `set_mode_guided()` already uses at line 426. Rewrite `send_loiter()` and `send_rtl()` to use mode switches matching this pattern:
  - LOITER flight mode = `MAV_CMD_DO_SET_MODE`, param2=5
  - RTL flight mode    = `MAV_CMD_DO_SET_MODE`, param2=6

> **Behavioral note:** Switching to LOITER *flight mode* gives ArduPilot's built-in position hold with RC authority. Sending `NAV_LOITER_UNLIM` while in GUIDED mode commands a guided hold-in-place without switching modes. The mode-switch approach is safer for failsafe: it removes the tracker from the control loop entirely and lets ArduPilot hold position independently.

**Files touched:** `mavlink_client/mavlink_client.py`

**Step 1 — Add ACK registry to `MAVLinkClient.__init__`:**
```python
import threading
self._ack_events:  dict[int, threading.Event] = {}
self._ack_results: dict[int, int]             = {}
self._ack_lock    = threading.Lock()
```

**Step 2 — Handle `COMMAND_ACK` in `_rx_loop`** (around line 284, after existing message handlers):
```python
elif msg_type == 'COMMAND_ACK':
    cmd_id = msg.command
    result = msg.result
    with self._ack_lock:
        ev = self._ack_events.get(cmd_id)
        if ev is not None:
            self._ack_results[cmd_id] = result
            ev.set()
```

**Step 3 — Add `send_command_with_ack()` helper:**
```python
def send_command_with_ack(
    self,
    command: int,
    p1: float = 0, p2: float = 0, p3: float = 0,
    p4: float = 0, p5: float = 0, p6: float = 0, p7: float = 0,
    retries: int = 3,
    timeout: float = 2.0,
) -> bool:
    """Send a MAVLink command_long and wait for COMMAND_ACK.

    Returns True if accepted (result == MAV_RESULT_ACCEPTED == 0).
    """
    if self._mav is None:
        return False
    ev = threading.Event()
    with self._ack_lock:
        self._ack_events[command] = ev
        self._ack_results.pop(command, None)
    try:
        for attempt in range(retries):
            ev.clear()
            self._mav.mav.command_long_send(        # NOTE: self._mav, not self.conn
                self._mav.target_system,
                self._mav.target_component,
                command, 0,
                p1, p2, p3, p4, p5, p6, p7,
            )
            if ev.wait(timeout):
                result = self._ack_results.get(command, -1)
                if result == 0:
                    return True
                logging.warning("[MAVLink] ACK rejected command %d result %d", command, result)
                return False
            logging.warning("[MAVLink] ACK timeout command %d attempt %d/%d",
                            command, attempt + 1, retries)
    finally:
        with self._ack_lock:
            self._ack_events.pop(command, None)
    return False
```

**Step 4 — Rewrite `send_loiter()` and `send_rtl()` using mode switches:**
```python
def send_loiter(self) -> bool:
    """Switch to LOITER flight mode (ACK-confirmed).

    Safe failsafe on tracking loss. Removes tracker from control loop.
    Uses MAV_CMD_DO_SET_MODE mode=5 — same pattern as set_mode_guided().
    """
    ok = self.send_command_with_ack(
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        p2=5,  # ArduCopter LOITER mode number
    )
    if ok:
        print("[MAVLink] LOITER mode confirmed")
    else:
        print("[MAVLink] WARNING: LOITER mode not confirmed")
    return ok

def send_rtl(self) -> bool:
    """Switch to RTL flight mode (ACK-confirmed). Battery-critical only.

    Uses MAV_CMD_DO_SET_MODE mode=6 — same pattern as set_mode_guided().
    """
    ok = self.send_command_with_ack(
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        p2=6,  # ArduCopter RTL mode number
    )
    if ok:
        print("[MAVLink] RTL mode confirmed")
    else:
        print("[MAVLink] WARNING: RTL mode not confirmed")
    return ok
```

**Step 5** — Apply same ACK pattern to `set_mode_guided()` (currently no ACK):
Replace the existing `command_long_send` in `set_mode_guided()` with:
```python
return self.send_command_with_ack(
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    p2=4,  # GUIDED
)
```

**Step 6** — In `DroneController`, check return value of `send_loiter()` and `send_rtl()`. Log warning if rejected. Surface `last_safety_cmd_failed: bool` flag for future health monitoring.

**Verification:** In SITL, break the MAVLink connection mid-flight; confirm log shows ACK timeout. Reconnect; issue loiter; confirm ACK receipt logged with result=0.

---

### P1-6 · Make YOLO Device Configurable (H6)

**Problem:** `tracker.py:1072` and `tracker.py:1083` hardcode `device=0`. Both primary and fallback inference fail on CPU-only machines or with a different GPU index.

**Correction from cross-check:** `_resolve_device` must live in `detection/detector.py`, not `tracker.py`, so Phase 3's P3-2 can inherit it without moving code twice.

**Files touched:** `config/config.py`, `detection/detector.py`, `tracker.py`, `main.py`

**config/config.py** — add under "Camera/Network":
```python
DETECT_DEVICE: str = "auto"  # "auto" | "cpu" | "cuda:0" | "0"
```

**detection/detector.py** — add device resolver here (not in tracker.py):
```python
def _resolve_device(pref: str) -> str:
    if pref != "auto":
        return pref
    try:
        import torch
        return "0" if torch.cuda.is_available() else "cpu"
    except ImportError:
        return "cpu"

class Detector:
    def __init__(self, model_path: str) -> None:
        self.device = _resolve_device(cfg.DETECT_DEVICE)
        self.model  = YOLO(model_path)
```

**tracker.py** — resolve device from `self.detector.device`:
- Line 1072: change `device=0` → `device=self.detector.device`
- Line 1083: change `device=0` → `device=self.detector.device`
- Line 1076 (bare `except`): add logging before fallback:
  ```python
  except Exception as e:
      logging.warning("[Detect] model.track failed (%s), retrying without tracker", e)
  ```

**main.py** — add CLI arg:
```python
parser.add_argument('--detect-device', default=None,
                    help='"auto" | "cpu" | "cuda:0"')
```
Write back to `cfg.DETECT_DEVICE` before constructing `Detector`.

**Verification:** `DETECT_DEVICE=cpu python3 main.py --headless` runs without CUDA errors and logs at least one detection frame. `DETECT_DEVICE=auto` on a CUDA machine selects `"0"`.

---

## Phase 2 — Medium Severity Fixes

These are independently fixable; no ordering dependency among them.

---

### P2-1 · Fix Stream Query Parsing (M1)

**File:** `gcs/stream_server.py` lines 425, 431, 442, 454

Current fragile pattern:
```python
dict(p.split('=') for p in qs.split('&') if '=' in p)
```
Replace every instance with:
```python
urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query, keep_blank_values=False)
```
`parse_qs` returns `dict[str, list[str]]`. Access values with `params.get("key", [""])[0]`.  
Add `import urllib.parse` at the top if not already present (it is needed by P1-4 too — add once).

---

### P2-2 · Fix Invalid Gimbal Direction Keeps Motion Alive (M2)

**File:** `tracker.py` lines 656-669 (`handle_web_gimbal`)

Current `else` branch at line 668 extends `_manual_key_t = time.time() + 86400.0` for unknown directions, keeping the previous movement alive. Replace with:
```python
else:
    logging.warning("[Web] Unknown gimbal direction: %r", direction)
    return False
```
**Important:** the `self._manual_key_t = time.time() + 86400.0` line (line 668) is **not** in an `else` — it runs unconditionally after all `if-elif` branches. An unknown direction falls through all branches, hits the timer line, and extends the hold. The fix is to insert an explicit `else` guard **before** the timer line:

```python
        elif direction == 'stop':
            self._manual_yaw_speed   = 0
            self._manual_pitch_speed = 0
            self.ctrl.stop()
        else:
            logging.warning("[Web] Unknown gimbal direction: %r", direction)
            return False   # caller sends HTTP 400; do NOT fall through to timer
        self._manual_key_t = time.time() + 86400.0
        return {'status': 'ok', 'direction': direction}
```

In `do_GET` for the `/gimbal` endpoint, check the return value: if `False`, send HTTP 400:
```python
ok = self._callbacks['gimbal'](direction)
code = 200 if ok else 400
self._send_json({'ok': ok}, code)
```

---

### P2-3 · Fix Auto-Zoom State Desync for Relative Zoom (M3)

**File:** `gimbal/siyi_controller.py` lines 325-329

`zoom_in()` and `zoom_out()` do not update `current_zoom`. `auto_zoom.py:49-51` checks `self.ctrl.current_zoom` and repeatedly issues zoom commands because the level never changes.

Add estimated zoom state update:
```python
# Step size is approximate — replace with calibrated deg/zoom-level value
# once hardware is measured. 0.5 is a placeholder.
_ZOOM_STEP_EST = 0.5

def zoom_in(self) -> None:
    self._send(0x05, struct.pack("b", 1))
    self.current_zoom = min(self.current_zoom + _ZOOM_STEP_EST, cfg.ZOOM_MAX)

def zoom_out(self) -> None:
    self._send(0x05, struct.pack("b", -1))
    self.current_zoom = max(self.current_zoom - _ZOOM_STEP_EST, 1.0)
```

> **TODO before field flight:** Measure the SIYI A8 mini's continuous zoom rate (zoom levels per second) and replace `_ZOOM_STEP_EST` with `zoom_rate_per_s * zoom_cmd_duration_s`.

---

### P2-4 · Fix Swallowed Exceptions in Critical Paths (M6)

Add rate-limited logging at each bare `except Exception: pass` site:

| File | Lines | Fix |
|------|-------|-----|
| `tracker.py` | 1076 | Done in P1-6: `logging.warning("[Detect] model.track failed: %s", e)` |
| `control/drone_controller.py` | 337-338 | `logging.warning("[EKF] update error: %s", e)` |
| `mavlink_client/mavlink_client.py` | 284-285 | `logging.debug("[MAVLink] RX loop error: %s", e)` |
| `utils/frame_grabber.py` | 88-102 | `logging.warning("[Grabber] pipeline failed: %s", e)` |

For `mavlink_client.py` RX loop, use `logging.debug` (not warning) because transient decode errors are normal; escalate to warning only after N consecutive failures using a counter.

---

### P2-5 · Fix Model Path Resolution (M7)

**Files:** `config/config.py`, `main.py`

In `main.py`, after CLI arg parsing and before constructing the tracker, resolve the model path relative to the project root regardless of the working directory:
```python
import pathlib
_PROJECT_ROOT = pathlib.Path(__file__).resolve().parent
if not pathlib.Path(cfg.MODEL_PATH).is_absolute():
    cfg.MODEL_PATH = str(_PROJECT_ROOT / cfg.MODEL_PATH)
if not pathlib.Path(cfg.MODEL_PATH).exists():
    sys.exit(f"[Main] Model not found: {cfg.MODEL_PATH}")
```
The early existence check surfaces deployment errors immediately rather than at first inference.

---

### P2-6 · Fix Stream Server Restart Lifecycle (M8)

**File:** `gcs/stream_server.py` lines 505-511

`self._server` and `self._thread` already exist (lines 60-61, 498-500). The only missing pieces are `server_close()`, thread `join()`, and an idempotent start guard.

Update `stop()`:
```python
def stop(self) -> None:
    if not self._active:
        return
    self._active = False
    with self._cond:
        self._cond.notify_all()
    if self._server is not None:
        self._server.shutdown()
        self._server.server_close()   # release the socket immediately
        self._server = None
    if self._thread is not None and self._thread.is_alive():
        self._thread.join(timeout=3.0)
        self._thread = None
```

Add idempotent guard to `start()`:
```python
def start(self) -> None:
    if self._active:
        return
    ...
```

---

### P2-7 · Set Ultralytics Config Dir (M9)

**File:** `main.py` — add before any Ultralytics-related import:
```python
import os, pathlib as _pl
_ul = _pl.Path("/tmp/ultralytics")
_ul.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("YOLO_CONFIG_DIR", str(_ul))
```
This prevents Ultralytics from writing to unpredictable system paths under systemd or read-only users.

---

### P2-8 · Fix Documentation Staleness (M4)

**File:** `readme.md`

| Location | Current (wrong) | Correct |
|----------|----------------|---------|
| Line 91 | `gimbal/siyi_gimbal.py` | `gimbal/siyi_controller.py` |
| Line 111 | `utils/colors.py` | Remove — file does not exist |
| Line 219 | `FOLLOW_STANDOFF_M` documented as unimplemented | Update after P1-3 merges |
| All commands | `python main.py` | `python3 main.py` |

---

## Phase 3 — Structural Refactors (Make Code Testable)

Each item is a separate feature branch. Review independently.

---

### P3-1 · Introduce `TargetDetection` Dataclass

**New file:** `tracking/target_detection.py`

```python
from dataclasses import dataclass

# NOTE: slots=True requires Python 3.10+. Drop it if targeting JetPack 4.x (Python 3.8).
# Confirmed: JetPack 5.x ships 3.10; JetPack 4.x ships 3.8. Check before enabling.
@dataclass
class TargetDetection:
    cx:       float   # normalised center x [0, 1]
    cy:       float   # normalised center y [0, 1]
    x1:       float   # normalised bbox left
    y1:       float   # normalised bbox top
    x2:       float   # normalised bbox right
    y2:       float   # normalised bbox bottom
    conf:     float   # detection confidence
    track_id: int     # ByteTrack ID (-1 if untracked)
```

Replace all `target_info[N]` tuple index accesses in `tracker.py` and `control/drone_controller.py`:
```bash
grep -n "target_info\[" tracker.py control/drone_controller.py
```

---

### P3-2 · Thicken the `Detector` Abstraction

**File:** `detection/detector.py`

`_resolve_device` already lives here from P1-6. Add the full inference method so `tracker.py` no longer calls Ultralytics APIs directly:

```python
from tracking.target_detection import TargetDetection

class Detector:
    def __init__(self, model_path: str) -> None:
        self.device = _resolve_device(cfg.DETECT_DEVICE)
        self.model  = YOLO(model_path)

    def detect_and_track(
        self, frame, conf: float, imgsz: int, persist: bool = True
    ) -> list[TargetDetection]:
        try:
            results = self.model.track(
                frame, conf=conf, imgsz=imgsz, device=self.device,
                tracker="bytetrack.yaml", persist=persist, verbose=False,
            )
        except Exception as e:
            logging.warning("[Detect] model.track failed (%s), retrying without tracker", e)
            results = self.model(frame, conf=conf, imgsz=imgsz, device=self.device, verbose=False)
        return _parse_results(results)

def _parse_results(results) -> list[TargetDetection]:
    """Parse Ultralytics Results into TargetDetection list.

    Uses index-based access (boxes.xyxyn[i], boxes.conf[i]) matching the
    existing pattern in tracker.py:202-214 to avoid CUDA tensor issues.
    Normalised coords [0,1] so TargetDetection is resolution-independent.
    """
    detections = []
    if not results or len(results[0].boxes) == 0:
        return detections
    boxes  = results[0].boxes
    bt_ids = boxes.id
    fw, fh = results[0].orig_shape[1], results[0].orig_shape[0]
    for i in range(len(boxes)):
        c = boxes.xyxy[i].cpu().numpy()                  # pixel coords
        x1n, y1n = float(c[0]) / fw, float(c[1]) / fh   # normalise
        x2n, y2n = float(c[2]) / fw, float(c[3]) / fh
        cxn = (x1n + x2n) / 2
        cyn = (y1n + y2n) / 2
        conf = float(boxes.conf[i].cpu().item())          # .cpu().item() for CUDA tensors
        tid  = int(bt_ids[i].cpu().item()) if bt_ids is not None else -1
        detections.append(TargetDetection(
            cx=cxn, cy=cyn,
            x1=x1n, y1=y1n, x2=x2n, y2=y2n,
            conf=conf, track_id=tid,
        ))
    return detections
```

`tracker.py` calls `self.detector.detect_and_track(frame, cfg.CONF_THRESHOLD, cfg.IMGSZ)` and receives `list[TargetDetection]`.

---

### P3-3 · Introduce Immutable `Settings` Object

**New file:** `config/settings.py`

```python
from dataclasses import dataclass, fields
import config.config as _cfg

@dataclass(frozen=True)
class Settings:
    camera_ip:             str   = _cfg.CAMERA_IP
    model_path:            str   = _cfg.MODEL_PATH
    detect_device:         str   = _cfg.DETECT_DEVICE
    follow_standoff_m:     float = _cfg.FOLLOW_STANDOFF_M
    # ... one field per config entry, using exact lowercase names

def load_settings(**overrides) -> Settings:
    base = {}
    for f in fields(Settings):
        attr = f.name.upper()
        if hasattr(_cfg, attr):
            base[f.name] = getattr(_cfg, attr)
        else:
            base[f.name] = f.default   # field has no matching _cfg entry
    base.update(overrides)
    return Settings(**base)
```

> **Naming convention assumption:** field `foo_bar` maps to `_cfg.FOO_BAR`. Verify every field during migration — any mismatch silently uses the dataclass default. Add an assertion loop in `load_settings` if needed.

Pass a `Settings` instance into every class constructor. Migrate one class at a time. Start with `SafetyMonitor` (fewest external dependencies), then `MAVLinkClient`, then `DroneController`.

---

### P3-4 · Split `tracker.py` into Focused Units

Target ≤ 300 LOC per module. Suggested decomposition:

| New module | Responsibility | Extracted from tracker.py |
|---|---|---|
| `mission/mission_loop.py` | Main `run()` loop, per-frame orchestration | lines 953–1305 |
| `mission/target_selector.py` | `_select_target`, lock/unlock, web click | lines 200–350 |
| `mission/state_machine.py` | TRACKING/SEARCHING/HOVERING/LOST transitions | lines 400–600 |
| `mission/hud_renderer.py` | `_draw_overlay`, all visual drawing | lines 762–950 |
| `mission/input_controller.py` | `_handle_key`, keyboard dispatch | lines 1039–1060 |
| `mission/recording_manager.py` | `_recorder_loop`, start/stop recording | lines 727–760 |
| `mission/stream_adapter.py` | Stream push, quality switching | lines 700–727 |

`tracker.py` becomes a thin assembler wiring these units together.  
**Approach:** Extract one unit at a time; keep all existing behaviour; merge incrementally.

> **Ordering dependency:** P3-4 makes P3-5 test writing significantly easier. Write tests for each extracted unit as it is created, not after all units are done.

---

### P3-5 · Add Test Structure

**New directory:** `tests/`

Minimum tests before field flight:

| Module | What to test |
|---|---|
| `tests/test_pid.py` | Output limits, dead zone clamping, derivative kick suppression |
| `tests/test_safety.py` | Altitude clamp, speed clamp, geofence boundary, battery threshold |
| `tests/test_siyi_crc.py` | CRC16 test vectors from SIYI SDK, packet round-trip |
| `tests/test_query_parse.py` | Valid query, `=` in value, URL-encoded chars, malformed input |
| `tests/test_drone_gates.py` | Each gate (not GUIDED, not armed, no GPS, no home, battery critical) blocks velocity |
| `tests/test_target_detection.py` | Dataclass construction, field access, equality |
| `tests/test_standoff.py` | Standoff bearing: velocity direction vs drone-to-person fallback, separation guard |
| `tests/test_mavlink_ack.py` | ACK success, ACK rejection, ACK timeout, retry count |

Add `pytest>=8.0` to `requirements.txt`. Run with `python3 -m pytest tests/ -v`.

---

## Phase 4 — Deployment Hygiene

---

### P4-1 · Add `requirements.txt`

```text
# runtime
opencv-python-headless>=4.8
numpy>=1.24
torch>=2.1
ultralytics>=8.0
pymavlink>=2.4

# dev / test
pytest>=8.0
ruff>=0.4
```

**Jetson note:** `torch` on JetPack must come from NVIDIA's Jetson wheel index, not PyPI. Document this in the README or a `INSTALL_JETSON.md`.

---

### P4-2 · Add `.gitignore`

```gitignore
__pycache__/
*.pyc
*.pyo
*.egg-info/
dist/
build/
.env
*.engine
*.pt
recordings/
logs/
/tmp/
.ultralytics/
```

---

### P4-3 · Migrate Model Files Out of Source Tree

`models/` is 88 MB. Options ranked by effort:

| Option | When to use |
|---|---|
| Git LFS | Team ≤ 10, model < 500 MB. `git lfs track "*.engine" "*.pt"` |
| GitHub Releases | Any size. Tag each model version; add `scripts/fetch_models.sh` |
| S3/Spaces | Multi-deployment. Add `MODEL_ASSET_URL` config + downloader script |

Default recommendation: Git LFS now; switch to release artifacts when model exceeds 500 MB.

---

### P4-4 · Fix Stale Diagnostic Utilities (M5)

For each file with hardcoded `CAMERA_IP` or old absolute paths:

| File | Fix |
|---|---|
| `utils/yolo_gpu_optimized.py:64-66` | `from config import config as cfg; CAMERA_IP = cfg.CAMERA_IP` |
| `utils/yolo_test_simple.py:12-14` | Same |
| `utils/diagnostic.py:41,63` | Same; fix title to "SIYI A8 mini" (was "YOLO11 + ZR30") |
| `utils/yolo_debug_optimized.py` | Add `argparse` for `--camera-ip`, `--model`, `--device` |

All four files: add `if __name__ == "__main__": argparse` entry points with `--camera-ip`, `--model`, `--device` CLI flags.

---

### P4-5 · Initialize a Valid Git Repository

The `.git/` directory is empty and invalid. Clean up before initialising:

```bash
cd /home/ai-engineer/Akash/Tasks/Akash/AI/Jetson/person_tracking
rm -rf .git                  # remove empty stub first — git init inside an invalid .git can behave unexpectedly
git init
git add .gitignore requirements.txt main.py tracker.py \
        config/ control/ detection/ gcs/ gimbal/ \
        mavlink_client/ safety/ tracking/ utils/ scripts/
git commit -m "chore: initial tracked source after audit remediation"
```

Do **not** add `models/`, `recordings/`, `logs/`, or `__pycache__/`.

---

## Execution Order Summary

```
Sprint 1 (1-2 days):  P1-1, P1-6, P2-1, P2-2, P2-7
Sprint 2 (2-3 days):  P1-2 + P1-3 (together), P2-3, P2-4, P2-5, P2-6
Sprint 3 (3-4 days):  P1-4, P1-5, P2-8, P4-1, P4-2, P4-5
Sprint 4 (ongoing):   P3-1, P3-2, P3-3, P3-4 + P3-5 (interleaved), P4-3, P4-4
```

Sprints 1–3 are the minimum required before field flight.  
Sprint 4 is ongoing structural health with no field-flight blocker status.

---

## Done Criteria per Sprint

| Sprint | Gate |
|---|---|
| Sprint 1 | `grep -n "recorder.write" tracker.py` → 1 match; `DETECT_DEVICE=cpu python3 main.py --headless` runs without CUDA errors; malformed `/gimbal?dir=bad` returns 400; no stream-server address-in-use on restart |
| Sprint 2 | Gimbal UDP disconnected → log shows "skip EKF update" after `GIMBAL_COAST_S`; SITL standoff holds 8 m distance; separation guard halts drone at < 4 m; no silent exceptions in detector/EKF/MAVLink/grabber logs |
| Sprint 3 | Control endpoint without token → 403; with token → 200; LOITER/RTL log shows "confirmed" with ACK result=0; README matches actual file tree |
| Sprint 4 | `pytest tests/ -v` all pass; `ruff check .` clean; `tracker.py` < 350 LOC; `Detector.detect_and_track()` is the only Ultralytics call site |
