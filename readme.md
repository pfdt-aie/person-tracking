# Person Tracking Drone

Real-time autonomous person-following system for a quadcopter, designed
for Jetson Orin Nano Super / Orin NX + SIYI A8 mini gimbal +
Orange Cube+ (ArduPilot) flight controller.

Gimbal-PID inner loop and proportional drone-body outer loop. EKF-smoothed
geolocation, 4-stage tracking-loss search escalation, persistent person
re-identification, and a full software safety stack (E-STOP, preflight
gate, retreat behaviour, FPS floor, RC override lock-out).

170 unit tests on every change.

---

## Tech stack

| Layer | Technology |
|---|---|
| **Compute** | Jetson Orin Nano Super / Orin NX, JetPack 6+, CUDA 12 |
| **Detection** | YOLOv8 via Ultralytics, TensorRT engine on GPU |
| **Tracker** | ByteTrack (multi-object), HSV-histogram re-identification |
| **State estimation** | Custom 4-state Extended Kalman Filter (pos N/E + vel N/E) |
| **Gimbal control** | Discrete PID (yaw + pitch), SIYI SDK v2.0 over UDP |
| **Drone control** | Proportional + EMA + jerk + accel limiter, pymavlink |
| **Flight controller** | ArduCopter on Orange Cube+, GUIDED mode |
| **Camera ingest** | RTSP H.264 → OpenCV/GStreamer (NVDEC hardware decode) |
| **Operator UI** | MJPEG-over-HTTP, browser-native JS, no plugins |
| **Recording** | H.264 via NVENC (`nvv4l2h264enc`), software fallback |
| **Language** | Python 3.10 |
| **Tests** | pytest (170 tests) |

---

## Hardware

| Component | Detail |
|---|---|
| Compute | Jetson Orin Nano Super / Orin NX (JetPack 6+) |
| Gimbal + camera | SIYI A8 mini — Wi-Fi `192.168.144.25`, UDP `:37260`, RTSP `:8554/main.264` |
| Flight controller | Orange Cube+ (ArduCopter) over USB serial `/dev/ttyACM0` @ 921600 baud |
| Battery | 3S–6S LiPo (auto-detected; override via `--cells`) |
| RC link | Any TX/RX bound to the FCU; required for manual override |

---

## Architecture

```
              ┌──────────────────────┐
RTSP H.264 →  │   FrameGrabber       │   (zero-latency NVDEC)
              └──────────┬───────────┘
                         ↓
              ┌──────────────────────┐
              │   YOLO + ByteTrack   │  (TensorRT, GPU)
              └──────────┬───────────┘
                         ↓
              ┌──────────────────────┐
              │  PersonRegistry      │  (HSV histogram re-ID)
              └──────────┬───────────┘
                         ↓
              ┌──────────────────────┐
              │  State Machine (8)   │  WAITING / SCAN / TRACK / PREDICT …
              └────┬─────────────┬───┘
                   │             │
        Inner 30 Hz│             │Outer 10 Hz (--drone only)
                   ↓             ↓
            ┌─────────────┐ ┌─────────────────────┐
            │ Gimbal PID  │ │  DroneController    │
            │  + Search   │ │  + EKF + Safety     │
            └──────┬──────┘ └──────────┬──────────┘
                   │                   │
              SIYI UDP            MAVLink serial
```

---

## Installation

```bash
# JetPack 6 ships Python 3.10. From the project root:
pip install -r requirements.txt

# Optional — drone body control
pip install pymavlink

# GStreamer hardware decode/encode is pre-installed on JetPack 6:
#   gstreamer1.0-plugins-bad, nvv4l2decoder, nvv4l2h264enc
```

---

## Quick start

```bash
cd /home/ai-engineer/Akash/Tasks/Akash/AI/Jetson/person_tracking
```

Three launch modes:

```bash
# 1. Gimbal-only (safest, no drone movement)
python3 main.py

# 2. Drone body following — BENCH rehearsal (no MAVLink TX, motors off)
python3 main.py --drone --ground-test

# 3. Drone body following — LIVE flight
python3 main.py --drone
```

Open the operator UI: **`http://<jetson-ip>:8080/`**

---

## Operating procedure

### Before every session

1. Power on SIYI camera (Wi-Fi 192.168.144.x).
2. Power on the drone (Orange Cube+ via USB serial).
3. Power on the RC transmitter. Mode switch on **LOITER** or **STABILIZE** (NOT GUIDED yet).
4. Launch `main.py` on the Jetson.
5. Open `http://<jetson-ip>:8080/` in a browser.

### To track a person (gimbal only)

1. Click on the person in the video.
2. Gimbal locks; HUD shows `LOCK ID N`.
3. Click empty space (or **UNLOCK**) to release.

### To enable autonomous drone following

1. Pilot switches RC mode to **GUIDED**.
2. Click **ARM ▾** in the web UI header. Checklist panel opens.
3. Wait until **all 10 items turn green** (MAVLink, heartbeat, GUIDED, ARMED, HOME, GPS quality, sensors, fence, battery, ArduPilot params).
4. Click **Arm Tracker**.
5. Click on the person in the video → drone follows at `FOLLOW_STANDOFF_M = 8 m`, `FOLLOW_ALTITUDE_M = 12 m`.
6. Click **Disarm** to stop body motion (gimbal continues).

### To take manual control — three layers

| Goal | Action | Effect |
|---|---|---|
| **Take over flight** | RC mode switch → anything not GUIDED (LOITER/STABILIZE/BRAKE/LAND/RTL) | FCU stops accepting our commands instantly. RC-override latch trips — auto-follow cannot resume until preflight + Arm again |
| **Take over camera** | Web UI **AUTO** button → **MANUAL** | D-pad appears; arrow keys drive pan/tilt |
| **Emergency stop** | Web UI red **E-STOP** button, or **Space** key | 1st press → BRAKE. 2nd press within 3 s → LAND. Always reachable, token-bypass |

### RC link conditions required

For autonomous following to be allowed: RC TX bound and healthy, mode on **GUIDED**, throttle above failsafe trigger.

For manual recovery (always available): transmitter must stay powered in pilot's hand throughout the flight. Any mode flip off GUIDED = instant takeover.

---

## CLI reference

```
python3 main.py [--drone] [--ground-test] [--cells N]
                [--device PATH] [--baud N]
                [--model PATH] [--calibration PATH] [--detect-device DEV]
                [--stream-host HOST] [--stream-token TOKEN]
```

| Flag | Default | Description |
|---|---|---|
| `--drone` | off | Enable MAVLink drone-body following |
| `--ground-test` | off | Dry-run: pipeline runs but no MAVLink TX |
| `--cells N` | 0 (auto) | Force battery cell count (3–6). Use on partially-charged packs |
| `--device PATH` | `/dev/ttyACM0` | MAVLink serial device |
| `--baud N` | `921600` | Serial baud rate |
| `--model PATH` | `models/yolo26s.engine` | YOLO TensorRT engine |
| `--detect-device DEV` | `auto` | `auto` / `cpu` / `cuda:0` |
| `--calibration PATH` | empty | OpenCV camera calibration YAML (else auto-estimate from HFOV=81°) |
| `--stream-host HOST` | `127.0.0.1` | Stream bind address. Use `0.0.0.0` for LAN / Tailscale |
| `--stream-token TOKEN` | empty | Auth token required on control endpoints when set |

---

## Keyboard (display window open)

| Key | Action |
|---|---|
| `q` | Quit |
| `t` | Toggle tracking on/off |
| `r` | Center gimbal + reset zoom |
| `s` | Toggle search on/off |
| `i` | Restart initial acquisition scan |
| `d` | Toggle display window |
| `z` / `x` | Zoom in / out |
| `a` | Toggle auto-zoom |
| `v` | Toggle video recording |
| `l` | Toggle live stream |
| `+` / `-` | Adaptive Kp gain |
| `[` / `]` | Max gimbal speed |
| `m` | Toggle AUTO / MANUAL |
| `←↑↓→` | Manual gimbal (only in MANUAL) |

---

## Terminal commands (headless / SSH)

```
track <id>   Lock to persistent person ID
unlock       Release lock
ids          Print currently detected IDs
q            Quit
```

---

## HTTP endpoints

| Endpoint | Auth | Description |
|---|---|---|
| `/` | — | Operator HTML page |
| `/stream` | — | MJPEG over HTTP (`multipart/x-mixed-replace`, JPEG frames) |
| `/status` | — | Live telemetry JSON: mode, GPS, batt, fence, RC-override, FPS, tracking-loss dt |
| `/preflight` | — | Preflight checklist JSON: `{items: [...], all_ok: bool, armed: bool}` |
| `/click?x=&y=` | token | Lock to normalised (x, y) |
| `/unlock` | token | Release lock |
| `/mode?set=auto\|manual` | token | Set tracker mode |
| `/gimbal?dir=up\|down\|left\|right\|stop` | token | Manual gimbal nudge (MANUAL only) |
| `/zoom_in` / `/zoom_out` | token | Gimbal zoom |
| `/arm_tracker?on=true\|false` | token | Arm or disarm drone-body following |
| `/estop` | **always allowed** | First press → BRAKE. Within 3 s → LAND. Life safety |

When `--stream-token` is set, control endpoints require `?token=YOURSECRET`. The `/estop` endpoint intentionally bypasses this.

---

## Streaming protocol

| Leg | Protocol | Notes |
|---|---|---|
| Camera → Jetson | RTSP / H.264 | `rtsp://192.168.144.25:8554/main.264`, NVDEC hardware decode |
| Jetson → operator browser | MJPEG over HTTP | `Content-Type: multipart/x-mixed-replace`. Two qualities served: 720p (`?q=hi`) and 480p (`?q=lo`). 25 fps cap |

MJPEG was chosen over WebRTC/HLS for sub-second latency and zero-plugin browser support.

---

## State machine

| State | Trigger | Description |
|---|---|---|
| `INITIAL_SCAN` | Power-on | 3-level raster acquisition (MDPI Drones 2024) |
| `TRACKING` | Person detected | Gimbal PID centring |
| `PREDICTING` | Lost 0–3 s | Full-speed velocity extrapolation |
| `PRED_FADE` | Lost 3–6 s | Tapering prediction (down to 30 %) |
| `SEARCHING` | Lost ≥ 6 s | Velocity-biased sector scan |
| `EXPANDING_SQUARE` | Sector timeout (15 s) | IAMSAR expanding square |
| `LISSAJOUS` | Expand timeout (45 s) | Sinusoidal fill (T_yaw/T_pitch ≈ √2/√3) |
| `WAITING` | Tracking disabled | Idle |

---

## Safety stack

Every safety rule is enforced before any MAVLink command leaves the Jetson.

| Layer | Rule |
|---|---|
| **Altitude** | All commands clamped to `[MIN_ALT_M, MAX_ALT_M] = [10, 80] m` |
| **Speed** | Horizontal velocity ≤ `MAX_TRACKING_SPEED_MS = 5 m/s`, vertical ≤ 2.5 m/s |
| **Acceleration** | `MAX_ACCEL_MS2 = 2.0 m/s²` after jerk limiter |
| **Geofence (app)** | 500 m circular, checked on both current position AND commanded target |
| **Geofence (FCU)** | Verified at preflight: `FENCE_ENABLE=1`, `FENCE_RADIUS ≥ app radius`, `FENCE_ALT_MAX ≥ ceiling` |
| **HOME keep-out** | 5 m no-fly cylinder around launch point |
| **MAVLink watchdog** | Heartbeat stale > 2.0 s → stop. Warn at 1.0 s |
| **GPS quality** | Fix ≥ 3D, HDOP ≤ 1.5, sats ≥ 10, EKF horizontal variance ≤ 1.0 |
| **EKF jump reject** | Measurements > 10 m from current state discarded |
| **Battery critical** | Per-cell V < 3.5 → RTL with retry. Cell count override via `--cells` |
| **Body confirm** | 5 consecutive detections required before drone body moves |
| **FPS floor** | Body holds if effective YOLO FPS < 8 |
| **Session timer** | Auto RTL after `MAX_FLIGHT_TIME_S = 600 s` |
| **Retreat + hysteresis** | Active 1 m/s retreat when sep < 4 m, resume above 5.5 m |
| **RC override** | GUIDED → other mode latches. Cleared only by re-arm after preflight |
| **E-STOP** | Web button + Space key. BRAKE → LAND escalation |

### Tracking-loss failsafe ladder

```
0–2 s   → EKF prediction, drone keeps velocity
2–5 s   → Zero velocity (decelerate)
5–15 s  → LOITER command (FCU holds)
> 15 s  → Stay in LOITER. Operator decides. Never auto-RTL on tracking loss alone
```

---

## ArduPilot parameters

Required on the flight controller. Verified at preflight; arming refused if any are wrong.

```
GUID_TIMEOUT     > 0       # GUIDED-mode command-loss timeout
FENCE_ENABLE     = 1       # Onboard geofence ON
FENCE_RADIUS    >= 500     # ≥ app GEOFENCE_RADIUS_M
FENCE_ALT_MAX   >= 80      # ≥ app MAX_ALT_M
RTL_ALT         >= 2000    # cm, i.e. ≥ 20 m
BATT_FS_LOW_ACT >= 2       # 2 = RTL, 3 = LAND
FS_GCS_ENABLE    = 1       # GCS-heartbeat failsafe ON
```

Recommended:

```
WPNAV_SPEED   = 500     # cm/s = 5 m/s
WPNAV_ACCEL   = 150     # cm/s²
PSC_JERK_XY   = 3.0     # m/s³
FENCE_ALT_MIN = 10
```

---

## Key configuration (`config/config.py`)

| Parameter | Default | Description |
|---|---|---|
| `MODEL_PATH` | `models/yolo26s.engine` | YOLO engine path |
| `CONF_THRESHOLD` | `0.45` | Detection confidence |
| `FOLLOW_ALTITUDE_M` | `12 m` | Target AGL altitude |
| `FOLLOW_STANDOFF_M` | `8 m` | Horizontal offset from subject |
| `MIN_PERSON_DRONE_SEP_M` | `4 m` | Retreat trigger |
| `RETREAT_HYSTERESIS_M` | `1.5 m` | Re-engage buffer |
| `RETREAT_SPEED_MS` | `1.0 m/s` | Retreat velocity |
| `BODY_MOVE_CONFIRM_FRAMES` | `5` | Frames before body moves |
| `MIN_TRACKING_FPS` | `8.0` | FPS floor for body motion |
| `MAX_FLIGHT_TIME_S` | `600 s` | Auto-RTL deadline |
| `HEARTBEAT_WATCHDOG_S` | `2.0 s` | MAVLink HB timeout |
| `BEARING_LATCH_S` | `1.0 s` | Sustained motion required to switch standoff bearing |
| `EKF_MAX_JUMP_M` | `10 m` | Measurement-rejection threshold |
| `STREAM_PORT` | `8080` | MJPEG HTTP port |
| `STREAM_MAX_FPS` | `25` | Stream FPS cap |
| `AUTO_RECORD` | `True` | Auto-record on launch |

---

## Logs & recording

| Output | Path |
|---|---|
| Session stdout/stderr (tee) | `logs/gimbal_track_YYYY-MM-DD_HH-MM-SS.log` |
| Structured flight events (JSONL) | `logs/flight_YYYYMMDD_HHMMSS.jsonl` |
| Video recording (H.264 MP4) | `recordings/track_YYYY-MM-DD_HH-MM-SS_WxH.mp4` |

Events emitted to the JSONL log: `arm`, `disarm`, `estop`, `mode_change`, `rc_override`, `session_rtl`, `fps_floor`, `safety_warning`, `log_open`.

---

## Tailscale remote operation

```bash
sudo tailscale up
tailscale ip -4              # note the IP

# Launch with token + LAN exposure
python3 main.py --drone --stream-host 0.0.0.0 --stream-token MYSECRET

# From any device on the tailnet:
#   http://<tailscale-ip>:8080/?token=MYSECRET
```

---

## Tests

```bash
pytest -q
# 170 passed
```

The suite covers safety (`test_safety.py`, `test_preflight.py`, `test_param_verifier.py`,
`test_home_keepout.py`, `test_retreat.py`, `test_body_confirm.py`,
`test_ekf_jump.py`, `test_accel_cap.py`, `test_bearing_hysteresis.py`,
`test_gps_quality.py`, `test_estop.py`, `test_rc_override.py`,
`test_fps_floor.py`, `test_session_rtl.py`, `test_cells_override.py`,
`test_ground_test_mode.py`, `test_flight_log.py`, `test_status_telemetry.py`),
tracking (`test_target_detection.py`, `test_target_selector.py`,
`test_tracker_web_click.py`, `test_standoff.py`, `test_drone_target_geofence.py`),
control (`test_pid.py`, `test_mavlink_ack.py`), and web UI (`test_stream_token_html.py`, `test_query_parse.py`).

---

## Project layout

```
person_tracking/
├── main.py                         Entry point — CLI args, banner, log setup
├── tracker.py                      Orchestrator — state machine + main loop
├── config/
│   ├── config.py                   Tunable parameters
│   └── settings.py                 Immutable Settings dataclass
├── detection/
│   └── detector.py                 YOLO TensorRT + ByteTrack
├── gimbal/
│   ├── siyi_controller.py          SIYI SDK v2.0 over UDP
│   └── auto_zoom.py                Optional auto-zoom
├── control/
│   ├── pid_controller.py           Discrete PID + EMA centroid smoother
│   ├── search_patterns.py          4-stage search escalation
│   └── drone_controller.py         Outer loop — EKF + safety + retreat
├── tracking/
│   ├── state_machine.py            8-state FSM
│   ├── person_registry.py          HSV histogram re-ID
│   ├── velocity_tracker.py         Recency-weighted velocity
│   ├── person_geolocation.py       Pixel → GPS + 4-state EKF
│   ├── tracker_state.py            Shared mutable state
│   ├── target_selector.py          Target selection (lock / largest)
│   ├── target_detection.py         TargetDetection dataclass
│   ├── gimbal_state_machine.py     Gimbal FSM
│   └── operator_input.py           Keyboard / terminal handler
├── mavlink_client/
│   ├── mavlink_client.py           Thread-safe pymavlink wrapper
│   └── param_verifier.py           FCU parameter verifier (S2.3)
├── safety/
│   ├── safety.py                   Altitude, speed, geofence, watchdog, battery, keep-out
│   └── preflight.py                Preflight checklist (S1.3)
├── gcs/
│   ├── stream_server.py            MJPEG HTTP + click-to-track web UI + E-STOP
│   └── web_control_adapter.py      Web callback handlers
├── mission/
│   ├── hud_renderer.py             On-screen HUD
│   ├── recording_manager.py        Recording lifecycle
│   └── stream_adapter.py           Stream-loop adapter
├── utils/
│   ├── frame_grabber.py            RTSP capture (NVDEC priority)
│   ├── video_recorder.py           H.264 record (NVENC priority)
│   ├── flight_log.py               JSONL structured event log (S3.2)
│   └── logger.py                   stdout tee
├── tests/                          170 pytest tests
└── models/
    └── yolo26s.engine              YOLOv8 TensorRT engine
```

---

## Shutdown

```
Ctrl-C       (in the terminal running main.py)
   — or —
q + Enter    (terminal mode)
   — or —
q key        (display mode)
```

Session log + flight log + video recording are all closed cleanly on exit.
