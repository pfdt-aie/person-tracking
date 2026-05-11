# Person Tracking Drone — v10.0-modular

Real-time autonomous person-following system for a quadcopter equipped with a SIYI A8 mini gimbal. Runs on Jetson Orin Nano Super / Orin NX (JetPack 6+) and communicates with an Orange Cube+ flight controller via MAVLink.

---

## Hardware

| Component | Details |
|---|---|
| **Compute** | Jetson Orin Nano Super / Orin NX (JetPack 6+) |
| **Gimbal** | SIYI A8 mini — 192.168.144.25 UDP :37260 |
| **Flight Controller** | Orange Cube+ (ArduPilot, GUIDED mode) |
| **Camera** | SIYI A8 mini built-in — RTSP `rtsp://192.168.144.25:8554/main.264` |
| **Battery** | 3S–4S LiPo |

---

## Architecture

The system runs a **two-loop hybrid control** architecture:

- **Inner loop — Gimbal (30 Hz):** PID controller centers the detected person in frame by sending yaw/pitch speed commands to the SIYI A8 mini over UDP.
- **Outer loop — Drone body (10 Hz):** When the `--drone` flag is set, a proportional controller translates gimbal pan/tilt error and GPS standoff distance into MAVLink velocity commands to the flight controller. Activates when gimbal pan exceeds 60°.

```
RTSP stream → FrameGrabber → YOLO + ByteTrack → PersonRegistry (re-ID)
                                                        ↓
                                               State Machine (8 states)
                                                   ↙        ↘
                                          PID Gimbal    DroneController
                                          (30 Hz)          (10 Hz)
                                              ↓                ↓
                                        SIYI UDP         MAVLink serial
```

---

## States

| State | Description |
|---|---|
| `INITIAL_SCAN` | Power-on 3-level raster acquisition scan |
| `TRACKING` | Person detected — gimbal actively centering |
| `PREDICTING` | Person lost — full-speed velocity extrapolation (0–3 s) |
| `PRED_FADE` | Fading prediction — speed tapers to 30% (3–6 s) |
| `SEARCHING` | Velocity-biased sector scan |
| `EXPANDING_SQUARE` | IAMSAR expanding square (after 15 s in sector scan) |
| `LISSAJOUS` | Sinusoidal fill search — runs indefinitely (after 45 s) |
| `WAITING` | Idle (tracking or search disabled) |

---

## Search Pattern Escalation

```
INITIAL_SCAN (power-on)
    ↓ person found → TRACKING
    ↓ tracking lost
PREDICTING (0–3 s)
    ↓
PRED_FADE (3–6 s)
    ↓
SEARCHING — sector scan, velocity-biased (≤ 15 s)
    ↓
EXPANDING_SQUARE — IAMSAR standard (≤ 45 s)
    ↓
LISSAJOUS — sinusoidal fill, indefinite
```

Search algorithms are based on published research:
- Initial scan: MDPI Drones 2024
- Expanding square: IAMSAR manual
- Lissajous: Scientific Reports 2024 (irrational period ratio T_yaw/T_pitch ≈ 40/49 ≈ √2/√3)

---

## Project Layout

```
person_tracking/
├── main.py                      Entry point, CLI args, banner, session log
├── tracker.py                   Central orchestrator — state machine + main loop
├── config/
│   └── config.py                All tunable parameters (no magic numbers elsewhere)
├── detection/
│   └── detector.py              YOLO TensorRT inference + ByteTrack target selection
├── gimbal/
│   ├── siyi_controller.py       Thread-safe UDP controller (SIYI SDK v2.0)
│   └── auto_zoom.py             Optional auto-zoom to keep person at target frame size
├── control/
│   ├── pid_controller.py        Discrete PID + TargetSmoother (EMA centroid filter)
│   ├── search_patterns.py       Four escalating search algorithms
│   └── drone_controller.py      Hybrid outer loop — EKF + proportional drone control
├── tracking/
│   ├── state_machine.py         8-state FSM definitions
│   ├── person_registry.py       HSV histogram re-identification (persistent P-IDs)
│   ├── velocity_tracker.py      Recency-weighted velocity estimator + edge-exit detection
│   └── person_geolocation.py   Pixel → GPS projection + 4-state EKF smoother
├── mavlink_client/
│   └── mavlink_client.py        Thread-safe pymavlink connection (Orange Cube+)
├── safety/
│   └── safety.py                Constraint enforcer: altitude, speed, geofence, watchdog, battery
├── gcs/
│   └── stream_server.py         MJPEG HTTP server with click-to-track web UI
├── utils/
│   ├── frame_grabber.py         Zero-latency threaded RTSP capture (NVDEC priority)
│   ├── video_recorder.py        H.264 recording (NVENC priority)
│   └── logger.py                Session log (tee stdout/stderr to timestamped file)
└── models/
    └── yolo26s.engine           YOLOv8 TensorRT engine (person class)
```

---

## Installation

```bash
# Python environment (JetPack 6 ships Python 3.10)
pip install ultralytics opencv-python numpy

# Optional — drone body control
pip install pymavlink

# GStreamer hardware decode/encode (already included in JetPack 6)
# gstreamer1.0-plugins-bad gstreamer1.0-plugins-good nvv4l2decoder nvv4l2h264enc
```

---

## Running

```bash
cd person_tracking

# Gimbal-only mode (default)
python3 main.py

# Full drone body control
python3 main.py --drone

# Custom serial port / baud
python3 main.py --drone --device /dev/ttyTHS1 --baud 115200

# Custom YOLO model
python3 main.py --model models/best.pt

# With camera calibration file
python3 main.py --calibration /path/to/calibration.yaml
```

---

## Controls

### Keyboard (display window open)

| Key | Action |
|---|---|
| `q` | Quit |
| `t` | Toggle tracking on/off |
| `r` | Center gimbal + reset zoom to 1× |
| `s` | Toggle search on/off |
| `i` | Restart initial acquisition scan |
| `d` | Toggle display window |
| `z` / `x` | Zoom in / out |
| `a` | Toggle auto-zoom |
| `v` | Toggle video recording |
| `l` | Toggle live stream |
| `+` / `-` | Increase / decrease adaptive Kp gain |
| `[` / `]` | Adjust max gimbal speed |

### Terminal / SSH (headless)

```
track <id>   Lock onto persistent person ID
unlock       Release lock, revert to largest-person mode
ids          Print all detected IDs + positions
q            Quit
```

### Web UI

Browse to `http://<jetson-ip>:8080/` (works over Tailscale).

- Click on a person to lock tracking to that person
- Click empty space to unlock
- Zoom in / out buttons

**Direct endpoints:**

| Endpoint | Description |
|---|---|
| `/stream` | Raw MJPEG (VLC / `<img src>`) |
| `/click?x=0.42&y=0.61` | Lock to normalised coordinates |
| `/unlock` | Release lock |
| `/status` | JSON `{lock_id, ids}` |
| `/zoom_in` / `/zoom_out` | Gimbal zoom |

---

## Key Configuration (`config/config.py`)

| Parameter | Default | Description |
|---|---|---|
| `MODEL_PATH` | `models/yolo26s.engine` | YOLO TensorRT engine path |
| `CONF_THRESHOLD` | `0.45` | Detection confidence threshold |
| `ADAPT_KP_MIN/MAX` | `12.0 / 30.0` | Adaptive PID gain range |
| `ADAPT_SPEED_MIN/MAX` | `35 / 65` | Gimbal speed range |
| `PREDICT_DURATION` | `3.0 s` | Full-speed velocity prediction after target loss |
| `PRED_FADE_DURATION` | `3.0 s` | Fading prediction phase duration |
| `EDGE_EXIT_BOOST` | `1.5×` | Speed multiplier when person exits frame edge |
| `SECTOR_SEARCH_TIMEOUT` | `15 s` | Escalate to expanding square after this |
| `EXPAND_SEARCH_TIMEOUT` | `45 s` | Escalate to Lissajous after this |
| `AUTO_ZOOM_ENABLED` | `False` | Auto-adjust zoom to keep person at 40% frame height |
| `FOLLOW_ALTITUDE_M` | `12 m` | Target AGL altitude (drone mode) |
| `FOLLOW_STANDOFF_M` | `8.0 m` | Horizontal standoff from person (drone mode) |
| `MAX_TRACKING_SPEED_MS` | `5.0 m/s` | Hard velocity cap — safety critical |
| `MIN_ALT_M / MAX_ALT_M` | `10 / 80 m` | Altitude floor/ceiling — safety critical |
| `STREAM_PORT` | `8080` | MJPEG HTTP server port |
| `AUTO_RECORD` | `True` | Start recording automatically on launch |
| `BATT_WARN_V / BATT_CRIT_V` | `11.5 / 10.5 V` | Battery warning thresholds |

---

## Safety System

Five constraints are **never bypassed**:

1. **Altitude floor/ceiling** — all commands clamped to `[MIN_ALT_M, MAX_ALT_M]`
2. **Speed cap** — horizontal velocity hard-limited to `MAX_TRACKING_SPEED_MS` (5 m/s)
3. **Geofence** — 500 m circular boundary; outside → zero velocity
4. **MAVLink watchdog** — heartbeat stale → stop sending commands
5. **Battery critical** — per-cell voltage < 3.5 V → RTL

**Tracking loss failsafe (drone mode):**

```
0–2 s   → EKF prediction, drone holds velocity
2–5 s   → Zero velocity (drone decelerates)
5–15 s  → LOITER command
> 15 s  → Stay in LOITER (no auto-RTL on tracking loss alone)
```

---

## ArduPilot Parameters

Set these on the flight controller before enabling drone mode:

```
GUID_TIMEOUT   = 5       # stop if no velocity command for 5 s
WPNAV_SPEED    = 500     # cm/s (5 m/s)
WPNAV_ACCEL    = 150     # cm/s²
PSC_JERK_XY    = 3.0     # m/s³
FENCE_ENABLE   = 1
FENCE_ALT_MIN  = 10      # m AGL
```

---

## Person Re-Identification

The `PersonRegistry` assigns stable **P-IDs** that persist across the full session, even when a person temporarily leaves frame and re-enters.

- Each detection crop is hashed to a 48-bin HSV colour histogram
- New detections are matched against the gallery via cosine similarity (threshold 0.80)
- Gallery entries not seen for 300 s are pruned

Re-ID runs fully on CPU using `numpy` histogram comparisons — no separate model required.

---

## Video & Streaming

**Recording:** Auto-starts on launch. Frames are encoded with H.264:
- Hardware path: Jetson NVENC via GStreamer (`nvv4l2h264enc`) — ~5% CPU at 1080p@30
- Software fallback: `x264enc`, then OpenCV `mp4v`

Files saved to `recordings/track_YYYY-MM-DD_HH-MM-SS_WxH.mp4`.

**Live stream:** MJPEG over HTTP on port 8080. The web UI uses boundary-based MJPEG parsing in JavaScript — frames are dropped if the browser is busy rendering, preventing lag accumulation.

**Tailscale remote access:**
```bash
sudo tailscale up
tailscale ip -4   # note the IP
# browse to http://<tailscale-ip>:8080/ from anywhere
```

---

## Logs

Session logs are saved to `logs/gimbal_track_YYYY-MM-DD_HH-MM-SS.log`. All `print()` output is tee'd to both the terminal and the log file automatically.
