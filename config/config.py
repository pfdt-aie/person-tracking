"""
config.py — Central configuration for the person-tracking drone system.

All tunable parameters live here. No magic numbers elsewhere.
Hardware: Jetson Orin Nano Super + Orange Cube+ (ArduPilot) + SIYI A8 Mini.
"""

# =============================================================================
#  CAMERA / NETWORK
# =============================================================================

CAMERA_IP: str   = "192.168.144.25"
GIMBAL_PORT: int = 37260
RTSP_URL: str    = "rtsp://192.168.144.25:8554/main.264"

MODEL_PATH: str       = "models/yolo26s.engine"
CONF_THRESHOLD: float = 0.30   # alternating detect/miss was fixed by BODY_CONFIRM_WINDOW_S; 0.25 increased false positives unnecessarily
IMGSZ: int            = 640
DETECT_DEVICE: str    = "auto"   # "auto" | "cpu" | "cuda:0" | "0"

# =============================================================================
#  RE-IDENTIFICATION  (PersonRegistry — HSV histogram, CPU)
# =============================================================================

REID_ENABLED: bool            = True    # False = disable PersonRegistry re-ID entirely

# =============================================================================
#  PID — GIMBAL (base values; adaptive Kp overrides these at runtime)
# =============================================================================

PID_YAW_KP: float   = 12.0
PID_YAW_KI: float   = 0.15
PID_YAW_KD: float   = 5.0
PID_PITCH_KP: float = 12.0
PID_PITCH_KI: float = 0.15
PID_PITCH_KD: float = 5.0

# --- Adaptive speed ---
ADAPT_LOW: float        = 0.15
ADAPT_HIGH: float       = 0.60
ADAPT_KP_MIN: float     = 12.0    # reduced to prevent oscillation
ADAPT_KP_MAX: float     = 30.0    # reduced to prevent oscillation
ADAPT_SPEED_MIN: int    = 35
ADAPT_SPEED_MAX: int    = 65

DEAD_ZONE: float             = 0.06       # stops chasing sub-pixel jitter
MAX_GIMBAL_SPEED: int        = 35         # limits overshoot
GIMBAL_MIN_SPEED: int        = 8          # below this the gimbal stalls
TARGET_SMOOTH_ALPHA: float   = 0.65       # ~120 ms lag at 25 fps
GIMBAL_CMD_MIN_INTERVAL: float = 0.05
MANUAL_GIMBAL_SPEED: int       = 40     # gimbal pan/tilt speed in MANUAL mode (0–100)

# =============================================================================
#  PREDICTION
# =============================================================================

PREDICT_DURATION: float      = 3.0    # Full-speed prediction phase (s)
PRED_FADE_DURATION: float    = 3.0    # Fading prediction phase (s)
PRED_FADE_MIN_FACTOR: float  = 0.30   # Minimum speed factor at end of fade
VELOCITY_WINDOW: float       = 1.0    # Rolling window for velocity calc (s)
VELOCITY_MIN_SAMPLES: int    = 5      # Minimum samples for valid velocity
EDGE_EXIT_BOOST: float       = 1.5    # Speed multiplier when person exits frame edge
EDGE_MARGIN_RATIO: float     = 0.05   # Bbox within 5% of frame edge = edge exit

# =============================================================================
#  SECTOR SCAN SEARCH
# =============================================================================

SEARCH_ENABLED: bool            = True
# SEARCH_SPEED in SIYI units (0–100); GIMBAL_SPEED_FULL_SCALE_DEG_S = 60°/s at 100.
# At speed=18 → 10.8°/s × 3.0s sweep = ±32° arc (was HALF the intended ±60°).
# At speed=33 → 19.8°/s × 3.0s = ±59° ≈ the documented SEARCH_PHASE1_ARC=120°.
SEARCH_SPEED: int               = 33
SEARCH_PHASE1_ARC: int          = 120
SEARCH_PHASE2_ARC: int          = 180
SEARCH_PHASE1_SWEEPS: int       = 2
SEARCH_PHASE2_SWEEPS: int       = 2
SEARCH_SWEEP_DURATION: float    = 3.0
SEARCH_PITCH_SCAN_SPEED: int    = 10
SEARCH_PITCH_SCAN_DURATION: float = 2.0
SEARCH_FALLBACK_SPEED: int      = 40   # was 25 → 24°/s for ±72° fallback arc

# =============================================================================
#  ZOOM (A8 mini digital zoom)
# =============================================================================

AUTO_ZOOM_ENABLED: bool          = False
ZOOM_MAX: float                  = 6.0
ZOOM_TARGET_HEIGHT_RATIO: float  = 0.40
ZOOM_TOLERANCE: float            = 0.10
ZOOM_CMD_INTERVAL: float         = 0.3

# =============================================================================
#  MULTI-PERSON / DEBOUNCE
# =============================================================================

LOST_CONFIRM_FRAMES: int    = 6    # Consecutive missed frames before "lost" (~200ms at 30fps)
MIN_VELOCITY_PREDICT: float = 0.05 # vel below this → skip PREDICTING
ID_REPORT_INTERVAL: float   = 2.0  # Seconds between ID prints (headless)
LOCK_TARGET_GRACE_S: float  = 0.4  # Keep a locked target through brief detector dropouts (~10 frames at 25fps); 1.5s was too long — held stale position for a walking target
LOCK_REACQUIRE_CENTER_RATIO: float = 0.25  # Max centre jump as fraction of frame diagonal
LOCK_REACQUIRE_STRICT_CENTER_RATIO: float = 0.08  # Allow no-overlap reacquire only very nearby
LOCK_REACQUIRE_MIN_IOU: float = 0.05  # Otherwise require some bbox overlap before remapping lock

# =============================================================================
#  PERSON RE-IDENTIFICATION  (PersonRegistry thresholds)
# =============================================================================

REID_SIM_THRESHOLD: float = 0.80   # cosine similarity to consider "same person"
REID_KNOWN_ID_MIN_SIM: float = 0.75 # reject recycled ByteTrack IDs below this appearance match
REID_GALLERY_TTL: float   = 300.0  # seconds before an unseen person is forgotten

# =============================================================================
#  BATTERY DISPLAY THRESHOLDS
# =============================================================================
# Calibrated for 4S LiPo (16.8V full, ~13.2V storage, 13.6V RTL threshold)

BATT_DISPLAY_MIN_V: float = 13.2   # 0% on battery bar — 4S storage voltage
BATT_DISPLAY_MAX_V: float = 16.8   # 100% on battery bar — 4S full charge
BATT_WARN_V: float        = 14.4   # green → yellow (3.6V/cell — plan to land)
BATT_CRIT_V: float        = 13.6   # yellow → red (3.4V/cell — RTL imminent)

BATTERY_POLL_INTERVAL: float  = 5.0    # send battery-request packet every N seconds (gimbal)
BATTERY_PRINT_INTERVAL: float = 30.0   # print battery info to terminal every N seconds

# =============================================================================
#  GIMBAL UDP WATCHDOG
# =============================================================================

GIMBAL_WATCHDOG_S: float = 5.0    # warn if no UDP response received within this window

# =============================================================================
#  VIDEO RECORDING
# =============================================================================

AUTO_RECORD: bool      = True          # start recording immediately on launch
RECORD_OVERLAY: bool   = True          # draw tracking overlay on recorded video
RECORD_FPS: float      = 30.0          # nominal recording frame rate
RECORD_BITRATE: int    = 20_000_000    # H.264 encoder bitrate (bps)

# =============================================================================
#  LIVE STREAMING (MJPEG over HTTP — Tailscale / LAN)
# =============================================================================

STREAM_ENABLED: bool   = True         # False = disable HTTP server entirely
STREAM_PORT: int       = 5000         # HTTP port (Tailscale Serve at https://ai2-desktop.tailca9f5b.ts.net/ proxies → 127.0.0.1:5000)
STREAM_MAX_FPS: int    = 25           # cap stream FPS
STREAM_HOST: str       = "0.0.0.0"    # bind address; default opens to LAN/Tailscale (always pair with STREAM_TOKEN)
STREAM_TOKEN: str      = ""           # if non-empty, required on all control endpoints

STREAM_QUALITY_HI: int = 65      # JPEG quality for 720p stream
STREAM_QUALITY_LO: int = 55      # JPEG quality for 480p stream

# =============================================================================
#  SEARCH ALGORITHM CONSTANTS  (Research-based, v9)
# =============================================================================
# Reference: Scientific Reports 2024, IAMSAR, MDPI Drones 2024

# --- Initial Acquisition Scan (3-level raster, MDPI Drones 2024) ---
INIT_SCAN_SPEED: int              = 20   # was 12 → 7.2°/s; now 20 → 12°/s for faster acquisition
INIT_SCAN_SWEEP_TIME: float       = 4.5
INIT_SCAN_SWEEPS_PER_LEVEL: int   = 2
INIT_SCAN_PITCH_STEP_TIME: float  = 1.2
INIT_SCAN_PITCH_RETURN_TIME: float = 2.4  # = (PHASES-1)*STEP_TIME — returns exactly to level-0
INIT_SCAN_PITCH_SPEED: int        = 10
INIT_SCAN_PRETILT_TIME: float     = 1.0   # tilt down before raster so level-0 is in ground zone

# --- Expanding Square Search (IAMSAR standard) ---
EXP_SQUARE_SPEED: int       = 12
EXP_SQUARE_PITCH_SPEED: int = 8
EXP_SQUARE_BASE_TIME: float = 4.0
EXP_SQUARE_MAX_ARMS: int    = 8

# --- Search phase timeouts ---
SECTOR_SEARCH_TIMEOUT: float = 30.0   # was 15s — at new speed, full 3-phase scan takes ~25s
EXPAND_SEARCH_TIMEOUT: float = 45.0   # s in expanding square before → Lissajous

# --- Search pitch envelope ---
# SIYI tilt convention: negative = looking down. Search patterns may move
# between these ground-looking bounds, but the FSM clamps commands that would
# drive above the shallow bound or deeper than the steep bound.
SEARCH_PITCH_SHALLOW_DEG: float = -12.0
SEARCH_PITCH_STEEP_DEG: float   = -80.0
SEARCH_RECENTER_PITCH_SPEED: int = 8
SEARCH_RECENTER_GROUND_TIMEOUT_S: float = 3.0

# --- Lissajous Long-Duration Search (Scientific Reports 2024) ---
LISSAJOUS_YAW_SPEED: int       = 14
LISSAJOUS_PITCH_SPEED: int     = 8
LISSAJOUS_YAW_PERIOD: float    = 40.0   # T_yaw (s)
LISSAJOUS_PITCH_PERIOD: float  = 49.0   # T_pitch — ratio 40/49 ≈ √2/√3
LISSAJOUS_RECENTER_PERIOD: float = 120.0
LISSAJOUS_RECENTER_DWELL: float  = 2.0

# =============================================================================
#  DRONE BODY CONTROL  (NEW — requires --drone flag)
# =============================================================================

# --- MAVLink connection ---
MAVLINK_DEVICE: str  = "/dev/ttyTHS1" # Tegra UART → Cube TELEM1/2 (override with --device for USB CDC e.g. /dev/ttyACM0)
MAVLINK_BAUD: int    = 921600

# --- Following geometry ---
FOLLOW_ALTITUDE_M: float      = 7.0    # Target AGL altitude to maintain (m)
FOLLOW_STANDOFF_M: float      = 7.0    # Horizontal distance to hold behind/away from person (m)
MIN_PERSON_DRONE_SEP_M: float = 4.0    # Hard minimum separation — retreat if closer (m)
MIN_VERTICAL_SEP_M: float     = 3.0    # Soft vertical clearance above person (m)
RETREAT_SPEED_MS: float       = 1.0    # Speed used when actively backing away from subject
RETREAT_HYSTERESIS_M: float   = 1.5    # Re-engage follow only when sep > MIN_SEP + this (m)
STANDOFF_VEL_THRESHOLD_MS: float = 0.6 # Use person velocity direction above this speed (m/s) — raised from 0.3: with Q_vel=0.3 the EKF velocity noise magnitude is ~0.36 m/s; threshold must be >noise so position-based bearing is used at hover
MAX_TRACKING_SPEED_MS: float  = 3.0    # Hard velocity cap (m/s) — was 1.5; at 1.5 drone can't catch a walking person (1.4 m/s)

# --- Standoff bearing hysteresis (S2.6) ---
# Person must sustain motion above STANDOFF_VEL_THRESHOLD_MS for this long
# before bearing source flips from position-based (drone→person line) to
# velocity-based (person's heading). Prevents flicker on brief stops.
BEARING_LATCH_S: float        = 1.0
# Maximum rate of change of the standoff bearing (degrees per second).
# Smooths transitions so the target NED point cannot teleport.
BEARING_SLEW_DEG_S: float     = 30.0
MIN_ALT_M: float             = 5.0   # Absolute altitude floor (m AGL) — SAFETY-CRITICAL
MAX_ALT_M: float             = 80.0  # Altitude ceiling (m AGL)

# --- Operator takeoff command ---
# Used by the `takeoff [alt]` terminal command and `/takeoff?alt=N` HTTP
# endpoint. The handler refuses an altitude outside [MIN, MAX]; values
# below MIN_ALT_M are allowed for hover testing but log a warning since
# the follow controller's floor will push the drone up to MIN_ALT_M
# the moment the tracker is armed.
DEFAULT_TAKEOFF_ALT_M: float = 7.0   # Default ascent target for `takeoff` (m AGL)
MIN_TAKEOFF_ALT_M: float     = 1.0   # Reject takeoff requests below this (m)
MAX_TAKEOFF_ALT_M: float     = 30.0  # Reject takeoff requests above this (m) — well under MAX_ALT_M
GEOFENCE_RADIUS_M: float     = 500.0 # Circular geofence radius around home (m)
HOME_KEEPOUT_RADIUS_M: float = 5.0   # No-fly cylinder around HOME (operator stands here)

# --- Proportional controller ---
DRONE_KP: float     = 0.4    # Position error → velocity (m/s per meter of error)
DRONE_KP_YAW: float = 0.6    # Gimbal pan angle → drone yaw rate (rad/s per degree)

# --- Velocity smoothing ---
VEL_EMA_ALPHA: float  = 0.35    # EMA filter factor — raised for faster response
MAX_JERK_MS3: float   = 5.0     # Jerk limit (m/s³) — was 2.0, raised for faster ramp
MAX_ACCEL_MS2: float  = 1.5     # Hard acceleration cap (m/s²) — reduced from 2.0 for smoother motion; 1.5 still ramps to 3 m/s in 2s, faster than a walking person

# --- MAVLink send rate ---
DRONE_CMD_RATE_HZ: int   = 10   # Velocity command send rate

# --- Failsafe timings ---
TRACKING_LOSS_HOVER_S: float  = 2.0    # Seconds before sending zero velocity
TRACKING_LOSS_LOITER_S: float = 10.0   # Seconds before issuing LOITER — was 5s, too short for intermittent YOLO
TRACKING_LOSS_ALERT_S: float  = 20.0   # Seconds before GCS terminal alert

# --- Detection confirmation before drone body movement ---
# Gimbal tracks immediately; drone body stays at zero velocity until this many
# consecutive frames have a valid detection. Prevents false-positive (bush/dog)
# from yanking the airframe.
BODY_MOVE_CONFIRM_FRAMES: int = 3    # kept for tests; replaced by time-based confirm
BODY_CONFIRM_WINDOW_S: float = 0.8  # drone body moves if person seen within this window (s)

# --- Latency floor for drone body control (S3.5) ---
# When effective detection FPS drops below this, the drone body holds (sends
# zero velocity) so stale frames cannot drive a position command. Gimbal is
# unaffected.
MIN_TRACKING_FPS: float = 5.0    # was 8.0 — on loaded Jetson (FFmpeg+recording+streaming) YOLO can drop below 8 fps
# Number of recent detection ticks averaged when estimating FPS.
FPS_WINDOW_SIZE: int    = 20

# --- Hybrid gimbal/drone thresholds ---
GIMBAL_PAN_SOFT_DEG: float  = 60.0    # Drone starts rotating toward person
GIMBAL_PAN_HARD_DEG: float  = 120.0   # Drone rotates aggressively

# --- Safety watchdog ---
# ArduPilot emits HEARTBEAT at 1 Hz. WATCHDOG must allow at least one missed
# packet to avoid false-firing on normal jitter; 2.0 s = 1 missed HB allowed.
# At MAX_TRACKING_SPEED_MS=5, 2.0 s × 5 = 10 m drift before commands stop.
HEARTBEAT_WATCHDOG_S: float  = 2.0    # Max seconds without MAVLink heartbeat
HEARTBEAT_WARN_S: float      = 1.0    # Early warning threshold (between HBs)
GPS_MIN_FIX_TYPE: int        = 3      # Minimum GPS fix (3 = 3D fix)
RC_WATCHDOG_S: float         = 2.0    # Max seconds since last RC_CHANNELS message
RC_MIN_CHANNELS: int         = 4      # Minimum populated channels for "link healthy"
TELEMETRY_STALE_S: float     = 3.0    # Max age for required FCU telemetry streams
GPS_MAX_HDOP: float          = 1.5    # Reject if HDOP > this
GPS_MIN_SATS: int            = 10     # Reject if visible sats < this
EKF_MAX_VARIANCE: float      = 1.0    # Reject if FCU EKF horizontal variance > this

# --- Battery ---
DEFAULT_CELLS: int           = 4      # Fallback if auto-detection fails
CELL_NOMINAL_MV: int         = 3700   # Nominal cell voltage for cell-count detection
CELL_WARN_MV: int            = 3600   # Per-cell warning voltage → alert operator (14.4V on 4S)
CELL_CRITICAL_MV: int        = 3500   # Per-cell critical voltage → trigger RTL (14.0V on 4S)

# --- SIYI attitude telemetry ---
GIMBAL_ATTITUDE_POLL_HZ: float = 10.0   # Request attitude at 10 Hz
GIMBAL_ATTITUDE_STALE_S: float = 0.2    # Use commanded angles if telemetry older than this
GIMBAL_COAST_S: float          = 0.5    # Suppress EKF update if telemetry stale beyond this
GIMBAL_SPEED_FULL_SCALE_DEG_S: float = 60.0  # Estimated deg/s at SIYI speed=100
GIMBAL_CMD_EST_MAX_DT_S: float = 0.25  # Cap one integration step after scheduler stalls
GIMBAL_PAN_MIN_DEG: float = -160.0
GIMBAL_PAN_MAX_DEG: float = 160.0
GIMBAL_TILT_MIN_DEG: float = -135.0
GIMBAL_TILT_MAX_DEG: float = 45.0

# =============================================================================
#  EXTENDED KALMAN FILTER  (person geolocation)
# =============================================================================

EKF_PROCESS_NOISE: list = [0.2, 0.2, 0.3, 0.3]   # Q diag — velocity reduced from 1.5→0.3: Q_vel=1.5 caused ±0.52 m/s feedforward noise at hover (EKF velocity exceeded STANDOFF_VEL_THRESHOLD 74% of time at standstill, causing bearing oscillation)
EKF_MEAS_NOISE:    list = [2.0, 2.0]              # R diag: [pN, pE] (m²)
EKF_GATE_SIGMA:    float = 5.0                    # Mahalanobis gate — was 3.0; at 3σ gate_sq=9.0 rejected borderline measurements; 5σ is more appropriate for noisy GPS
EKF_MAX_JUMP_M:    float = 60.0                   # Jump rejection — raised from 10→30→50→60m for steep-angle close-range projections

# =============================================================================
#  CAMERA INTRINSICS
# =============================================================================
# SIYI A8 mini: HFOV = 81° at 1× zoom (published spec).
# If a calibration YAML is provided via --calibration, those values override.
# Otherwise, intrinsics are auto-estimated from HFOV and frame resolution.

SIYI_A8_HFOV_DEG: float = 81.0    # Published horizontal FOV at 1× zoom
CALIBRATION_YAML: str   = ""       # Path to calibration file; "" = auto-estimate

# --- EKF projection fallbacks ---
GIMBAL_TILT_DEFAULT_DEG: float = -45.0   # Used if real gimbal tilt not yet known
DRONE_FOLLOW_DEADBAND_M: float = 1.5     # raised from 1.0: 1.0m deadband too close to GPS noise floor (~0.8m), causing 1-2 Hz in/out oscillation at hover
DRONE_MAX_YAW_RATE_DEG: float  = 20.0    # Max body-yaw rate during pan recenter

# --- Safety enhancements ---
GEOFENCE_RETURN_MAX_MS: float   = 2.5   # max speed when returning from geofence breach (m/s)
GEOFENCE_RETURN_KP: float       = 0.3   # proportional gain: error_m → return speed
RTL_MAX_ATTEMPTS: int           = 3     # max RTL command retries on battery critical
RTL_RETRY_INTERVAL_S: float     = 1.0   # seconds between RTL retries
MAX_FLIGHT_TIME_S: float        = 600.0 # Max armed-tracker session before auto-RTL (S3.3)
LOITER_CONFIRM_TIMEOUT_S: float = 2.0   # seconds to wait for LOITER mode confirmation
MODE_WARN_INTERVAL_S: float     = 5.0   # rate-limit for "not in GUIDED" log message

# =============================================================================
#  DISPLAY
# =============================================================================

DISPLAY_WIDTH: int  = 960
DISPLAY_HEIGHT: int = 540
