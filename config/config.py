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
CONF_THRESHOLD: float = 0.45
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
SEARCH_SPEED: int               = 18
SEARCH_PHASE1_ARC: int          = 120
SEARCH_PHASE2_ARC: int          = 180
SEARCH_PHASE1_SWEEPS: int       = 2
SEARCH_PHASE2_SWEEPS: int       = 2
SEARCH_SWEEP_DURATION: float    = 3.0
SEARCH_PITCH_SCAN_SPEED: int    = 10
SEARCH_PITCH_SCAN_DURATION: float = 2.0
SEARCH_FALLBACK_SPEED: int      = 25

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

LOST_CONFIRM_FRAMES: int    = 3    # Consecutive missed frames before "lost"
MIN_VELOCITY_PREDICT: float = 0.05 # vel below this → skip PREDICTING
ID_REPORT_INTERVAL: float   = 2.0  # Seconds between ID prints (headless)

# =============================================================================
#  PERSON RE-IDENTIFICATION  (PersonRegistry thresholds)
# =============================================================================

REID_SIM_THRESHOLD: float = 0.80   # cosine similarity to consider "same person"
REID_GALLERY_TTL: float   = 300.0  # seconds before an unseen person is forgotten

# =============================================================================
#  BATTERY DISPLAY THRESHOLDS
# =============================================================================
# Adjust per pack: 3S=9.9–12.6 V, 4S=13.2–16.8 V

BATT_DISPLAY_MIN_V: float = 10.0   # 0% on battery bar (Volts)
BATT_DISPLAY_MAX_V: float = 13.0   # 100% on battery bar
BATT_WARN_V: float        = 11.5   # green → yellow threshold
BATT_CRIT_V: float        = 10.5   # yellow → red threshold

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
INIT_SCAN_SPEED: int              = 12
INIT_SCAN_SWEEP_TIME: float       = 4.5
INIT_SCAN_SWEEPS_PER_LEVEL: int   = 2
INIT_SCAN_PITCH_STEP_TIME: float  = 1.2
INIT_SCAN_PITCH_RETURN_TIME: float = 2.8
INIT_SCAN_PITCH_SPEED: int        = 10

# --- Expanding Square Search (IAMSAR standard) ---
EXP_SQUARE_SPEED: int       = 12
EXP_SQUARE_PITCH_SPEED: int = 8
EXP_SQUARE_BASE_TIME: float = 4.0
EXP_SQUARE_MAX_ARMS: int    = 8

# --- Search phase timeouts ---
SECTOR_SEARCH_TIMEOUT: float = 15.0   # s in sector scan before → expanding square
EXPAND_SEARCH_TIMEOUT: float = 45.0   # s in expanding square before → Lissajous

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
MAVLINK_DEVICE: str  = "/dev/ttyACM0" # USB serial to Orange Cube+
MAVLINK_BAUD: int    = 921600

# --- Following geometry ---
FOLLOW_ALTITUDE_M: float      = 12.0   # Target AGL altitude to maintain (m)
FOLLOW_STANDOFF_M: float      = 12.0   # Horizontal distance to hold behind/away from person (m)
MIN_PERSON_DRONE_SEP_M: float = 8.0    # Hard minimum separation — retreat if closer (m)
MIN_VERTICAL_SEP_M: float     = 8.0    # Soft vertical clearance above person (m)
RETREAT_SPEED_MS: float       = 1.0    # Speed used when actively backing away from subject
RETREAT_HYSTERESIS_M: float   = 1.5    # Re-engage follow only when sep > MIN_SEP + this (m)
STANDOFF_VEL_THRESHOLD_MS: float = 0.3 # Use person velocity direction above this speed (m/s)
MAX_TRACKING_SPEED_MS: float  = 1.5    # Hard velocity cap (m/s) — SAFETY-CRITICAL

# --- Standoff bearing hysteresis (S2.6) ---
# Person must sustain motion above STANDOFF_VEL_THRESHOLD_MS for this long
# before bearing source flips from position-based (drone→person line) to
# velocity-based (person's heading). Prevents flicker on brief stops.
BEARING_LATCH_S: float        = 1.0
# Maximum rate of change of the standoff bearing (degrees per second).
# Smooths transitions so the target NED point cannot teleport.
BEARING_SLEW_DEG_S: float     = 30.0
MIN_ALT_M: float             = 10.0  # Absolute altitude floor (m AGL) — SAFETY-CRITICAL
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
HOME_KEEPOUT_RADIUS_M: float = 12.0  # No-fly cylinder around HOME (operator stands here)

# --- Proportional controller ---
DRONE_KP: float     = 0.4    # Position error → velocity (m/s per meter of error)
DRONE_KP_YAW: float = 0.6    # Gimbal pan angle → drone yaw rate (rad/s per degree)

# --- Velocity smoothing ---
VEL_EMA_ALPHA: float  = 0.25    # EMA filter factor (0.1=smooth/laggy, 0.5=responsive)
MAX_JERK_MS3: float   = 2.0     # Jerk limit (m/s³)
MAX_ACCEL_MS2: float  = 0.5     # Hard acceleration cap (m/s²) applied after jerk limiter

# --- MAVLink send rate ---
DRONE_CMD_RATE_HZ: int   = 10   # Velocity command send rate

# --- Failsafe timings ---
TRACKING_LOSS_HOVER_S: float  = 2.0    # Seconds before sending zero velocity
TRACKING_LOSS_LOITER_S: float = 5.0    # Seconds before issuing LOITER command
TRACKING_LOSS_ALERT_S: float  = 15.0   # Seconds before GCS terminal alert

# --- Detection confirmation before drone body movement ---
# Gimbal tracks immediately; drone body stays at zero velocity until this many
# consecutive frames have a valid detection. Prevents false-positive (bush/dog)
# from yanking the airframe.
BODY_MOVE_CONFIRM_FRAMES: int = 5

# --- Latency floor for drone body control (S3.5) ---
# When effective detection FPS drops below this, the drone body holds (sends
# zero velocity) so stale frames cannot drive a position command. Gimbal is
# unaffected.
MIN_TRACKING_FPS: float = 8.0
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
CELL_CRITICAL_MV: int        = 3500   # Per-cell critical voltage → trigger RTL

# --- SIYI attitude telemetry ---
GIMBAL_ATTITUDE_POLL_HZ: float = 10.0   # Request attitude at 10 Hz
GIMBAL_ATTITUDE_STALE_S: float = 0.2    # Use commanded angles if telemetry older than this
GIMBAL_COAST_S: float          = 0.5    # Suppress EKF update if telemetry stale beyond this

# =============================================================================
#  EXTENDED KALMAN FILTER  (person geolocation)
# =============================================================================

EKF_PROCESS_NOISE: list = [0.1, 0.1, 0.5, 0.5]   # Q diag: [pN, pE, vN, vE]
EKF_MEAS_NOISE:    list = [2.0, 2.0]              # R diag: [pN, pE] (m²)
EKF_GATE_SIGMA:    float = 3.0                    # Mahalanobis gate (σ)
EKF_MAX_JUMP_M:    float = 10.0                   # Euclidean jump rejection (S2.4)

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
DRONE_FOLLOW_DEADBAND_M: float = 0.5     # Position error below this = no horizontal command
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
