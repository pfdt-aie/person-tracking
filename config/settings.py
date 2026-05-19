"""
config/settings.py — Immutable Settings object for the tracking system.

Replaces direct reads of the mutable `config` module globals with a
frozen dataclass that is constructed once (in main.py after CLI parsing)
and injected into class constructors.

Migration path: classes accept an optional `settings: Settings` parameter
alongside existing cfg usage.  Convert one class at a time; start with
SafetyMonitor (fewest external deps), then MAVLinkClient, DroneController.

Usage:
    from config.settings import Settings, load_settings
    settings = load_settings(follow_standoff_m=5.0)   # CLI overrides
    safety   = SafetyMonitor(settings=settings)
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import List

import config as _cfg


@dataclass(frozen=True)
class Settings:
    # --- Camera / network ---
    camera_ip:          str   = _cfg.CAMERA_IP
    gimbal_port:        int   = _cfg.GIMBAL_PORT
    rtsp_url:           str   = _cfg.RTSP_URL
    model_path:         str   = _cfg.MODEL_PATH
    conf_threshold:     float = _cfg.CONF_THRESHOLD
    imgsz:              int   = _cfg.IMGSZ
    detect_device:      str   = _cfg.DETECT_DEVICE

    # --- PID gimbal ---
    pid_yaw_kp:         float = _cfg.PID_YAW_KP
    pid_yaw_ki:         float = _cfg.PID_YAW_KI
    pid_yaw_kd:         float = _cfg.PID_YAW_KD
    pid_pitch_kp:       float = _cfg.PID_PITCH_KP
    pid_pitch_ki:       float = _cfg.PID_PITCH_KI
    pid_pitch_kd:       float = _cfg.PID_PITCH_KD
    dead_zone:          float = _cfg.DEAD_ZONE
    max_gimbal_speed:   int   = _cfg.MAX_GIMBAL_SPEED
    gimbal_min_speed:   int   = _cfg.GIMBAL_MIN_SPEED
    manual_gimbal_speed: int  = _cfg.MANUAL_GIMBAL_SPEED

    # --- Following / standoff ---
    follow_altitude_m:        float = _cfg.FOLLOW_ALTITUDE_M
    follow_standoff_m:        float = _cfg.FOLLOW_STANDOFF_M
    min_person_drone_sep_m:   float = _cfg.MIN_PERSON_DRONE_SEP_M
    standoff_vel_threshold_ms: float = _cfg.STANDOFF_VEL_THRESHOLD_MS
    max_tracking_speed_ms:    float = _cfg.MAX_TRACKING_SPEED_MS
    min_alt_m:                float = _cfg.MIN_ALT_M
    max_alt_m:                float = _cfg.MAX_ALT_M
    geofence_radius_m:        float = _cfg.GEOFENCE_RADIUS_M

    # --- Drone control ---
    mavlink_device:   str   = _cfg.MAVLINK_DEVICE
    mavlink_baud:     int   = _cfg.MAVLINK_BAUD
    drone_kp:         float = _cfg.DRONE_KP
    drone_kp_yaw:     float = _cfg.DRONE_KP_YAW
    drone_cmd_rate_hz: int  = _cfg.DRONE_CMD_RATE_HZ
    drone_follow_deadband_m: float = _cfg.DRONE_FOLLOW_DEADBAND_M
    drone_max_yaw_rate_deg:  float = _cfg.DRONE_MAX_YAW_RATE_DEG
    vel_ema_alpha:           float = _cfg.VEL_EMA_ALPHA
    max_jerk_ms3:            float = _cfg.MAX_JERK_MS3
    max_accel_ms2:           float = _cfg.MAX_ACCEL_MS2

    # --- Following / hybrid gimbal-drone ---
    gimbal_pan_soft_deg:    float = _cfg.GIMBAL_PAN_SOFT_DEG
    gimbal_pan_hard_deg:    float = _cfg.GIMBAL_PAN_HARD_DEG
    gimbal_tilt_default_deg: float = _cfg.GIMBAL_TILT_DEFAULT_DEG
    retreat_speed_ms:       float = _cfg.RETREAT_SPEED_MS
    retreat_hysteresis_m:   float = _cfg.RETREAT_HYSTERESIS_M
    min_vertical_sep_m:     float = _cfg.MIN_VERTICAL_SEP_M
    bearing_latch_s:        float = _cfg.BEARING_LATCH_S
    bearing_slew_deg_s:     float = _cfg.BEARING_SLEW_DEG_S

    # --- Tracking-loss failsafes ---
    lost_confirm_frames:      int = _cfg.LOST_CONFIRM_FRAMES
    body_move_confirm_frames: int = _cfg.BODY_MOVE_CONFIRM_FRAMES
    tracking_loss_hover_s:  float = _cfg.TRACKING_LOSS_HOVER_S
    tracking_loss_loiter_s: float = _cfg.TRACKING_LOSS_LOITER_S
    tracking_loss_alert_s:  float = _cfg.TRACKING_LOSS_ALERT_S
    min_tracking_fps:       float = _cfg.MIN_TRACKING_FPS
    fps_window_size:        int   = _cfg.FPS_WINDOW_SIZE
    max_flight_time_s:      float = _cfg.MAX_FLIGHT_TIME_S
    loiter_confirm_timeout_s: float = _cfg.LOITER_CONFIRM_TIMEOUT_S
    mode_warn_interval_s:   float = _cfg.MODE_WARN_INTERVAL_S

    # --- Geofence return / RTL retry ---
    geofence_return_max_ms: float = _cfg.GEOFENCE_RETURN_MAX_MS
    geofence_return_kp:     float = _cfg.GEOFENCE_RETURN_KP
    rtl_max_attempts:       int   = _cfg.RTL_MAX_ATTEMPTS
    rtl_retry_interval_s:   float = _cfg.RTL_RETRY_INTERVAL_S

    # --- Safety ---
    heartbeat_watchdog_s:   float = _cfg.HEARTBEAT_WATCHDOG_S
    heartbeat_warn_s:       float = _cfg.HEARTBEAT_WARN_S
    cell_warn_mv:           int   = _cfg.CELL_WARN_MV
    cell_critical_mv:       int   = _cfg.CELL_CRITICAL_MV
    cell_nominal_mv:        int   = _cfg.CELL_NOMINAL_MV
    default_cells:          int   = _cfg.DEFAULT_CELLS
    home_keepout_radius_m:  float = _cfg.HOME_KEEPOUT_RADIUS_M

    # --- Telemetry quality gates ---
    telemetry_stale_s:      float = _cfg.TELEMETRY_STALE_S
    gps_min_fix_type:       int   = _cfg.GPS_MIN_FIX_TYPE
    gps_min_sats:           int   = _cfg.GPS_MIN_SATS
    gps_max_hdop:           float = _cfg.GPS_MAX_HDOP
    ekf_max_variance:       float = _cfg.EKF_MAX_VARIANCE
    rc_watchdog_s:          float = _cfg.RC_WATCHDOG_S
    rc_min_channels:        int   = _cfg.RC_MIN_CHANNELS

    # --- Ground-test (S1.2) ---
    # When True the MAVLinkClient logs every intended TX but never actually
    # transmits. RX is unaffected so the operator still sees real telemetry.
    ground_test: bool = False

    # --- Battery cell override (S3.4) ---
    # 0 = auto-detect from pack voltage. 3–6 = force cell count.
    cells_override: int = 0

    # --- Gimbal telemetry ---
    gimbal_attitude_stale_s: float = _cfg.GIMBAL_ATTITUDE_STALE_S
    gimbal_coast_s:          float = _cfg.GIMBAL_COAST_S

    # --- Streaming ---
    stream_enabled: bool = _cfg.STREAM_ENABLED
    stream_port:    int  = _cfg.STREAM_PORT
    stream_host:    str  = _cfg.STREAM_HOST
    stream_token:   str  = _cfg.STREAM_TOKEN

    # --- Recording ---
    auto_record:    bool  = _cfg.AUTO_RECORD
    record_overlay: bool  = _cfg.RECORD_OVERLAY
    record_fps:     float = _cfg.RECORD_FPS

    # --- EKF ---
    ekf_process_noise: tuple = field(default_factory=lambda: tuple(_cfg.EKF_PROCESS_NOISE))
    ekf_meas_noise:    tuple = field(default_factory=lambda: tuple(_cfg.EKF_MEAS_NOISE))
    ekf_gate_sigma:    float = _cfg.EKF_GATE_SIGMA

    # --- Camera intrinsics ---
    siyi_a8_hfov_deg:  float = _cfg.SIYI_A8_HFOV_DEG
    calibration_yaml:  str   = _cfg.CALIBRATION_YAML


def load_settings(**overrides) -> Settings:
    """Build a Settings instance from config defaults with optional overrides.

    Overrides use the lowercase field names of Settings.  Unknown keys raise
    TypeError at construction time so typos are caught immediately.

    Example:
        s = load_settings(follow_standoff_m=5.0, detect_device="cpu")
    """
    base: dict = {}
    for f in fields(Settings):
        cfg_name = f.name.upper()
        if hasattr(_cfg, cfg_name):
            val = getattr(_cfg, cfg_name)
            # lists → tuples for frozen dataclass compatibility
            if isinstance(val, list):
                val = tuple(val)
            base[f.name] = val
        # fields with defaults (like ekf_process_noise) fall back to dataclass default
    base.update(overrides)
    return Settings(**base)
