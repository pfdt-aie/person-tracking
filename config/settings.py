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

    # --- Safety ---
    heartbeat_watchdog_s: float = _cfg.HEARTBEAT_WATCHDOG_S
    cell_critical_mv:     int   = _cfg.CELL_CRITICAL_MV
    default_cells:        int   = _cfg.DEFAULT_CELLS

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
