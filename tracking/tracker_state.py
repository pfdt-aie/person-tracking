"""
tracker_state.py — Shared mutable state for the tracker's extracted subsystems.

All five extracted classes (TargetSelector, GimbalStateMachine,
OperatorInputController, WebControlAdapter, and PersonGimbalTracker itself)
hold a reference to the same TrackerState instance.  This avoids deep
parameter-threading while keeping state clearly named and in one place.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from tracking.state_machine import State
from tracking.target_detection import TargetDetection


@dataclass
class TrackerState:
    # --- Target selection ---
    lock_id:      Optional[int]          = None
    detected_ids: dict                   = field(default_factory=dict)  # dict[int, TargetDetection]

    # --- Mode ---
    mode: str = "AUTO"   # "AUTO" | "MANUAL"

    # --- Gimbal state machine ---
    state:            str  = State.WAITING
    tracking_enabled: bool = True
    search_enabled:   bool = True

    # --- Manual gimbal commands (operator-driven) ---
    manual_yaw_speed:    int   = 0
    manual_pitch_speed:  int   = 0
    manual_key_t:        float = 0.0   # keyboard-hold expiry timestamp
    manual_zoom_stop_at: float = 0.0   # web zoom auto-stop timestamp

    # --- Telemetry written by GimbalStateMachine, read by HudRenderer ---
    telem_error_x:        float = 0.0
    telem_error_y:        float = 0.0
    telem_yaw_cmd:        int   = 0
    telem_pitch_cmd:      int   = 0
    telem_bbox_ratio:     float = 0.0
    telem_adaptive_kp:    float = 0.0
    telem_adaptive_speed: float = 0.0

    # --- Lifecycle ---
    running: bool = True
