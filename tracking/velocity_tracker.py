"""
velocity_tracker.py — Rolling velocity estimator for the tracked person.

Maintains a time-stamped history of bounding-box centroids (in normalised
frame coordinates) and computes a recency-weighted velocity estimate.

Used for:
  - Prediction phase: extrapolate where the person is after YOLO loses them.
  - Edge-exit detection: boost gimbal speed when person leaves frame boundary.
  - Search direction: bias sector scan toward the last known travel direction.
  - Drone feedforward: supply person velocity to drone_controller.py EKF.
"""

import math
import time
from collections import deque

import config as cfg


class VelocityTracker:
    """Recency-weighted velocity from a rolling centroid history.

    Normalised coordinates: (-1, -1) = top-left, (0, 0) = frame centre,
    (+1, +1) = bottom-right.

    Args:
        window_sec: Time window over which velocity is averaged.
    """

    def __init__(self, window_sec: float = cfg.VELOCITY_WINDOW) -> None:
        self.window_sec = window_sec
        # (timestamp, ncx, ncy, x1, y1, x2, y2, frame_w, frame_h)
        self.history: deque = deque(maxlen=200)

        self.vel_x: float        = 0.0   # positive = moving right (norm/s)
        self.vel_y: float        = 0.0   # positive = moving down  (norm/s)
        self.speed: float        = 0.0   # magnitude
        self.direction_deg: float = 0.0  # 0=right, 90=down, 180=left, 270=up

        self.edge_exit: str | None  = None   # 'left'|'right'|'top'|'bottom'|None
        self.edge_boost_yaw: float   = 0.0
        self.edge_boost_pitch: float = 0.0

        self.last_cx: float = 0.0
        self.last_cy: float = 0.0
        self.valid: bool    = False

    # ------------------------------------------------------------------
    #  Public update
    # ------------------------------------------------------------------

    def update(
        self,
        cx: float, cy: float,
        x1: float, y1: float,
        x2: float, y2: float,
        frame_w: int, frame_h: int,
    ) -> None:
        """Record a new centroid observation and recompute velocity.

        Args:
            cx, cy:         Pixel centroid of bounding box.
            x1, y1, x2, y2: Bounding box corners (pixels).
            frame_w, frame_h: Frame resolution.
        """
        now = time.time()
        ncx = (cx - frame_w / 2) / (frame_w / 2)
        ncy = (cy - frame_h / 2) / (frame_h / 2)
        self.history.append((now, ncx, ncy, x1, y1, x2, y2, frame_w, frame_h))
        self.last_cx = ncx
        self.last_cy = ncy
        self._compute_velocity(now)
        self._detect_edge(x1, y1, x2, y2, frame_w, frame_h)

    # ------------------------------------------------------------------
    #  Velocity computation
    # ------------------------------------------------------------------

    def _compute_velocity(self, now: float) -> None:
        """Weighted linear regression over recent centroid history."""
        cutoff  = now - self.window_sec
        samples = [(t, x, y) for t, x, y, *_ in self.history if t >= cutoff]

        if len(samples) < cfg.VELOCITY_MIN_SAMPLES:
            self.valid = False
            return

        vx_sum = vy_sum = w_sum = 0.0
        for i in range(1, len(samples)):
            dt = samples[i][0] - samples[i - 1][0]
            if dt < 0.001:
                continue
            vx = (samples[i][1] - samples[i - 1][1]) / dt
            vy = (samples[i][2] - samples[i - 1][2]) / dt
            age    = now - samples[i][0]
            weight = max(0.1, 1.0 - age / self.window_sec)
            vx_sum += vx * weight
            vy_sum += vy * weight
            w_sum  += weight

        if w_sum > 0:
            self.vel_x         = vx_sum / w_sum
            self.vel_y         = vy_sum / w_sum
            self.speed         = math.sqrt(self.vel_x**2 + self.vel_y**2)
            self.direction_deg = math.degrees(math.atan2(self.vel_y, self.vel_x))
            self.valid         = True
        else:
            self.valid = False

    # ------------------------------------------------------------------
    #  Edge detection
    # ------------------------------------------------------------------

    def _detect_edge(
        self,
        x1: float, y1: float,
        x2: float, y2: float,
        fw: int, fh: int,
    ) -> None:
        """Detect if the bounding box is touching (or crossing) a frame edge."""
        mx = fw * cfg.EDGE_MARGIN_RATIO
        my = fh * cfg.EDGE_MARGIN_RATIO
        self.edge_exit       = None
        self.edge_boost_yaw  = 0.0
        self.edge_boost_pitch = 0.0

        at_left   = x1 <= mx
        at_right  = x2 >= fw - mx
        at_top    = y1 <= my
        at_bottom = y2 >= fh - my

        # Build exit label from both axes independently so corner exits
        # (e.g. right+bottom) are represented correctly.  The previous
        # `self.edge_exit or "bottom"` pattern was a bug: a truthy yaw-edge
        # string caused the pitch edge to be silently dropped for corners.
        _exits: list[str] = []
        if at_right and self.vel_x >= 0:
            _exits.append("right")
            self.edge_boost_yaw = cfg.EDGE_EXIT_BOOST
        elif at_left and self.vel_x <= 0:
            _exits.append("left")
            self.edge_boost_yaw = cfg.EDGE_EXIT_BOOST

        if at_bottom and self.vel_y >= 0:
            _exits.append("bottom")
            self.edge_boost_pitch = cfg.EDGE_EXIT_BOOST
        elif at_top and self.vel_y <= 0:
            _exits.append("top")
            self.edge_boost_pitch = cfg.EDGE_EXIT_BOOST

        self.edge_exit = "+".join(_exits) if _exits else None

    # ------------------------------------------------------------------
    #  Prediction
    # ------------------------------------------------------------------

    def predict_command(
        self,
        dt_since_loss: float,
        adaptive_kp: float,
        adaptive_speed: float,
    ) -> tuple[int, int]:
        """Return (yaw_cmd, pitch_cmd) based on last known velocity.

        Args:
            dt_since_loss:  Seconds since the person was last seen.
            adaptive_kp:    Current proportional gain (from _adapt_speed).
            adaptive_speed: Current speed limit.

        Returns:
            Integer gimbal speed commands (yaw, pitch), each ±100.
        """
        if not self.valid:
            return 0, 0

        boost_y = self.edge_boost_yaw   if self.edge_boost_yaw   > 0 else 1.0
        boost_p = self.edge_boost_pitch if self.edge_boost_pitch > 0 else 1.0

        yaw_cmd   =  self.vel_x * adaptive_kp * boost_y
        pitch_cmd = -self.vel_y * adaptive_kp * boost_p  # invert: +vel_y = down = neg pitch

        yaw_cmd   = max(-adaptive_speed, min(adaptive_speed, yaw_cmd))
        pitch_cmd = max(-adaptive_speed, min(adaptive_speed, pitch_cmd))

        if 0 < abs(yaw_cmd)   < cfg.GIMBAL_MIN_SPEED:
            yaw_cmd   = cfg.GIMBAL_MIN_SPEED * (1 if yaw_cmd   > 0 else -1)
        if 0 < abs(pitch_cmd) < cfg.GIMBAL_MIN_SPEED:
            pitch_cmd = cfg.GIMBAL_MIN_SPEED * (1 if pitch_cmd > 0 else -1)

        return int(yaw_cmd), int(pitch_cmd)

    def get_search_direction(self) -> tuple[int, int]:
        """Return (yaw_dir, pitch_dir) for the sector scan search.

        Returns:
            yaw_dir:   +1 = right, -1 = left.
            pitch_dir: -1 = down,  +1 = up.
        """
        if not self.valid:
            return 1, 0
        yaw_dir   = 1 if self.vel_x >= 0 else -1
        pitch_dir = -1 if self.vel_y >= 0 else 1
        return yaw_dir, pitch_dir

    # ------------------------------------------------------------------
    #  Reset
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Clear all history and state (call when stale after long search)."""
        self.history.clear()
        self.vel_x = self.vel_y = self.speed = 0.0
        self.direction_deg   = 0.0
        self.edge_exit       = None
        self.edge_boost_yaw  = self.edge_boost_pitch = 0.0
        self.valid           = False
