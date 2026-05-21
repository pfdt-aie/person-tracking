"""
pid_controller.py — PID controller and target smoother for gimbal tracking.

PIDController: full PID with integral windup guard, derivative filter,
               and output smoothing.
TargetSmoother: exponential moving average for bbox centroid smoothing.
"""

import time

import config as cfg


class PIDController:
    """Discrete PID controller with anti-windup, derivative filter, and output smoothing.

    Args:
        kp: Proportional gain.
        ki: Integral gain.
        kd: Derivative gain.
        output_limit: Symmetric output clamp ±output_limit.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float = 100.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.integral:         float = 0.0
        self.prev_error:       float | None = None
        self.prev_time:        float | None = None
        self.prev_derivative:  float = 0.0
        self.prev_output:      float = 0.0
        self.integral_limit:   float = 12.0   # tighter anti-windup (was 20.0)
        self.deriv_filter:     float = 0.55  # heavier derivative filter to kill noise
        self.output_smooth:    float = 0.50  # stronger output smoothing to damp oscillation

    def update(self, error: float) -> float:
        """Compute PID output for the given error.

        Args:
            error: Current error signal (e.g. normalised pixel offset).

        Returns:
            Control output clamped to ±output_limit.
        """
        now = time.time()
        lim = self.output_limit

        if self.prev_error is None or self.prev_time is None:
            out = max(-lim, min(lim, self.kp * error))
            self.prev_error  = error
            self.prev_time   = now
            self.prev_output = out
            return out

        dt = now - self.prev_time
        if dt < 0.005:
            return self.prev_output

        p = self.kp * error

        self.integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral + error * dt),
        )
        i = self.ki * self.integral

        raw_deriv = (error - self.prev_error) / dt
        filtered_deriv = (
            self.deriv_filter * self.prev_derivative
            + (1 - self.deriv_filter) * raw_deriv
        )
        d = self.kd * filtered_deriv
        self.prev_derivative = filtered_deriv

        raw = max(-lim, min(lim, p + i + d))
        smoothed = self.output_smooth * self.prev_output + (1 - self.output_smooth) * raw

        self.prev_error  = error
        self.prev_time   = now
        self.prev_output = smoothed
        return smoothed

    def reset(self) -> None:
        """Reset all internal state (call when target is re-acquired)."""
        self.integral        = 0.0
        self.prev_error      = None
        self.prev_time       = None
        self.prev_derivative = 0.0
        self.prev_output     = 0.0


class TargetSmoother:
    """Exponential moving average filter for bounding-box centroid.

    Reduces jitter caused by single-frame YOLO detection noise.

    Args:
        alpha: Smoothing factor. 0 = full smoothing (never moves),
               1 = no smoothing (raw value). Default from config.
    """

    def __init__(self, alpha: float = cfg.TARGET_SMOOTH_ALPHA) -> None:
        self.alpha = alpha
        self.fast_alpha = min(alpha, cfg.TARGET_SMOOTH_ALPHA_FAST)
        self.fast_px_s = cfg.TARGET_SMOOTH_FAST_PX_S
        self.cx: float | None = None
        self.cy: float | None = None
        self._last_raw_x: float | None = None
        self._last_raw_y: float | None = None
        self._last_t: float | None = None

    def update(self, rx: float, ry: float) -> tuple[float, float]:
        """Update with a new raw centroid and return the smoothed value.

        Args:
            rx: Raw centre-x in pixels.
            ry: Raw centre-y in pixels.

        Returns:
            (smoothed_cx, smoothed_cy)
        """
        now = time.time()
        if self.cx is None:
            self.cx, self.cy = rx, ry
        else:
            alpha = self._adaptive_alpha(rx, ry, now)
            self.cx = alpha * self.cx + (1 - alpha) * rx
            self.cy = alpha * self.cy + (1 - alpha) * ry
        self._last_raw_x = rx
        self._last_raw_y = ry
        self._last_t = now
        return self.cx, self.cy  # type: ignore[return-value]

    def _adaptive_alpha(self, rx: float, ry: float, now: float) -> float:
        """Use less lag when the person is genuinely moving in the frame."""
        if self._last_raw_x is None or self._last_raw_y is None or self._last_t is None:
            return self.alpha
        dt = max(1e-3, now - self._last_t)
        speed = (((rx - self._last_raw_x) ** 2 + (ry - self._last_raw_y) ** 2) ** 0.5) / dt
        t = max(0.0, min(1.0, speed / max(self.fast_px_s, 1.0)))
        return self.alpha + t * (self.fast_alpha - self.alpha)

    def reset(self) -> None:
        """Clear smoothed state (call when tracking is re-acquired)."""
        self.cx = None
        self.cy = None
        self._last_raw_x = None
        self._last_raw_y = None
        self._last_t = None
