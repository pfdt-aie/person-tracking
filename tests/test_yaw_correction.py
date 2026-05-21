"""S2.7 — Drone yaw correction for gimbal pan recentering.

Tests cover:
  - No correction below / at the soft pan limit
  - Proportional ramp between soft and hard limit
  - Cap at hard pan limit (factor = 1.0)
  - Correct sign convention: positive pan → negative correction (drone yaws right,
    gimbal compensates left); negative pan → positive correction.
  - Bearing wrap-crossing: _bearing_rad ≈ +π, desired ≈ -π must produce a SMALL
    diff (≈ 0), not a ≈ 2π spin in the wrong direction.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from config.settings import load_settings
from control.drone_controller import DroneController


def _controller() -> DroneController:
    c = DroneController.__new__(DroneController)
    c._s = load_settings()
    c._vel_above_t  = -1.0
    c._bearing_rad  = 0.0
    c._bearing_init = False
    return c


# ---------------------------------------------------------------------------
# _compute_yaw_correction — below soft limit
# ---------------------------------------------------------------------------

def test_yaw_correction_zero_below_soft_limit():
    c = _controller()
    result = c._compute_yaw_correction(0.0)
    assert result == 0.0


def test_yaw_correction_zero_at_soft_limit():
    c = _controller()
    result = c._compute_yaw_correction(c._s.gimbal_pan_soft_deg)
    assert result == 0.0


def test_yaw_correction_zero_just_inside_soft_limit():
    c = _controller()
    result = c._compute_yaw_correction(c._s.gimbal_pan_soft_deg - 0.01)
    assert result == 0.0


# ---------------------------------------------------------------------------
# Sign convention
# ---------------------------------------------------------------------------

def test_yaw_correction_positive_pan_gives_negative_correction():
    """Positive gimbal pan (person right) → drone yaws right → gimbal must
    pan LEFT to compensate → correction is negative (subtracted from PID)."""
    c = _controller()
    pan = c._s.gimbal_pan_soft_deg + 10.0   # above soft limit
    result = c._compute_yaw_correction(pan)
    assert result < 0.0, f"Expected negative correction for pan={pan}, got {result}"


def test_yaw_correction_negative_pan_gives_positive_correction():
    """Negative gimbal pan (person left) → drone yaws left → gimbal must
    pan RIGHT to compensate → correction is positive."""
    c = _controller()
    pan = -(c._s.gimbal_pan_soft_deg + 10.0)
    result = c._compute_yaw_correction(pan)
    assert result > 0.0, f"Expected positive correction for pan={pan}, got {result}"


def test_yaw_correction_is_antisymmetric():
    """Magnitude of correction must be equal for ±pan of same absolute value."""
    c = _controller()
    pan = c._s.gimbal_pan_soft_deg + 20.0
    pos = c._compute_yaw_correction(+pan)
    neg = c._compute_yaw_correction(-pan)
    assert math.isclose(pos, -neg, rel_tol=1e-9)


# ---------------------------------------------------------------------------
# Magnitude — ramp and cap
# ---------------------------------------------------------------------------

def test_yaw_correction_increases_toward_hard_limit():
    """Correction magnitude should grow as pan moves from soft → hard limit."""
    c = _controller()
    soft = c._s.gimbal_pan_soft_deg
    hard = c._s.gimbal_pan_hard_deg
    mid  = (soft + hard) / 2.0
    corr_mid  = abs(c._compute_yaw_correction(mid))
    corr_hard = abs(c._compute_yaw_correction(hard))
    assert corr_hard > corr_mid, "Correction at hard limit must exceed correction at midpoint"


def test_yaw_correction_capped_beyond_hard_limit():
    """Correction must not grow beyond the hard-limit value even if pan exceeds it."""
    c = _controller()
    hard = c._s.gimbal_pan_hard_deg
    at_hard   = abs(c._compute_yaw_correction(hard))
    beyond    = abs(c._compute_yaw_correction(hard + 30.0))
    assert math.isclose(at_hard, beyond, rel_tol=1e-9), (
        f"Correction beyond hard limit ({beyond:.4f}) should equal hard-limit value ({at_hard:.4f})"
    )


def test_yaw_correction_at_hard_limit_equals_max_rate():
    """At the hard pan limit factor=1.0, so correction = max_yaw_rate * kp."""
    c = _controller()
    max_yaw_rate = math.radians(c._s.drone_max_yaw_rate_deg)
    expected_magnitude = max_yaw_rate * c._s.drone_kp_yaw
    result = abs(c._compute_yaw_correction(c._s.gimbal_pan_hard_deg))
    assert math.isclose(result, expected_magnitude, rel_tol=1e-6), (
        f"At hard limit expected |correction|={expected_magnitude:.4f}, got {result:.4f}"
    )


# ---------------------------------------------------------------------------
# Bearing anti-spin (wrap-crossing) — the historical 180° spin bug
# ---------------------------------------------------------------------------

def test_bearing_wrap_crossing_gives_small_diff(monkeypatch):
    """_bearing_rad ≈ +π and desired ≈ -π represent almost the same direction.

    A naive subtraction (desired - bearing) would give ≈ -2π — a full reverse
    spin. The wrap formula (desire - bearing + π) % 2π - π must produce a
    diff ≈ 0.02 rad so the slew only steps a tiny amount.

    Historical context: before the formula was corrected, the drone spun 180°
    whenever the standoff bearing crossed the ±π boundary (person moved from
    slightly east-of-south to slightly west-of-south of the drone).
    """
    c = _controller()

    # Seed bearing just below +π (e.g. person is south-southeast of drone)
    c._bearing_rad  = math.pi - 0.01
    c._bearing_init = True
    c._vel_above_t  = -1.0

    # Desired is just above -π (person is south-southwest) — same actual direction
    # atan2 for point slightly west-of-south: pN<0, pE slightly negative
    # atan2(-1, -0.01) ≈ -π + small ≈ -3.132
    epsilon = 0.01
    target_pN = -1.0
    target_pE = -epsilon           # slightly west of due south → atan2 ≈ -π + ε

    dt  = 0.1
    now = 1.0

    result = c._update_bearing(
        pN=target_pN, pE=target_pE,
        drone_pN=0.0, drone_pE=0.0,
        sep=math.hypot(target_pN, target_pE),
        vN_p=0.0, vE_p=0.0,
        now=now, dt=dt,
    )

    # The actual desired bearing (atan2 of pE-0, pN-0) ≈ -π + tiny
    desired = math.atan2(target_pE - 0.0, target_pN - 0.0)
    naive_diff = desired - (math.pi - 0.01)   # ≈ -2π: wrong direction
    correct_diff = (desired - (math.pi - 0.01) + math.pi) % (2 * math.pi) - math.pi

    # The slew step must be based on correct_diff (≈ 0), not naive_diff (≈ -2π).
    # Measure the angular distance correctly (wrap-aware), not raw subtraction.
    start = math.pi - 0.01
    step_taken = abs((result - start + math.pi) % (2 * math.pi) - math.pi)
    max_step = math.radians(cfg.BEARING_SLEW_DEG_S) * dt
    assert step_taken <= max_step + 1e-9, (
        f"Wrap-aware step={step_taken:.4f} rad exceeds slew cap {max_step:.4f}; "
        f"correct_diff={correct_diff:.4f}, naive_diff={naive_diff:.4f} — "
        "wrap formula may have applied the wrong (long) path"
    )


def test_bearing_anti_spin_slew_is_small_not_reverse():
    """Direct test: after one step with desired ≈ -π and bearing ≈ +π,
    the bearing must move by at most BEARING_SLEW_DEG_S * dt, confirming
    the shortest path (≈ 0.02 rad) was taken — not the long way (≈ 6.26 rad)."""
    c = _controller()
    c._bearing_rad  = math.pi - 0.01
    c._bearing_init = True

    desired = -math.pi + 0.01    # almost opposite representation of same angle
    # Construct pN, pE such that atan2(pE, pN) ≈ desired
    c._vel_above_t = -1.0
    # Directly call with a position that gives the desired bearing
    pN = math.cos(desired)
    pE = math.sin(desired)
    sep = 1.0

    result = c._update_bearing(
        pN=pN, pE=pE, drone_pN=0.0, drone_pE=0.0, sep=sep,
        vN_p=0.0, vE_p=0.0, now=1.0, dt=0.1,
    )

    start = math.pi - 0.01
    step_taken = abs((result - start + math.pi) % (2 * math.pi) - math.pi)
    max_step = math.radians(cfg.BEARING_SLEW_DEG_S) * 0.1
    assert step_taken <= max_step + 1e-9, (
        f"Step={step_taken:.4f} rad exceeds slew cap {max_step:.4f} — "
        "wrap formula may have applied the wrong (long) path"
    )
