"""tests/test_standoff.py — Standoff bearing and separation guard logic (P1-3)."""
import math, sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg

# Replicate the standoff bearing logic from drone_controller._compute_follow_target
# so we can test it without hardware.
def _compute_standoff(
    pN, pE, drone_pN, drone_pE, vN_p, vE_p,
    standoff_m=None, min_sep=None, vel_threshold=None,
):
    standoff_m    = standoff_m    or cfg.FOLLOW_STANDOFF_M
    min_sep       = min_sep       or cfg.MIN_PERSON_DRONE_SEP_M
    vel_threshold = vel_threshold or cfg.STANDOFF_VEL_THRESHOLD_MS

    sep = math.hypot(pN - drone_pN, pE - drone_pE)
    if sep < min_sep:
        return None, None, "too_close"

    speed_p = math.hypot(vN_p, vE_p)
    if speed_p > vel_threshold:
        bearing = math.atan2(vE_p, vN_p)
    else:
        bearing = math.atan2(pE - drone_pE, pN - drone_pN) if sep > 0.1 else 0.0

    target_pN = pN - math.cos(bearing) * standoff_m
    target_pE = pE - math.sin(bearing) * standoff_m
    return target_pN, target_pE, "ok"


def test_standoff_north_of_stationary_person():
    # Person at (10, 0), drone at (0, 0) → drone-to-person bearing = North
    # standoff target should be at (10 - 8, 0) = (2, 0)
    tN, tE, status = _compute_standoff(
        pN=10, pE=0, drone_pN=0, drone_pE=0,
        vN_p=0, vE_p=0,
        standoff_m=8.0, vel_threshold=0.3,
    )
    assert status == "ok"
    assert abs(tN - 2.0) < 0.01
    assert abs(tE - 0.0) < 0.01


def test_standoff_uses_velocity_direction_when_moving():
    # Person moving East at 1 m/s; standoff should be West of person
    tN, tE, status = _compute_standoff(
        pN=0, pE=0, drone_pN=-5, drone_pE=0,
        vN_p=0, vE_p=1.0,   # moving East
        standoff_m=8.0, vel_threshold=0.3,
    )
    assert status == "ok"
    # bearing = East (atan2(1,0) = π/2); standoff is West of person: tE ≈ -8
    assert abs(tE - (-8.0)) < 0.1
    assert abs(tN - 0.0) < 0.1


def test_standoff_uses_drone_to_person_when_slow():
    # Person nearly stationary: velocity = 0.1 m/s < threshold 0.3
    tN, tE, status = _compute_standoff(
        pN=10, pE=0, drone_pN=0, drone_pE=0,
        vN_p=0.1, vE_p=0,   # slow — below threshold
        standoff_m=8.0, vel_threshold=0.3,
    )
    assert status == "ok"
    # Falls back to drone-to-person bearing (North), so target ≈ (2, 0)
    assert abs(tN - 2.0) < 0.1


def test_separation_guard_halts_when_too_close():
    tN, tE, status = _compute_standoff(
        pN=1, pE=0, drone_pN=0, drone_pE=0,  # sep = 1 m < min_sep 4 m
        vN_p=0, vE_p=0,
        min_sep=4.0,
    )
    assert status == "too_close"
    assert tN is None


def test_separation_guard_allows_safe_distance():
    tN, tE, status = _compute_standoff(
        pN=10, pE=0, drone_pN=0, drone_pE=0,  # sep = 10 m > min_sep 4 m
        vN_p=0, vE_p=0,
        min_sep=4.0,
    )
    assert status == "ok"
