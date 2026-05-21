"""PersonEKF — unit tests for correctness and stability.

Tests cover:
  - Constant-velocity model prediction
  - Measurement update and Kalman convergence
  - Sprint-startup position lag (should be < 0.5m at t=1s)
  - Abrupt-stop velocity decay (should reach < 0.1 m/s within 1.5s)
  - predict() is a no-op when uninitialised (large stale dt safety)
  - Mahalanobis gate and jump rejection interaction
  - gps_to_ned / ned_to_gps round-trip consistency
  - P remains positive-definite after sustained operation
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import numpy as np
import config as cfg
from tracking.person_geolocation import PersonEKF


# ---------------------------------------------------------------------------
# Initialisation
# ---------------------------------------------------------------------------

def test_not_valid_before_first_measurement():
    ekf = PersonEKF()
    assert not ekf.is_valid


def test_valid_after_first_measurement():
    ekf = PersonEKF()
    ekf.update(0.0, 0.0)
    assert ekf.is_valid


def test_first_measurement_sets_position():
    ekf = PersonEKF()
    ekf.update(5.0, 3.0)
    N, E = ekf.get_position_ned()
    assert abs(N - 5.0) < 0.01
    assert abs(E - 3.0) < 0.01


def test_first_measurement_sets_zero_velocity():
    ekf = PersonEKF()
    ekf.update(5.0, 3.0)
    vN, vE = ekf.get_velocity_ned()
    assert abs(vN) < 1e-9
    assert abs(vE) < 1e-9


# ---------------------------------------------------------------------------
# predict() safety
# ---------------------------------------------------------------------------

def test_predict_is_noop_when_uninitialised():
    """Large stale dt must not corrupt an uninitialised filter."""
    ekf = PersonEKF()
    P_before = ekf._P.copy()
    ekf.predict(30.0)   # 30s stale dt — simulates first call after re-follow
    assert np.allclose(ekf._P, P_before), "P changed despite filter uninitialised"
    assert not ekf.is_valid


def test_predict_advances_position_by_velocity():
    ekf = PersonEKF()
    ekf.update(0.0, 0.0)
    # Seed velocity by 10 measurements at 1.5 m/s so filter converges
    for i in range(1, 11):
        ekf.predict(0.1); ekf.update(0.15 * i, 0.0)
    N_before, _ = ekf.get_position_ned()
    ekf.predict(0.1)
    N_after, _ = ekf.get_position_ned()
    assert N_after > N_before, (
        f"predict() did not advance position: before={N_before:.3f}, after={N_after:.3f}"
    )


# ---------------------------------------------------------------------------
# Sprint startup lag
# ---------------------------------------------------------------------------

def test_sprint_startup_position_lag_under_half_metre_at_1s():
    """EKF should track a 2 m/s sprint with <0.5m position lag at t=1s."""
    ekf = PersonEKF()
    ekf.update(0.0, 0.0)
    dt = 0.1
    for step in range(10):
        t = (step + 1) * dt
        ekf.predict(dt)
        ekf.update(2.0 * t, 0.0)
    N_est, _ = ekf.get_position_ned()
    N_true = 2.0 * 1.0
    lag = abs(N_true - N_est)
    assert lag < 0.5, f"Sprint lag at 1s = {lag:.3f}m, expected < 0.5m"


# ---------------------------------------------------------------------------
# Abrupt-stop velocity decay
# ---------------------------------------------------------------------------

def test_velocity_decays_after_person_stops():
    """After person stops, EKF velocity estimate must fall below 0.1 m/s within 3.0s.

    Q_vel was reduced from 1.5 → 0.3 to eliminate GPS-velocity-noise-driven hover
    oscillation (the feedforward term is now gated on the bearing latch, so a slower
    EKF decay has no effect on drone behaviour — the drone stops sending feedforward
    the moment the person's EKF speed drops below STANDOFF_VEL_THRESHOLD_MS for 1s).
    The decay horizon is updated from 1.5s to 3.0s to match Q_vel=0.3 dynamics.
    """
    ekf = PersonEKF()
    ekf.update(0.0, 0.0)
    dt = 0.1
    # Run at 2 m/s for 2s
    pos = 0.0
    for _ in range(20):
        pos += 2.0 * dt
        ekf.predict(dt)
        ekf.update(pos, 0.0)

    # Now stop — 3.0s horizon matches Q_vel=0.3 decay rate
    stop_pos = pos
    for _ in range(30):   # 3.0s
        ekf.predict(dt)
        ekf.update(stop_pos, 0.0)

    vN, _ = ekf.get_velocity_ned()
    assert abs(vN) < 0.1, f"Velocity after 3.0s stop = {vN:.3f} m/s, expected < 0.1"


# ---------------------------------------------------------------------------
# Mahalanobis gate and jump rejection
# ---------------------------------------------------------------------------

def test_large_jump_rejected():
    ekf = PersonEKF()
    ekf.update(0.0, 0.0)
    result = ekf.update(cfg.EKF_MAX_JUMP_M + 5.0, 0.0)
    assert result is False


def test_valid_redetection_after_occlusion_accepted():
    """Person walks 4m during 6s occlusion — re-detection must be accepted."""
    ekf = PersonEKF()
    ekf.update(0.0, 0.0)
    dt = 0.1
    for _ in range(60):    # 6s predict-only
        ekf.predict(dt)
    result = ekf.update(4.0, 0.0)
    assert result is True, "Valid 4m re-detection rejected after 6s occlusion"


def test_state_not_corrupted_by_rejected_jump():
    ekf = PersonEKF()
    ekf.update(2.0, 3.0)
    N_before, E_before = ekf.get_position_ned()
    ekf.update(cfg.EKF_MAX_JUMP_M + 20.0, cfg.EKF_MAX_JUMP_M + 20.0)
    N_after, E_after = ekf.get_position_ned()
    assert abs(N_after - N_before) < 1.0
    assert abs(E_after - E_before) < 1.0


# ---------------------------------------------------------------------------
# GPS ↔ NED round-trip
# ---------------------------------------------------------------------------

def test_gps_ned_roundtrip():
    ekf = PersonEKF()
    origin_lat, origin_lon = 51.5, -0.1
    # 7m North, 3m East
    n_in, e_in = 7.0, 3.0
    lat, lon = ekf.ned_to_gps(n_in, e_in, origin_lat, origin_lon)
    n_out, e_out = ekf.gps_to_ned(lat, lon, origin_lat, origin_lon)
    assert abs(n_out - n_in) < 0.001, f"North roundtrip error: {abs(n_out-n_in):.4f}m"
    assert abs(e_out - e_in) < 0.001, f"East roundtrip error: {abs(e_out-e_in):.4f}m"


# ---------------------------------------------------------------------------
# Numerical stability
# ---------------------------------------------------------------------------

def test_P_stays_positive_definite_after_many_updates():
    """Covariance matrix must remain positive-definite after 500 updates."""
    ekf = PersonEKF()
    ekf.update(0.0, 0.0)
    dt = 0.1
    for i in range(500):
        ekf.predict(dt)
        ekf.update(float(i) * dt * 1.5, 0.0)
    eigvals = np.linalg.eigvals(ekf._P)
    assert all(e.real > 0 for e in eigvals), "P is not positive-definite after 500 updates"


def test_reset_clears_state():
    ekf = PersonEKF()
    ekf.update(10.0, 5.0)
    assert ekf.is_valid
    ekf.reset()
    assert not ekf.is_valid
    N, E = ekf.get_position_ned()
    assert N == 0.0 and E == 0.0
