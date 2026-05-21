"""tests/test_pid.py — PIDController unit tests."""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from control.pid_controller import PIDController, TargetSmoother
def _pid(kp=1.0, ki=0.0, kd=0.0, limit=100.0) -> PIDController:
    return PIDController(kp=kp, ki=ki, kd=kd, output_limit=limit)

def test_first_call_is_proportional():
    pid = _pid(kp=2.0, limit=100.0)
    out = pid.update(5.0)
    assert out == 10.0

    
def test_output_clamped_positive():
    pid = _pid(kp=100.0, limit=50.0)
    out = pid.update(10.0)
    assert out <= 50.0


def test_output_clamped_negative():
    pid = _pid(kp=100.0, limit=50.0)
    out = pid.update(-10.0)
    assert out >= -50.0


def test_zero_error_gives_zero_output():
    pid = _pid(kp=5.0)
    out = pid.update(0.0)
    assert out == 0.0


def test_reset_clears_integral():
    pid = _pid(kp=1.0, ki=1.0, limit=200.0)
    pid.update(1.0)
    time.sleep(0.02)
    pid.update(1.0)
    pid.reset()
    assert pid.integral == 0.0
    assert pid.prev_error is None


def test_reset_allows_fresh_start():
    pid = _pid(kp=2.0, limit=100.0)
    pid.update(3.0)
    pid.reset()
    out = pid.update(4.0)
    assert out == 8.0   # proportional-only after reset


def test_dead_zone_sdz():
    """Static dead-zone helper: values within dz should return 0."""
    from tracking.gimbal_state_machine import GimbalStateMachine
    assert GimbalStateMachine._sdz(0.0, 0.1) == 0.0
    assert GimbalStateMachine._sdz(0.05, 0.1) == 0.0
    assert GimbalStateMachine._sdz(0.1, 0.1) == 0.0


def test_search_pitch_clamp_stops_down_at_steep_limit():
    import config as cfg
    from tracking.gimbal_state_machine import GimbalStateMachine

    yaw, pitch = GimbalStateMachine._clamp_search_pitch_for_tilt(
        12, -8, cfg.SEARCH_PITCH_STEEP_DEG - 1.0
    )

    assert yaw == 12
    assert pitch == 0


def test_search_pitch_clamp_stops_up_at_shallow_limit():
    import config as cfg
    from tracking.gimbal_state_machine import GimbalStateMachine

    yaw, pitch = GimbalStateMachine._clamp_search_pitch_for_tilt(
        12, 8, cfg.SEARCH_PITCH_SHALLOW_DEG + 1.0
    )

    assert yaw == 12
    assert pitch == 0


def test_search_pitch_clamp_allows_motion_inside_envelope():
    from tracking.gimbal_state_machine import GimbalStateMachine

    assert GimbalStateMachine._clamp_search_pitch_for_tilt(12, -8, -45.0) == (12, -8)
    assert GimbalStateMachine._clamp_search_pitch_for_tilt(12, 8, -45.0) == (12, 8)


def test_lissajous_ground_recovery_pitches_down_above_shallow_limit():
    import config as cfg
    from tracking.gimbal_state_machine import GimbalStateMachine

    class _Ctrl:
        gimbal_tilt_deg = 0.0

    gsm = GimbalStateMachine.__new__(GimbalStateMachine)
    gsm._ctrl = _Ctrl()
    gsm._lissajous_ground_recover_until = time.time() + 1.0

    assert gsm._apply_lissajous_ground_recovery(14, 0) == (
        0,
        -cfg.SEARCH_RECENTER_PITCH_SPEED,
    )


def test_lissajous_ground_recovery_ends_at_shallow_limit():
    import config as cfg
    from tracking.gimbal_state_machine import GimbalStateMachine

    class _Ctrl:
        gimbal_tilt_deg = cfg.SEARCH_PITCH_SHALLOW_DEG - 1.0

    gsm = GimbalStateMachine.__new__(GimbalStateMachine)
    gsm._ctrl = _Ctrl()
    gsm._lissajous_ground_recover_until = time.time() + 1.0

    assert gsm._apply_lissajous_ground_recovery(14, 0) == (14, 0)
    assert gsm._lissajous_ground_recover_until == 0.0


def test_target_smoother_converges():
    sm = TargetSmoother()
    for _ in range(30):
        cx, cy = sm.update(100.0, 200.0)
    assert abs(cx - 100.0) < 1.0
    assert abs(cy - 200.0) < 1.0


def test_target_smoother_responds_faster_to_real_motion(monkeypatch):
    now = [100.0]
    monkeypatch.setattr("control.pid_controller.time.time", lambda: now[0])

    slow = TargetSmoother(alpha=0.8)
    slow.fast_alpha = 0.2
    slow.fast_px_s = 100.0
    slow.update(0.0, 0.0)
    now[0] += 0.1
    slow_x, _ = slow.update(1.0, 0.0)

    fast = TargetSmoother(alpha=0.8)
    fast.fast_alpha = 0.2
    fast.fast_px_s = 100.0
    fast.update(0.0, 0.0)
    now[0] += 0.1
    fast_x, _ = fast.update(100.0, 0.0)

    assert slow_x < 0.5
    assert fast_x > 70.0
