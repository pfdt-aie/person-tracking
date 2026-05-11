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


def test_target_smoother_converges():
    sm = TargetSmoother()
    for _ in range(30):
        cx, cy = sm.update(100.0, 200.0)
    assert abs(cx - 100.0) < 1.0
    assert abs(cy - 200.0) < 1.0
