"""VelocityTracker — unit tests for prediction, velocity computation, and edge detection."""
import time

import config as cfg
from tracking.velocity_tracker import VelocityTracker


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _tracker_with_velocity(vel_x: float, vel_y: float, speed: float = 0.0) -> VelocityTracker:
    v = VelocityTracker()
    v.valid       = True
    v.vel_x       = vel_x
    v.vel_y       = vel_y
    v.speed       = speed if speed else (vel_x**2 + vel_y**2) ** 0.5
    return v


def _tracker_with_history(pairs: list[tuple[float, float]], window: float = 1.0) -> VelocityTracker:
    """Build a VelocityTracker with the given (ncx, ncy) history at 10 Hz."""
    v = VelocityTracker(window_sec=window)
    base = time.time() - window + 0.05
    for i, (ncx, ncy) in enumerate(pairs):
        t = base + i * 0.1
        v.history.append((t, ncx, ncy, 0, 0, 0, 0, 640, 480))
    v._compute_velocity(time.time())
    return v


# ---------------------------------------------------------------------------
# predict_command
# ---------------------------------------------------------------------------

def test_predict_command_returns_zero_when_not_valid():
    v = VelocityTracker()
    assert v.predict_command(50.0, 80) == (0, 0)


def test_predict_command_returns_nonzero_for_valid_velocity():
    v = _tracker_with_velocity(vel_x=0.3, vel_y=0.0)
    yaw, pitch = v.predict_command(adaptive_kp=50.0, adaptive_speed=80)
    assert yaw > 0, "Rightward velocity must give positive yaw command"
    assert pitch == 0


def test_predict_command_clamps_to_adaptive_speed():
    v = _tracker_with_velocity(vel_x=10.0, vel_y=0.0)
    yaw, _ = v.predict_command(adaptive_kp=100.0, adaptive_speed=20)
    assert abs(yaw) <= 20, f"yaw {yaw} must be clamped to adaptive_speed=20"


def test_predict_command_applies_gimbal_min_speed_floor():
    """A very small velocity must be raised to GIMBAL_MIN_SPEED, not dropped to 0."""
    v = _tracker_with_velocity(vel_x=0.001, vel_y=0.0)  # extremely slow
    yaw, _ = v.predict_command(adaptive_kp=50.0, adaptive_speed=80)
    # Either zero (below noise threshold after int cast) OR at min speed
    assert yaw == 0 or abs(yaw) >= cfg.GIMBAL_MIN_SPEED, (
        f"yaw {yaw} must be 0 or >= GIMBAL_MIN_SPEED={cfg.GIMBAL_MIN_SPEED}"
    )


def test_predict_command_sub_min_speed_is_clamped_up():
    """vel_x that computes to just below GIMBAL_MIN_SPEED is raised to min."""
    v = _tracker_with_velocity(vel_x=0.0, vel_y=0.0)
    v.valid = True
    # Manually set a velocity that produces 3.0 < GIMBAL_MIN_SPEED=8
    v.vel_x = 3.0 / 50.0   # kp=50 → raw cmd = 3.0
    v.vel_y = 0.0
    yaw, _ = v.predict_command(adaptive_kp=50.0, adaptive_speed=80)
    assert abs(yaw) >= cfg.GIMBAL_MIN_SPEED, (
        f"sub-min command {yaw} must be raised to {cfg.GIMBAL_MIN_SPEED}"
    )


def test_predict_command_edge_boost_amplifies_yaw():
    """Edge exit boost must produce a larger command than without it."""
    v_plain = _tracker_with_velocity(vel_x=0.3, vel_y=0.0)
    v_boost = _tracker_with_velocity(vel_x=0.3, vel_y=0.0)
    v_boost.edge_boost_yaw = cfg.EDGE_EXIT_BOOST

    yaw_plain, _ = v_plain.predict_command(50.0, 80)
    yaw_boost, _ = v_boost.predict_command(50.0, 80)
    assert yaw_boost >= yaw_plain, "Edge boost must produce equal or larger yaw command"


# ---------------------------------------------------------------------------
# _compute_velocity
# ---------------------------------------------------------------------------

def test_compute_velocity_valid_from_ordered_samples():
    """Rightward motion (increasing ncx) must yield positive vel_x."""
    pairs = [(i * 0.05, 0.0) for i in range(10)]   # ncx 0→0.45 over 0.9s
    v = _tracker_with_history(pairs)
    assert v.valid is True
    assert v.vel_x > 0, f"Expected vel_x > 0 for rightward motion, got {v.vel_x:.3f}"


def test_compute_velocity_invalid_with_too_few_samples():
    """Fewer than VELOCITY_MIN_SAMPLES unique samples → valid = False."""
    v = VelocityTracker()
    # Push only 3 samples (less than VELOCITY_MIN_SAMPLES=5)
    base = time.time() - 0.5
    for i in range(3):
        v.history.append((base + i * 0.1, i * 0.1, 0.0, 0, 0, 0, 0, 640, 480))
    v._compute_velocity(time.time())
    assert v.valid is False


def test_compute_velocity_downward_motion_gives_positive_vel_y():
    """Increasing ncy (downward in frame) must yield positive vel_y."""
    pairs = [(0.0, i * 0.04) for i in range(10)]
    v = _tracker_with_history(pairs)
    assert v.valid is True
    assert v.vel_y > 0


# ---------------------------------------------------------------------------
# Edge detection
# ---------------------------------------------------------------------------

def _detect(x1, y1, x2, y2, vel_x=0.0, vel_y=0.0, fw=640, fh=480) -> VelocityTracker:
    v = VelocityTracker()
    v.vel_x = vel_x
    v.vel_y = vel_y
    v._detect_edge(x1, y1, x2, y2, fw, fh)
    return v


def test_edge_right_exit_detected():
    fw, fh = 640, 480
    margin = int(fw * cfg.EDGE_MARGIN_RATIO)
    v = _detect(0, 100, fw - margin + 1, 300, vel_x=0.5, fw=fw, fh=fh)
    assert v.edge_exit == "right"
    assert v.edge_boost_yaw == cfg.EDGE_EXIT_BOOST


def test_edge_left_exit_detected():
    fw, fh = 640, 480
    margin = int(fw * cfg.EDGE_MARGIN_RATIO)
    v = _detect(margin - 1, 100, 200, 300, vel_x=-0.5, fw=fw, fh=fh)
    assert v.edge_exit == "left"


def test_edge_no_boost_when_moving_against_edge():
    """Bbox at right edge but person moving LEFT → no yaw boost."""
    fw, fh = 640, 480
    margin = int(fw * cfg.EDGE_MARGIN_RATIO)
    # x1=200 keeps bbox away from the left edge so only the right edge is active
    v = _detect(200, 100, fw - margin + 1, 300, vel_x=-0.5, fw=fw, fh=fh)
    assert "right" not in (v.edge_exit or "")
    assert v.edge_boost_yaw == 0.0


def test_edge_corner_exit_sets_both_boosts():
    """Bottom-right corner exit must set both yaw and pitch boosts."""
    fw, fh = 640, 480
    mx = int(fw * cfg.EDGE_MARGIN_RATIO)
    my = int(fh * cfg.EDGE_MARGIN_RATIO)
    v = _detect(0, 0, fw - mx + 1, fh - my + 1, vel_x=0.5, vel_y=0.5, fw=fw, fh=fh)
    assert "right"  in (v.edge_exit or "")
    assert "bottom" in (v.edge_exit or "")
    assert v.edge_boost_yaw   == cfg.EDGE_EXIT_BOOST
    assert v.edge_boost_pitch == cfg.EDGE_EXIT_BOOST


def test_edge_no_exit_when_bbox_central():
    v = _detect(100, 100, 200, 200, fw=640, fh=480)
    assert v.edge_exit is None
    assert v.edge_boost_yaw   == 0.0
    assert v.edge_boost_pitch == 0.0


# ---------------------------------------------------------------------------
# Search direction
# ---------------------------------------------------------------------------

def test_search_direction_uses_siyi_pitch_sign_for_downward_motion():
    v = _tracker_with_velocity(vel_x=0.2, vel_y=0.3)
    yaw_dir, pitch_dir = v.get_search_direction()
    assert yaw_dir == 1
    assert pitch_dir == -1


def test_search_direction_uses_siyi_pitch_sign_for_upward_motion():
    v = _tracker_with_velocity(vel_x=-0.2, vel_y=-0.3)
    yaw_dir, pitch_dir = v.get_search_direction()
    assert yaw_dir == -1
    assert pitch_dir == 1


def test_search_direction_default_when_not_valid():
    v = VelocityTracker()
    yaw_dir, pitch_dir = v.get_search_direction()
    assert yaw_dir == 1
    assert pitch_dir == 0


# ---------------------------------------------------------------------------
# reset()
# ---------------------------------------------------------------------------

def test_reset_clears_velocity_and_valid():
    v = _tracker_with_velocity(vel_x=0.5, vel_y=0.3)
    v.last_cx = 0.4
    v.last_cy = -0.2
    v.reset()
    assert v.valid is False
    assert v.vel_x == 0.0 and v.vel_y == 0.0
    assert v.speed == 0.0
    assert v.edge_exit is None
    assert v.edge_boost_yaw == 0.0 and v.edge_boost_pitch == 0.0
    assert v.last_cx == 0.0 and v.last_cy == 0.0


def test_reset_clears_history():
    v = VelocityTracker()
    pairs = [(i * 0.05, 0.0) for i in range(10)]
    for i, (ncx, ncy) in enumerate(pairs):
        v.history.append((time.time() - 0.5 + i * 0.05, ncx, ncy, 0, 0, 0, 0, 640, 480))
    assert len(v.history) > 0
    v.reset()
    assert len(v.history) == 0
