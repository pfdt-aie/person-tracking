"""Tests for velocity-derived search hints."""

from tracking.velocity_tracker import VelocityTracker


def test_search_direction_uses_siyi_pitch_sign_for_downward_motion():
    v = VelocityTracker()
    v.valid = True
    v.vel_x = 0.2
    v.vel_y = 0.3

    yaw_dir, pitch_dir = v.get_search_direction()

    assert yaw_dir == 1
    assert pitch_dir == -1


def test_search_direction_uses_siyi_pitch_sign_for_upward_motion():
    v = VelocityTracker()
    v.valid = True
    v.vel_x = -0.2
    v.vel_y = -0.3

    yaw_dir, pitch_dir = v.get_search_direction()

    assert yaw_dir == -1
    assert pitch_dir == 1
