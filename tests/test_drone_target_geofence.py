"""Regression tests for app-level commanded-target geofence checks."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from config.settings import load_settings
from control.drone_controller import DroneController
from safety.safety import GeofenceCircle, SafetyMonitor
from tracking.person_geolocation import PersonEKF


def _controller() -> DroneController:
    safety = SafetyMonitor(
        geofence=GeofenceCircle(
            centre_lat=51.5,
            centre_lon=-0.1,
            radius_m=50.0,
            alt_min_m=10.0,
            alt_max_m=80.0,
        )
    )
    controller = DroneController.__new__(DroneController)
    controller._s = load_settings()
    controller._safety = safety
    controller._ekf = PersonEKF()
    controller._origin_set = True
    controller._origin_lat = 51.5
    controller._origin_lon = -0.1
    controller._target_fence_warn_t = 0.0
    return controller


def test_command_target_inside_geofence_allowed():
    controller = _controller()

    assert controller._target_within_geofence(10.0, 0.0, 20.0) is True


def test_command_target_outside_geofence_rejected():
    controller = _controller()

    assert controller._target_within_geofence(100.0, 0.0, 20.0) is False
