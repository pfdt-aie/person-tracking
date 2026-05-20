"""
person_geolocation.py — Person GPS estimation from camera + drone state.

Two components:
  CameraGeolocation : Projects a bounding-box pixel coordinate through the
                      camera model, gimbal angles, and drone attitude to
                      produce an estimated person GPS position.

  PersonEKF         : 4-state Extended Kalman Filter that smooths the noisy
                      GPS estimates and predicts position during occlusion.
                      State: [pos_N, pos_E, vel_N, vel_E] (metres, m/s).

Research basis:
  - SMART-TRACK (arxiv 2410.10409): EKF reduces position RMSE from 3–5 m
    to < 0.5 m; prediction enables ROI search during occlusion.
  - Active Object Detection — MDPI Drones 2024: ray-ground projection with
    bounding-box FEET (lower-edge centre) reduces error by ~0.9 m vs centroid.

Key safety note: use bounding-box FEET, not centroid.
At 15 m altitude / –45° tilt, using the centroid introduces ~0.9 m of
upward projection error (estimating position of person's chest, not feet).
"""

import math
from typing import Optional

import numpy as np

import config as cfg


EARTH_RADIUS_M: float = 6_371_000.0


# ---------------------------------------------------------------------------
#  Camera Geolocation
# ---------------------------------------------------------------------------

class CameraGeolocation:
    """Projects bounding-box pixel coordinates to estimated GPS position.

    Intrinsics are either loaded from a calibration YAML or auto-estimated
    from the SIYI A8 mini published HFOV (81°).  YAML always takes priority.

    Args:
        calib_yaml: Path to camera calibration YAML (OpenCV format).
                    Pass "" or None to use auto-estimation.
    """

    def __init__(self, calib_yaml: str = cfg.CALIBRATION_YAML) -> None:
        self._fx: float = 0.0
        self._fy: float = 0.0
        self._cx: float = 0.0
        self._cy: float = 0.0
        self._calibrated: bool = False

        if calib_yaml:
            self._load_yaml(calib_yaml)

    # ------------------------------------------------------------------
    #  Intrinsic setup
    # ------------------------------------------------------------------

    def _load_yaml(self, path: str) -> None:
        """Load camera matrix from OpenCV YAML calibration file."""
        try:
            import cv2  # type: ignore
            fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
            K  = fs.getNode("camera_matrix").mat()
            fs.release()
            if K is not None and K.shape == (3, 3):
                self._fx         = float(K[0, 0])
                self._fy         = float(K[1, 1])
                self._cx         = float(K[0, 2])
                self._cy         = float(K[1, 2])
                self._calibrated = True
                print(
                    f"[Geo] Calibration loaded: "
                    f"fx={self._fx:.1f}  fy={self._fy:.1f}  "
                    f"cx={self._cx:.1f}  cy={self._cy:.1f}"
                )
        except Exception as exc:
            print(f"[Geo] YAML load failed ({exc}) — using auto-estimate")

    def _ensure_intrinsics(self, frame_w: int, frame_h: int) -> None:
        """Auto-estimate intrinsics from SIYI A8 mini HFOV if not calibrated."""
        if self._calibrated and self._fx > 0:
            return
        hfov_rad   = math.radians(cfg.SIYI_A8_HFOV_DEG)
        self._fx   = (frame_w / 2.0) / math.tan(hfov_rad / 2.0)
        self._fy   = self._fx           # assume square pixels
        self._cx   = frame_w  / 2.0
        self._cy   = frame_h  / 2.0

    # ------------------------------------------------------------------
    #  Rotation helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _Rx(a: float) -> np.ndarray:
        """Rotation matrix about X axis."""
        c, s = math.cos(a), math.sin(a)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)

    @staticmethod
    def _Ry(a: float) -> np.ndarray:
        """Rotation matrix about Y axis."""
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)

    @staticmethod
    def _Rz(a: float) -> np.ndarray:
        """Rotation matrix about Z axis (yaw in NED = right-hand, Z down)."""
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)

    def _body_to_ned(
        self, roll: float, pitch: float, yaw: float
    ) -> np.ndarray:
        """ZYX Euler → body-to-NED rotation matrix.

        Args:
            roll, pitch, yaw: Drone attitude in radians.
        """
        return self._Rz(yaw) @ self._Ry(pitch) @ self._Rx(roll)

    # ------------------------------------------------------------------
    #  Main projection
    # ------------------------------------------------------------------

    def project(
        self,
        bbox_x1: float, bbox_y1: float,
        bbox_x2: float, bbox_y2: float,
        frame_w: int,   frame_h: int,
        drone_lat: float, drone_lon: float, drone_alt_agl: float,
        drone_roll: float, drone_pitch: float, drone_yaw: float,
        gimbal_pan_rad: float, gimbal_tilt_rad: float,
    ) -> Optional[tuple[float, float]]:
        """Estimate person GPS position from bounding box + drone/gimbal state.

        Uses the bounding-box LOWER-EDGE CENTRE (person's feet) as the
        measurement point — reduces projection error vs. centroid by ~0.9 m
        at 15 m altitude.

        Args:
            bbox_x1–y2:       Bounding box pixel corners.
            frame_w/h:        Frame resolution.
            drone_lat/lon:    Drone GPS position (degrees).
            drone_alt_agl:    Drone altitude above ground level (m).
            drone_roll/pitch/yaw: Drone attitude (radians).
            gimbal_pan_rad:   Gimbal pan angle (radians; + = right).
            gimbal_tilt_rad:  Gimbal tilt angle (radians; − = looking down).

        Returns:
            (person_lat, person_lon) in degrees, or None if projection fails
            (e.g. ray points upward away from ground).
        """
        if drone_alt_agl <= 0.5:
            return None   # too low — ray-ground projection unreliable below 0.5 m

        self._ensure_intrinsics(frame_w, frame_h)

        # 1. Bounding-box feet pixel (lower-edge centre)
        px = (bbox_x1 + bbox_x2) / 2.0
        py = bbox_y2   # bottom of box = feet

        # 2. Pixel → normalised camera ray
        ray_cam = np.array([
            (px  - self._cx) / self._fx,
            (py  - self._cy) / self._fy,
            1.0,
        ], dtype=float)
        ray_cam /= np.linalg.norm(ray_cam)

        # 3. Camera → gimbal frame (camera aligned with gimbal, no rotation)
        ray_gimbal = ray_cam

        # 4. Gimbal → body frame
        # SIYI convention: pan = Z rotation, tilt = Y rotation
        R_g2b  = self._Rz(gimbal_pan_rad) @ self._Ry(-gimbal_tilt_rad)
        ray_body = R_g2b @ ray_gimbal

        # 5. Body → NED frame
        R_b2ned = self._body_to_ned(drone_roll, drone_pitch, drone_yaw)
        ray_ned = R_b2ned @ ray_body

        # 6. Ray–ground intersection (flat earth, ground at z=0 NED)
        # In NED, positive Z = down. Ray must be pointing down (ray_ned[2] > 0).
        if ray_ned[2] <= 0.01:
            return None   # ray pointing up or horizontal — cannot intersect ground

        t         = drone_alt_agl / ray_ned[2]
        offset_n  = ray_ned[0] * t   # metres north
        offset_e  = ray_ned[1] * t   # metres east

        # 7. NED offsets → GPS (flat-earth approximation, valid < 10 km)
        person_lat = drone_lat + math.degrees(offset_n / EARTH_RADIUS_M)
        person_lon = drone_lon + math.degrees(
            offset_e / (EARTH_RADIUS_M * math.cos(math.radians(drone_lat)))
        )
        return person_lat, person_lon


# ---------------------------------------------------------------------------
#  Person EKF
# ---------------------------------------------------------------------------

class PersonEKF:
    """4-state Extended Kalman Filter for person position and velocity.

    State vector: x = [pos_N, pos_E, vel_N, vel_E]  (metres and m/s)

    Process model: constant velocity
        x(k+1) = F(dt) @ x(k) + process_noise

    Measurement model: observe position only
        z = H @ x + measurement_noise

    Noise tuning (from config.py):
        Q = diag(EKF_PROCESS_NOISE)   default [0.1, 0.1, 0.5, 0.5]
        R = diag(EKF_MEAS_NOISE)      default [2.0, 2.0] (metres²)

    Gate: Mahalanobis distance > EKF_GATE_SIGMA σ → measurement rejected.
    This prevents YOLO false detections or GPS jumps from corrupting the filter.
    """

    def __init__(self) -> None:
        n = 4
        self._x  = np.zeros(n, dtype=float)          # state estimate
        self._P  = np.eye(n, dtype=float) * 100.0    # large initial covariance
        self._Q  = np.diag(cfg.EKF_PROCESS_NOISE).astype(float)
        self._R  = np.diag(cfg.EKF_MEAS_NOISE).astype(float)
        self._H  = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        self._initialised = False

    # ------------------------------------------------------------------
    #  Public interface
    # ------------------------------------------------------------------

    def predict(self, dt: float) -> None:
        """Propagate state forward by dt seconds (constant-velocity model).

        Args:
            dt: Time since last predict() call (seconds).
        """
        if not self._initialised:
            return
        F   = np.array([
            [1, 0, dt, 0],
            [0, 1, 0,  dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=float)
        self._x = F @ self._x
        self._P = F @ self._P @ F.T + self._Q

    def update(self, meas_n: float, meas_e: float) -> bool:
        """Incorporate a new position measurement.

        Args:
            meas_n: Measured person position North (metres from EKF origin).
            meas_e: Measured person position East  (metres from EKF origin).

        Returns:
            True if the measurement was accepted (within Mahalanobis gate).
            False if rejected as an outlier.
        """
        z = np.array([meas_n, meas_e], dtype=float)

        if not self._initialised:
            # First measurement: initialise state directly
            self._x[:] = [meas_n, meas_e, 0.0, 0.0]
            self._P    = np.eye(4, dtype=float) * 10.0
            self._initialised = True
            return True

        # S2.4 — Euclidean jump rejection. Catches teleport-style errors
        # (re-ID swap to a different person, projection glitch) BEFORE the
        # Mahalanobis gate, which can be defeated by inflated covariance
        # after a long predict-only run.
        dx = meas_n - float(self._x[0])
        dy = meas_e - float(self._x[1])
        jump = math.hypot(dx, dy)
        if jump > cfg.EKF_MAX_JUMP_M:
            from utils import terminal as _t
            _t.log_only(f"[EKF] Position jump rejected: {jump:.1f}m "
                        f"(limit {cfg.EKF_MAX_JUMP_M:.1f}m) "
                        f"— possible re-ID swap or projection glitch")
            return False

        # Innovation
        innov = z - self._H @ self._x
        S     = self._H @ self._P @ self._H.T + self._R

        # Mahalanobis gate
        try:
            S_inv   = np.linalg.inv(S)
            mah_sq  = float(innov @ S_inv @ innov)
            gate_sq = cfg.EKF_GATE_SIGMA ** 2
            if mah_sq > gate_sq:
                from utils import terminal as _t
                _t.log_only(f"[EKF] Mahalanobis gate rejected: mah²={mah_sq:.1f} > {gate_sq:.1f} "
                            f"— noisy projection or GPS error")
                return False
        except np.linalg.LinAlgError:
            return False

        # Kalman gain and state update
        K      = self._P @ self._H.T @ S_inv
        self._x = self._x + K @ innov
        I       = np.eye(4, dtype=float)
        self._P = (I - K @ self._H) @ self._P
        return True

    def get_state(self) -> tuple[float, float, float, float]:
        """Return (pos_N, pos_E, vel_N, vel_E) current estimate."""
        return (
            float(self._x[0]),
            float(self._x[1]),
            float(self._x[2]),
            float(self._x[3]),
        )

    def get_position_ned(self) -> tuple[float, float]:
        """Return (pos_N, pos_E) in metres from EKF origin."""
        return float(self._x[0]), float(self._x[1])

    def get_velocity_ned(self) -> tuple[float, float]:
        """Return estimated (vel_N, vel_E) in m/s."""
        return float(self._x[2]), float(self._x[3])

    def gps_to_ned(
        self,
        lat: float, lon: float,
        origin_lat: float, origin_lon: float,
    ) -> tuple[float, float]:
        """Convert GPS coordinates to NED offsets from an origin point.

        Uses flat-earth approximation (valid < 10 km).

        Args:
            lat, lon:           Target GPS position (degrees).
            origin_lat/lon:     NED origin GPS position (degrees).

        Returns:
            (offset_north_m, offset_east_m)
        """
        dn = math.radians(lat - origin_lat) * EARTH_RADIUS_M
        de = (
            math.radians(lon - origin_lon)
            * EARTH_RADIUS_M
            * math.cos(math.radians(origin_lat))
        )
        return dn, de

    def ned_to_gps(
        self,
        n: float, e: float,
        origin_lat: float, origin_lon: float,
    ) -> tuple[float, float]:
        """Convert NED offsets back to GPS coordinates.

        Args:
            n, e:               NED offsets (metres).
            origin_lat/lon:     NED origin (degrees).

        Returns:
            (lat_deg, lon_deg)
        """
        lat = origin_lat + math.degrees(n / EARTH_RADIUS_M)
        lon = origin_lon + math.degrees(
            e / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat)))
        )
        return lat, lon

    def reset(self) -> None:
        """Reset filter state (call when person is re-acquired after long loss)."""
        n = 4
        self._x           = np.zeros(n, dtype=float)
        self._P           = np.eye(n, dtype=float) * 100.0
        self._initialised = False

    @property
    def is_valid(self) -> bool:
        """True after the first measurement has been incorporated."""
        return self._initialised
