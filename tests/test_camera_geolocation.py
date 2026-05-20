"""CameraGeolocation.project() — ray-ground projection correctness.

Ground truth: at 7 m AGL, drone level (zero attitude), heading North (yaw=0).
  tilt=-T deg (T > 0 = looking down), pan=0  →  person is at N = 7/tan(T) m, E = 0
  pan=+90, tilt=-45                           →  person is at N = 0, E = 7 m
  pixel right of centre by Δx                →  East offset proportional to Δx/fx
  feet pixel below centre by Δy              →  person closer (N < 7 m), E unchanged
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from tracking.person_geolocation import CameraGeolocation

_R = 6_371_000.0
_TOL = 0.15   # metres — acceptable projection error


def _project(tilt_deg, pan_deg, px_off=0, py_off=0, alt=7.0):
    """Project a single pixel through the fixed camera model.

    px_off / py_off are offsets from image centre in pixels.
    The bbox is constructed so that feet (y2) land at cy + py_off.
    Returns (N_metres, E_metres) relative to drone position, or None.
    """
    geo = CameraGeolocation("")   # no YAML → auto intrinsics from HFOV
    cx, cy = 640, 360
    x1 = cx + px_off - 10
    x2 = cx + px_off + 10
    y1 = cy + py_off - 80
    y2 = cy + py_off        # feet at cy + py_off

    result = geo.project(
        x1, y1, x2, y2, 1280, 720,
        0.0, 0.0, alt,                # drone at lat=0, lon=0, alt_agl=alt
        0.0, 0.0, 0.0,                # drone level, heading North
        math.radians(pan_deg), math.radians(tilt_deg),
    )
    if result is None:
        return None
    lat, lon = result
    N = lat * (math.pi / 180.0) * _R
    E = lon * (math.pi / 180.0) * _R
    return N, E


# ---------------------------------------------------------------------------
# Tilt angle ground truth
# ---------------------------------------------------------------------------

def test_tilt_minus_45_center_pixel():
    r = _project(-45, 0)
    assert r is not None
    N, E = r
    assert abs(N - 7.0) < _TOL, f"N={N:.3f}, expected 7.0"
    assert abs(E) < _TOL, f"E={E:.3f}, expected 0"


def test_tilt_minus_10_center_pixel():
    # Near-horizontal: person is far away
    expected_N = 7.0 / math.tan(math.radians(10))   # ~39.7 m
    r = _project(-10, 0)
    assert r is not None
    N, E = r
    assert abs(N - expected_N) < _TOL, f"N={N:.3f}, expected {expected_N:.3f}"
    assert abs(E) < _TOL


def test_tilt_minus_80_center_pixel():
    # Steep down: person is close
    expected_N = 7.0 / math.tan(math.radians(80))   # ~1.23 m
    r = _project(-80, 0)
    assert r is not None
    N, E = r
    assert abs(N - expected_N) < _TOL, f"N={N:.3f}, expected {expected_N:.3f}"
    assert abs(E) < _TOL


# ---------------------------------------------------------------------------
# Pan angle ground truth
# ---------------------------------------------------------------------------

def test_pan_plus90_tilt_minus45():
    r = _project(-45, 90)
    assert r is not None
    N, E = r
    assert abs(N) < _TOL, f"N={N:.3f}, expected 0"
    assert abs(E - 7.0) < _TOL, f"E={E:.3f}, expected 7.0"


def test_pan_minus90_tilt_minus45():
    r = _project(-45, -90)
    assert r is not None
    N, E = r
    assert abs(N) < _TOL
    assert abs(E + 7.0) < _TOL, f"E={E:.3f}, expected -7.0"


def test_pan_180_tilt_minus45():
    r = _project(-45, 180)
    assert r is not None
    N, E = r
    assert abs(N + 7.0) < _TOL, f"N={N:.3f}, expected -7.0"
    assert abs(E) < _TOL


# ---------------------------------------------------------------------------
# Off-centre pixel direction tests
# ---------------------------------------------------------------------------

def test_rightward_pixel_produces_east_offset():
    """A pixel right of centre must produce a positive East offset, not North."""
    r_center = _project(-45, 0, px_off=0)
    r_right  = _project(-45, 0, px_off=100)
    assert r_center is not None and r_right is not None
    N_c, E_c = r_center
    N_r, E_r = r_right
    # East increases for rightward pixel
    assert E_r > E_c + 0.5, f"Right pixel: E={E_r:.3f} not > E_center={E_c:.3f}"
    # North should remain approximately unchanged
    assert abs(N_r - N_c) < 0.5, f"Right pixel: N changed by {abs(N_r-N_c):.3f}m"


def test_downward_feet_pixel_produces_closer_north():
    """Feet below image centre mean the person is closer — N should decrease."""
    r_center = _project(-45, 0, py_off=0)
    r_below  = _project(-45, 0, py_off=100)
    assert r_center is not None and r_below is not None
    N_c, E_c = r_center
    N_b, E_b = r_below
    # Person closer → smaller N
    assert N_b < N_c - 0.5, f"Feet below centre: N={N_b:.3f} not < N_center={N_c:.3f}"
    # No spurious East offset
    assert abs(E_b) < _TOL, f"Feet below centre: spurious E={E_b:.3f}"


# ---------------------------------------------------------------------------
# Safety guards
# ---------------------------------------------------------------------------

def test_returns_none_below_minimum_altitude():
    r = _project(-45, 0, alt=0.4)
    assert r is None


def test_returns_none_horizontal_ray():
    # tilt=0 → horizontal → ray does not intersect ground
    r = _project(0, 0)
    assert r is None
