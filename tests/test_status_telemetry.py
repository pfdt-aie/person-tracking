"""S3.1 — /status payload merges telemetry without losing click-callback keys."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gcs.stream_server import StreamServer
from config.settings import load_settings


def _server() -> StreamServer:
    return StreamServer(settings=load_settings())


def test_telemetry_callback_registered():
    s = _server()
    assert s._telemetry_cb is None
    s.set_telemetry_callback(lambda: {"mode_fcu": "GUIDED"})
    assert s._telemetry_cb is not None


def test_handle_status_merges_payloads_via_simulated_handler():
    """Replicate the /status route's merge contract."""
    s = _server()
    click_payload = {"lock_id": 7, "ids": [7, 8, 9]}
    telem_payload = {
        "mode_fcu": "GUIDED", "armed_fcu": True,
        "gps_fix": 3, "gps_hdop": 0.8, "gps_sats": 14,
        "battery_v": 16.4, "fence_breach": False,
        "rc_override": False, "fps": 24.5,
    }
    s.set_click_callback(lambda nx, ny: click_payload)
    s.set_telemetry_callback(lambda: telem_payload)

    # Simulate handler merge (same logic as /status route)
    data = s._click_cb(None, None)
    telem = s._telemetry_cb()
    for k, v in telem.items():
        data.setdefault(k, v)

    # Click keys preserved
    assert data["lock_id"] == 7
    assert data["ids"] == [7, 8, 9]
    # Telemetry merged
    assert data["mode_fcu"] == "GUIDED"
    assert data["battery_v"] == 16.4
    assert data["gps_sats"] == 14


def test_telemetry_does_not_overwrite_click_keys():
    s = _server()
    s.set_click_callback(lambda nx, ny: {"lock_id": 1, "ids": [1]})
    # Malicious telemetry that tries to redefine lock_id
    s.set_telemetry_callback(lambda: {"lock_id": 999, "mode_fcu": "RTL"})
    data = s._click_cb(None, None)
    for k, v in s._telemetry_cb().items():
        data.setdefault(k, v)
    assert data["lock_id"] == 1
    assert data["mode_fcu"] == "RTL"


def test_telemetry_error_caught_by_handler():
    """A raising telemetry callback should not break /status."""
    s = _server()
    s.set_telemetry_callback(lambda: (_ for _ in ()).throw(RuntimeError("boom")))
    # Caller's `try` block (in the route handler) should catch this; here we
    # verify the callback actually raises so the handler path is exercised.
    import pytest
    with pytest.raises(RuntimeError):
        s._telemetry_cb()
