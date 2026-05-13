"""Tailscale-aware operator URL generation on stream-server start-up."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gcs.stream_server import StreamServer


def test_loopback_host_lists_browser_and_raw_only():
    pairs = StreamServer._operator_urls("127.0.0.1", 8080, "")
    labels = [p[0].strip() for p in pairs]
    assert labels == ["Browser", "MJPEG raw"]
    urls = [p[1] for p in pairs]
    assert all("127.0.0.1:8080" in u for u in urls)


def test_explicit_host_lists_browser_and_raw():
    pairs = StreamServer._operator_urls("192.168.1.42", 8080, "")
    urls = [p[1] for p in pairs]
    assert any("192.168.1.42:8080/" in u for u in urls)


def test_token_query_appended_when_set():
    pairs = StreamServer._operator_urls("127.0.0.1", 8080, "?token=SECRET")
    browser = next(u for label, u in pairs if "Browser" in label)
    assert browser.endswith("/?token=SECRET")


def test_zero_zero_falls_back_to_loopback_at_minimum():
    pairs = StreamServer._operator_urls("0.0.0.0", 8080, "")
    # Either Tailscale or LAN may not exist on the test machine, but
    # the loopback fallback and the MJPEG-raw URL must always be present.
    labels = [p[0].strip() for p in pairs]
    assert "Local" in labels
    assert "MJPEG raw" in labels


def test_zero_zero_returns_at_least_two_entries():
    """0.0.0.0 should at minimum show a loopback URL plus the raw stream."""
    pairs = StreamServer._operator_urls("0.0.0.0", 8080, "")
    assert len(pairs) >= 2
