"""Regression tests for stream control token propagation in browser HTML."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import config as cfg
from gcs.stream_server import StreamServer


def test_stream_html_appends_token_to_control_fetches():
    old_token = cfg.STREAM_TOKEN
    try:
        cfg.STREAM_TOKEN = "abc 123"
        html = StreamServer()._build_html().decode()
    finally:
        cfg.STREAM_TOKEN = old_token

    assert 'const AUTH_TOKEN = "abc 123";' in html
    assert "fetch(ctlUrl('/click?x='" in html
    assert "fetch(ctlUrl('/mode?set=' + next))" in html
    assert "fetch(ctlUrl('/mode?set=' + mode))" in html
    assert "fetch(ctlUrl('/gimbal?dir=' + dir))" in html
    assert "fetch(ctlUrl('/mode'))" in html
    assert "Stop Follow" in html
    assert "person_protection" in html
    assert "SAFE TO FOLLOW" in html
    assert "HOLDING" in html
    assert "PILOT ACTION REQUIRED" in html
