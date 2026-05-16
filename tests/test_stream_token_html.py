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


def test_stream_html_renders_preflight_breakdown_on_semicolon():
    """Sprint E — the web UI must split multi-part preflight messages
    (joined by '; ' in _check_params) into separate indented sub-lines
    instead of cramming a ~300-char string into a single span. Mirrors
    the stdin breakdown rendered by
    OperatorInputController._print_preflight_breakdown."""
    html = StreamServer()._build_html().decode()
    # The split logic and the pfdetail row must both be present.
    assert "msg.includes('; ')" in html
    assert "msg.split('; ')" in html
    assert "pfdetail" in html
    # The CSS class must be defined too — without it the sub-lines
    # would render with no visual distinction.
    assert ".pfdetail" in html


def test_preflight_breakdown_only_for_failing_items():
    """An OK item with an empty/short message should not produce
    pfdetail rows — only failing items with explicit ';'-joined detail."""
    html = StreamServer()._build_html().decode()
    # Guard condition prevents pfdetail rows on OK items.
    assert "!it.ok && msg.includes('; ')" in html
