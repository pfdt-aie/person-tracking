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


def test_web_ui_renders_outdoor_only_with_pfwait_class():
    """Sprint F — outdoor-only failures (GPS, HOME) must render with the
    pfwait amber class, not pferr red. Stops the operator's eye from
    treating laws-of-physics items as configuration bugs."""
    html = StreamServer()._build_html().decode()
    # CSS class is defined
    assert ".pfwait" in html
    # JS branch tests outdoor_only and routes to pfwait
    assert "isWaiting = !it.ok && it.outdoor_only" in html
    assert "pfwait" in html
    # The label gets a [WAIT] prefix
    assert "'[WAIT] '" in html


def test_web_ui_outdoor_only_does_not_affect_arm_button_disable():
    """SAFETY — the [WAIT] visual must not affect d.all_ok. The ARM
    button is still disabled if any item is not ok, including outdoor-only
    ones. Check the JS still uses d.all_ok untouched."""
    html = StreamServer()._build_html().decode()
    # The ARM button gating is unchanged.
    assert "document.getElementById('armgo').disabled = !d.all_ok" in html
