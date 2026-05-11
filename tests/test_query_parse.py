"""tests/test_query_parse.py — urllib.parse.parse_qs correctness (P2-1 coverage)."""
import urllib.parse


def _parse(qs: str) -> dict:
    return urllib.parse.parse_qs(qs, keep_blank_values=False)


def test_simple_key_value():
    kv = _parse("dir=left")
    assert kv.get("dir", ["stop"])[0] == "left"


def test_multiple_params():
    kv = _parse("x=0.5&y=0.3")
    assert float(kv.get("x", [""])[0]) == 0.5
    assert float(kv.get("y", [""])[0]) == 0.3


def test_missing_key_returns_default():
    kv = _parse("")
    assert kv.get("dir", ["stop"])[0] == "stop"
    assert kv.get("set", [None])[0] is None


def test_url_encoded_value():
    kv = _parse("mode=AUTO%20GUIDED")
    assert kv.get("mode", [""])[0] == "AUTO GUIDED"


def test_value_containing_equals():
    # parse_qs handles base64-style values with = correctly
    kv = _parse("token=abc%3Ddef")
    assert kv.get("token", [""])[0] == "abc=def"


def test_malformed_no_equals():
    # A bare key with no = should not crash and not appear
    kv = _parse("novalue")
    assert "novalue" not in kv


def test_quality_lo():
    kv = _parse("q=lo")
    quality = "lo" if kv.get("q", ["hi"])[0].lower() == "lo" else "hi"
    assert quality == "lo"


def test_quality_default_hi():
    kv = _parse("")
    quality = "lo" if kv.get("q", ["hi"])[0].lower() == "lo" else "hi"
    assert quality == "hi"


def test_float_conversion_empty_raises():
    kv = _parse("")
    try:
        float(kv.get("x", [""])[0])
        assert False, "Should have raised"
    except ValueError:
        pass
