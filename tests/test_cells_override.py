"""S3.4 — battery cell count override and detection confidence."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import pytest

pytest.importorskip("pymavlink")

import config as cfg
from config.settings import load_settings
from mavlink_client.mavlink_client import MAVLinkClient
from safety.safety import SafetyMonitor


def _client(override: int, voltage_mv: int):
    settings = load_settings(cells_override=override)
    safety = SafetyMonitor(settings=settings)
    c = MAVLinkClient(safety=safety, settings=settings)
    c._vbat_mv = voltage_mv
    return c, safety


def test_override_3s_overrides_auto_detect(capsys):
    c, _ = _client(override=3, voltage_mv=16400)   # voltage looks 4S
    c._detect_cell_count()
    assert c._cell_count == 3
    assert c._cell_detected is True
    out = capsys.readouterr().out
    assert "confidence=override" in out
    assert "n=3S" in out


def test_override_4s_takes_priority():
    c, _ = _client(override=4, voltage_mv=14800)
    c._detect_cell_count()
    assert c._cell_count == 4


def test_no_override_auto_detect_4s_high_confidence(capsys):
    c, _ = _client(override=0, voltage_mv=14800)   # exact 4 × 3700
    c._detect_cell_count()
    assert c._cell_count == 4
    out = capsys.readouterr().out
    assert "confidence=high" in out


def test_no_override_ambiguous_voltage_falls_back(capsys):
    """Half-way between 3S and 4S = low confidence → fallback."""
    c, _ = _client(override=0, voltage_mv=13000)   # ratio ~3.51
    c._detect_cell_count()
    assert c._cell_detected is False
    assert c._cell_count == cfg.DEFAULT_CELLS
    out = capsys.readouterr().out
    assert "confidence=low" in out
    assert "Override with --cells" in out


def test_invalid_override_ignored():
    """0 means no override; out-of-range ints are silently treated as 0."""
    c, _ = _client(override=0, voltage_mv=14800)
    c._detect_cell_count()
    assert c._cell_count == 4   # auto-detect, not overridden


def test_override_5s_logs_correct_count(capsys):
    c, _ = _client(override=5, voltage_mv=18000)
    c._detect_cell_count()
    out = capsys.readouterr().out
    assert "n=5S" in out


def test_ambiguous_message_prints_once_only(capsys):
    """Ambiguous detection used to fire every SYS_STATUS (~2 Hz) — 70+
    duplicate lines per session. Now it should only print on the first
    occurrence and stay silent after."""
    c, _ = _client(override=0, voltage_mv=13000)   # ambiguous ratio ~3.51

    for _ in range(50):
        c._detect_cell_count()

    out = capsys.readouterr().out
    occurrences = out.count("Battery cell detection ambiguous")
    assert occurrences == 1, f"expected 1 ambiguous warning, got {occurrences}"
    assert "suppressing further ambiguity warnings" in out
