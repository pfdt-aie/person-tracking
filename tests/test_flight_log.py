"""S3.2 — JSONL flight log."""
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import pytest

from utils.flight_log import (
    FlightLog, init_flight_log, get_flight_log, close_flight_log, _NullLog,
)


def _read_jsonl(path: str) -> list:
    out = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def test_writes_jsonl(tmp_path):
    log = FlightLog(str(tmp_path))
    log.event("test_event", value=42, label="hello")
    log.close()
    records = _read_jsonl(log.path)
    # First record is log_open, second is the test event
    assert records[0]["event"] == "log_open"
    assert records[1]["event"] == "test_event"
    assert records[1]["value"] == 42
    assert records[1]["label"] == "hello"


def test_every_record_has_t_mono_event(tmp_path):
    log = FlightLog(str(tmp_path))
    log.event("a")
    log.event("b", x=1)
    log.close()
    for rec in _read_jsonl(log.path):
        assert "t" in rec
        assert "mono" in rec
        assert "event" in rec


def test_null_log_before_init():
    close_flight_log()           # ensure clean slate
    log = get_flight_log()
    assert isinstance(log, _NullLog)
    # Must accept event() without raising
    log.event("anything", foo="bar")


def test_init_replaces_previous_log(tmp_path):
    init_flight_log(str(tmp_path))
    first = get_flight_log()
    init_flight_log(str(tmp_path))
    second = get_flight_log()
    assert first is not second
    close_flight_log()


def test_filename_includes_session_stamp(tmp_path):
    log = FlightLog(str(tmp_path))
    name = pathlib.Path(log.path).name
    log.close()
    assert name.startswith("flight_")
    assert name.endswith(".jsonl")


def test_non_serialisable_payload_does_not_crash(tmp_path):
    log = FlightLog(str(tmp_path))
    class _Weird:
        def __repr__(self): return "<weird>"
    log.event("ok_event", obj=_Weird())   # falls back to str()
    log.close()
    records = _read_jsonl(log.path)
    assert any(r["event"] == "ok_event" for r in records)


def test_thread_safety_no_interleaved_lines(tmp_path):
    import threading
    log = FlightLog(str(tmp_path))
    def burst():
        for _ in range(200):
            log.event("burst", n=1)
    threads = [threading.Thread(target=burst) for _ in range(4)]
    for t in threads: t.start()
    for t in threads: t.join()
    log.close()
    # Every line must parse cleanly — interleaved writes would corrupt JSON.
    records = _read_jsonl(log.path)
    assert len(records) >= 800   # 4 × 200 + log_open
