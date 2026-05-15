"""
flight_log.py — Structured JSONL flight-event logger (S3.2).

Purpose
-------
Capture safety-relevant events as one JSON object per line so a
post-incident review can replay what happened without parsing free-text
terminal output. Used alongside ``utils/logger.py`` which captures
stdout — this module logs *events*, the other logs *text*.

Schema
------
Every line is a JSON object with at minimum:

    { "t": <ISO-8601 with timezone>,
      "mono": <monotonic seconds since process start>,
      "event": <short snake_case name>,
      ... arbitrary key/value payload ... }

Typical event names: ``mode_change``, ``estop``, ``rtl``,
``geofence_breach``, ``preflight_arm``, ``rc_override``,
``tracking_loss``, ``fps_floor``.

Thread safety
-------------
A single ``threading.Lock`` serialises writes. ``event()`` is safe to
call from any thread. The file is opened in line-buffered mode so a
crash mid-flight still flushes complete JSON objects.

Singleton
---------
``init_flight_log(dir)`` is called once from ``main.py``;
``get_flight_log()`` returns the active logger (or a no-op stub before
init so library code doesn't have to None-check).
"""

from __future__ import annotations

import json
import os
import threading
import time
from datetime import datetime, timezone
from typing import Any, Optional


class _NullLog:
    """No-op fallback used until ``init_flight_log()`` has been called."""

    def event(self, name: str, **fields: Any) -> None:    # noqa: D401
        pass

    def close(self) -> None:
        pass

    @property
    def path(self) -> Optional[str]:
        return None


class FlightLog:
    """Append-only JSONL logger for flight-safety events.

    Args:
        log_dir: Directory in which to create the per-session log file.
                 Created if it does not exist.
        prefix:  Filename prefix; the session timestamp is appended.
    """

    def __init__(self, log_dir: str, prefix: str = "flight") -> None:
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = os.path.join(log_dir, f"{prefix}_{stamp}.jsonl")
        # Line-buffered text mode so each completed event is flushed.
        self._fh = open(self._path, "a", buffering=1, encoding="utf-8")
        self._lock = threading.Lock()
        self._t0  = time.monotonic()
        self.event("log_open", path=self._path)

    @property
    def path(self) -> str:
        return self._path

    def event(self, name: str, **fields: Any) -> None:
        """Append a single event line.

        Args:
            name:   short snake_case event name (e.g. ``"estop"``).
            fields: arbitrary JSON-serialisable kwargs.
        """
        record = {
            "t":     datetime.now(timezone.utc).isoformat(),
            "mono":  round(time.monotonic() - self._t0, 4),
            "event": name,
        }
        # Caller's fields override the standard keys only if they really
        # mean to — pydantic-style merge would hide bugs.
        for k, v in fields.items():
            if k in ("t", "mono", "event"):
                continue
            record[k] = v
        try:
            line = json.dumps(record, default=str)
        except TypeError:
            line = json.dumps({"t": record["t"], "mono": record["mono"],
                               "event": "log_error",
                               "orig": name,
                               "reason": "non-serialisable payload"})
        with self._lock:
            try:
                self._fh.write(line + "\n")
            except Exception:
                # Never let logging take down the flight loop.
                pass

    def close(self) -> None:
        with self._lock:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
#  Module-level singleton
# ---------------------------------------------------------------------------

_active: "FlightLog | _NullLog" = _NullLog()


def init_flight_log(log_dir: str, prefix: str = "flight") -> FlightLog:
    """Initialise the process-wide flight log. Safe to call once.

    Subsequent calls close the previous log and replace it — useful for
    tests but not expected in production.
    """
    global _active
    if isinstance(_active, FlightLog):
        _active.close()
    _active = FlightLog(log_dir, prefix=prefix)
    return _active


def get_flight_log() -> "FlightLog | _NullLog":
    """Return the active flight log (or a no-op stub if not yet inited)."""
    return _active


def safe_event(name: str, **fields: Any) -> None:
    """Append an event line; never raise.

    Thin wrapper around ``get_flight_log().event()`` for call sites in the
    flight loop, RX threads, and error handlers where a logging failure
    must not propagate. Tests monkeypatch ``utils.flight_log.get_flight_log``
    so the lookup happens at call time, not import time.
    """
    try:
        get_flight_log().event(name, **fields)
    except Exception:
        pass


def close_flight_log() -> None:
    """Close the singleton log (used by main.py teardown)."""
    global _active
    if isinstance(_active, FlightLog):
        _active.close()
    _active = _NullLog()
