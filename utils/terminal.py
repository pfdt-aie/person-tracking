"""Thread-safe terminal output with in-place status-line support.

Two functions:
  status(msg)  — overwrites the current terminal line in place (no newline).
                 Use for live data that updates frequently, e.g. [IDs].
  event(msg)   — clears any live status line first, then prints a permanent
                 log line. Use for [State], [Safety], [Cmd] messages.

Plain print() calls from other modules work alongside these — they produce
normal newlines and do not need to be changed.

Both functions fall back to plain print() when stdout is not a TTY (i.e.
when output is piped to a log file) so log files remain fully readable.
"""
from __future__ import annotations

import sys
import threading

_lock   = threading.Lock()
_active = False   # True while a status line is live (cursor not yet at \n)
_tty    = sys.stdout.isatty()


def status(msg: str) -> None:
    """Overwrite the current terminal line with *msg* (no newline).

    Subsequent calls overwrite the same line, keeping the terminal clean
    when a value updates frequently.  When stdout is not a TTY the message
    is printed normally so log files stay readable.
    """
    global _active
    if not _tty:
        print(msg)
        return
    with _lock:
        sys.stdout.write(f"\r\033[K{msg}")
        sys.stdout.flush()
        _active = True


def event(msg: str) -> None:
    """Print a permanent log line, moving past any live status line first.

    If a status() line is currently on screen this inserts a newline before
    printing so the event appears on its own line and the status line is
    preserved above it.
    """
    global _active
    if not _tty:
        print(msg)
        return
    with _lock:
        if _active:
            sys.stdout.write("\n")
            _active = False
        print(msg)


def clear() -> None:
    """Clear the current status line (if any) and leave cursor at column 0."""
    global _active
    if not _tty:
        return
    with _lock:
        if _active:
            sys.stdout.write("\r\033[K")
            sys.stdout.flush()
            _active = False
