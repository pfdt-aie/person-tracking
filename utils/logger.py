"""
logger.py — Session logging: tee stdout/stderr to a timestamped log file.

Usage:
    from utils.logger import setup_log, teardown_log

    log_path, orig_stdout, orig_stderr, log_file = setup_log(log_dir)
    # ... run program ...
    teardown_log(orig_stdout, orig_stderr, log_file)
"""

import os
import sys
import threading
from datetime import datetime


class _Tee:
    """Mirrors writes to both the original stream (terminal) and a log file.

    Both streams share the same underlying file handle so interleaved
    stdout/stderr writes appear in the correct order in the log.
    A shared threading.Lock prevents concurrent print() calls from
    interleaving lines inside the log file.
    """

    def __init__(self, stream, log_file, lock: threading.Lock) -> None:
        self._stream   = stream
        self._log_file = log_file
        self._lock     = lock

    def write(self, msg: str) -> int:
        with self._lock:
            self._stream.write(msg)
            self._log_file.write(msg)
        return len(msg)

    def flush(self) -> None:
        with self._lock:
            self._stream.flush()
            try:
                self._log_file.flush()
            except Exception:
                pass

    def fileno(self) -> int:
        return self._stream.fileno()

    def isatty(self) -> bool:
        # Reflect the actual terminal so terminal.status() can overwrite the
        # same line in-place when running interactively.  The log file always
        # receives every line regardless (write() never skips).
        return self._stream.isatty()


def setup_log(log_dir: str) -> tuple:
    """Create a timestamped log file and return tee streams.

    Returns:
        (log_path, orig_stdout, orig_stderr, log_file)

    The caller is responsible for restoring sys.stdout / sys.stderr and
    closing log_file when done.
    """
    os.makedirs(log_dir, exist_ok=True)
    ts       = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_path = os.path.join(log_dir, f"gimbal_track_{ts}.log")

    log_file   = open(log_path, "w", encoding="utf-8", buffering=1)
    tee_lock   = threading.Lock()
    orig_out   = sys.stdout
    orig_err   = sys.stderr
    sys.stdout = _Tee(orig_out, log_file, tee_lock)
    sys.stderr = _Tee(orig_err, log_file, tee_lock)
    # Give terminal.log_only() a direct handle to the log file so it can
    # write debug lines that go to the file but NOT to the terminal.
    from utils import terminal as _t
    _t.set_log_file(log_file)
    return log_path, orig_out, orig_err, log_file


def teardown_log(orig_stdout, orig_stderr, log_file, log_path: str = "") -> None:
    """Restore stdout/stderr and close the log file."""
    sys.stdout = orig_stdout
    sys.stderr = orig_stderr
    log_file.close()
    if log_path:
        print(f"[Log] Saved → {log_path}")
