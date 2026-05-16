"""B3 — persistent YOLO settings dir.

Pre-fix, main.py pinned YOLO_CONFIG_DIR=/tmp/ultralytics. /tmp is
tmpfs on the Jetson (and most modern Linux distros), wiped on every
reboot, so cold-boot launches produced a noisy 'Creating new
Ultralytics Settings v0.0.6 file' banner. Now we prefer
~/.cache/ultralytics (persistent across reboots) and fall back to
/tmp only if $HOME is unwritable.
"""
import importlib
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))


def _reload_main_module():
    """Import main fresh so the module-level _resolve_ul_dir gets re-run."""
    if "main" in sys.modules:
        del sys.modules["main"]
    import main as _main  # noqa: F401
    return _main


def test_resolves_to_home_cache_when_writable(monkeypatch, tmp_path):
    """Happy path: $HOME exists and is writable → ~/.cache/ultralytics."""
    monkeypatch.setattr(pathlib.Path, "home", lambda: tmp_path)

    # Re-run only the resolver, not the whole main module (which would
    # trigger preflight dep checks etc.).
    import main as _main
    importlib.reload(_main)

    expected = tmp_path / ".cache" / "ultralytics"
    assert _main._ul_dir == expected
    assert expected.exists()


def test_falls_back_to_tmp_when_home_unwritable(monkeypatch):
    """If creating ~/.cache/ultralytics raises, we must still produce a
    usable directory so startup doesn't crash."""
    class _BadHome:
        def __truediv__(self, _other):
            return self
        def mkdir(self, *_a, **_kw):
            raise OSError("read-only home")

    monkeypatch.setattr(pathlib.Path, "home", lambda: _BadHome())

    import main as _main
    importlib.reload(_main)

    assert _main._ul_dir == pathlib.Path("/tmp/ultralytics")
    assert _main._ul_dir.exists()
