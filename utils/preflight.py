"""
preflight.py — Verify required Python packages are installed before startup.

Runs before any third-party import in main.py. If a dependency listed in
requirements.txt is missing from the current interpreter, prints the exact
command to fix it and exits non-zero. Deliberately does NOT attempt to
pip-install at runtime: silent installs on a flight system can drift
versions away from the tested matrix and fail when off-network.
"""

from __future__ import annotations

import re
import sys
from importlib import metadata
from pathlib import Path

# Map of distribution names that differ from the importable module name.
# Only needed if we ever fall back to import-based checks; metadata.distribution()
# uses the PyPI/dist name directly so this is informational.
_REQ_SPLIT_RE = re.compile(r"[<>=!~;\s]")


def _parse_requirements(requirements_path: Path) -> list[str]:
    """Return PyPI distribution names from a requirements.txt (no version specs).

    Skips lines marked with a trailing `# preflight: skip` comment so dev/CI-only
    tools (lint, test) don't block flight startup.
    """
    names: list[str] = []
    for raw in requirements_path.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or line.startswith("-"):
            continue
        if "preflight: skip" in line:
            continue
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        name = _REQ_SPLIT_RE.split(line, 1)[0].strip()
        if name:
            names.append(name)
    return names


def check_dependencies(requirements_path: Path) -> list[str]:
    """Return the list of declared dependencies that are NOT installed."""
    missing: list[str] = []
    for name in _parse_requirements(requirements_path):
        try:
            metadata.distribution(name)
        except metadata.PackageNotFoundError:
            missing.append(name)
    return missing


def enforce_dependencies(requirements_path: Path) -> None:
    """Exit non-zero with a clear operator message if any dependency is missing."""
    if not requirements_path.exists():
        # No manifest — nothing to enforce. Don't block startup.
        return
    missing = check_dependencies(requirements_path)
    if not missing:
        return

    venv_hint = sys.prefix
    print("=" * 62, file=sys.stderr)
    print("  PREFLIGHT FAILURE — missing Python dependencies", file=sys.stderr)
    print("=" * 62, file=sys.stderr)
    print(f"  Interpreter : {sys.executable}", file=sys.stderr)
    print(f"  Venv prefix : {venv_hint}", file=sys.stderr)
    print(f"  Missing     : {', '.join(missing)}", file=sys.stderr)
    print("", file=sys.stderr)
    print("  Fix (run inside the same venv):", file=sys.stderr)
    print(f"    pip install -r {requirements_path}", file=sys.stderr)
    print("", file=sys.stderr)
    print("  Auto-install at runtime is intentionally disabled on this", file=sys.stderr)
    print("  flight system to prevent silent version drift and off-network", file=sys.stderr)
    print("  failures. Install once, then re-launch.", file=sys.stderr)
    print("=" * 62, file=sys.stderr)
    sys.exit(2)
