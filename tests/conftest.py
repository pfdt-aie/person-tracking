"""tests/conftest.py — Shared pytest setup for the person-tracking suite.

Adds the project root to ``sys.path`` once at collection time so test
modules can ``import tracker`` / ``import config`` without each file
repeating its own ``sys.path.insert(...)`` boilerplate.

Existing test files still carry that boilerplate; it is now a no-op
(``insert`` of an already-present path is idempotent). Future tests
need only their normal ``import`` statements.
"""

from __future__ import annotations

import pathlib
import sys

_PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))
