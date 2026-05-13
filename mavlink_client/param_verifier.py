"""
param_verifier.py — Verify ArduPilot params required for safe autonomous flight (S2.3).

Each Requirement names a parameter, the comparison rule, and the expected
right-hand-side value. We resolve actuals via MAVLinkClient.fetch_param()
and report any mismatch. The preflight checklist consumes the summary so
the operator can fix the FCU configuration before arming.

Comparison rules:
  ==   exact equality (booleans, modes)
  >=   minimum (radii, altitudes)
  >    strictly positive (timeouts)
  bit  bitmask AND of expected bits must be present

The defaults below match the safety contract documented at the top of
mavlink_client.py — keep both in sync.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import config as cfg


@dataclass(frozen=True)
class Requirement:
    name:     str
    rule:     str       # one of: "==", ">=", ">", "bit"
    expected: float
    note:     str = ""

    def evaluate(self, actual: float | None) -> tuple[bool, str]:
        if actual is None:
            return False, "not advertised by FCU"
        if self.rule == "==":
            ok = (actual == self.expected)
        elif self.rule == ">=":
            ok = (actual >= self.expected)
        elif self.rule == ">":
            ok = (actual > self.expected)
        elif self.rule == "bit":
            ok = (int(actual) & int(self.expected)) == int(self.expected)
        else:
            return False, f"unknown rule {self.rule!r}"
        message = f"{actual} {self.rule} {self.expected}"
        if not ok and self.note:
            message += f" — {self.note}"
        return ok, message


@dataclass(frozen=True)
class ParamCheck:
    name:    str
    ok:      bool
    actual:  float | None
    message: str

    def to_dict(self) -> dict:
        return {"name": self.name, "ok": self.ok,
                "actual": self.actual, "message": self.message}


def default_requirements() -> List[Requirement]:
    """Return the safety-critical ArduPilot params and expected values.

    Values are derived from config.py so changing the app-side bound also
    raises the FCU-side minimum.
    """
    return [
        Requirement("FENCE_ENABLE", "==", 1.0,
                    note="onboard geofence must be enabled"),
        Requirement("FENCE_RADIUS", ">=", float(cfg.GEOFENCE_RADIUS_M),
                    note=f"FCU radius must be ≥ app GEOFENCE_RADIUS_M ({cfg.GEOFENCE_RADIUS_M:.0f} m)"),
        Requirement("FENCE_ALT_MAX", ">=", float(cfg.MAX_ALT_M),
                    note=f"FCU ceiling must be ≥ app MAX_ALT_M ({cfg.MAX_ALT_M:.0f} m)"),
        Requirement("RTL_ALT", ">=", 2000.0,
                    note="RTL altitude in cm — ≥ 20 m above home"),
        Requirement("BATT_FS_LOW_ACT", ">=", 2.0,
                    note="battery low failsafe action must be RTL (2) or LAND (3)"),
        Requirement("FS_GCS_ENABLE", "==", 1.0,
                    note="GCS heartbeat failsafe must be on"),
        Requirement("GUID_TIMEOUT", ">", 0.0,
                    note="GUIDED-mode command timeout must be positive"),
    ]


class ParamVerifier:
    """Query the FCU for safety-critical params and report the comparison.

    Args:
        mav:           MAVLinkClient.
        timeout_per_param: PARAM_VALUE wait timeout per request (seconds).
        requirements:  Override the default requirement list (used in tests).
    """

    def __init__(
        self,
        mav,
        timeout_per_param: float = 2.0,
        requirements: List[Requirement] | None = None,
    ) -> None:
        self._mav = mav
        self._timeout = timeout_per_param
        self._reqs = requirements if requirements is not None else default_requirements()

    def run(self) -> List[ParamCheck]:
        out: List[ParamCheck] = []
        for req in self._reqs:
            try:
                actual = self._mav.fetch_param(req.name, timeout=self._timeout)
            except Exception as exc:
                out.append(ParamCheck(req.name, False, None, f"fetch error: {exc}"))
                continue
            ok, msg = req.evaluate(actual)
            out.append(ParamCheck(req.name, ok, actual, msg))
        return out

    def all_pass(self) -> bool:
        return all(c.ok for c in self.run())
