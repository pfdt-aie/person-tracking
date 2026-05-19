"""
preflight.py — Preflight readiness checklist (S1.3).

Aggregates every gate that must be green before the operator is permitted
to arm the drone-body tracker. Runs against the live MAVLinkClient +
SafetyMonitor so the answers reflect actual telemetry, not configuration
assumptions.

Each check returns a (name, ok, message) triple. The web UI consumes the
full list so the operator can see which item is failing.

Design notes:
  - Pure read-only — never mutates state.
  - Returns deterministic ordering so the UI can render a stable list.
  - Adds no new threads; called inline from the HTTP request thread.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional

import config as cfg


@dataclass(frozen=True)
class PreflightItem:
    name:    str
    ok:      bool
    message: str
    outdoor_only: bool = False   # True when this gate cannot pass indoors
                                 # (GPS lock, HOME position). The arm gate
                                 # STILL requires ok=True; this flag is for
                                 # the UI/stdin to mark indoor-expected
                                 # failures as 'waiting for GPS' rather
                                 # than coloring them like a real bug.

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "ok": self.ok,
            "message": self.message,
            "outdoor_only": self.outdoor_only,
        }


class PreflightCheck:
    """Compute the preflight checklist on demand.

    Args:
        mav:    MAVLinkClient instance.
        safety: SafetyMonitor instance.
        ground_test: True when --ground-test is set; some gates relax (e.g.
                     ARMED is not required because TX is suppressed anyway).
    """

    def __init__(
        self,
        mav,
        safety,
        ground_test: bool = False,
        param_verifier=None,
    ) -> None:
        self._mav = mav
        self._safety = safety
        self._ground_test = ground_test
        self._param_verifier = param_verifier   # optional, see _check_params()

    # ------------------------------------------------------------------

    def run(self) -> List[PreflightItem]:
        """Return every check result in stable order."""
        checks: List[PreflightItem] = []
        checks.append(self._check_mavlink_connected())
        checks.append(self._check_heartbeat())
        checks.append(self._check_mode_guided())
        checks.append(self._check_armed())
        checks.append(self._check_home_set())
        checks.append(self._check_gps())
        checks.append(self._check_sensors())
        checks.append(self._check_rc_link())
        checks.append(self._check_fence())
        checks.append(self._check_battery())
        checks.append(self._check_params())
        return checks

    def all_pass(self) -> bool:
        return all(c.ok for c in self.run())

    # ------------------------------------------------------------------
    #  Individual gates
    # ------------------------------------------------------------------

    def _check_mavlink_connected(self) -> PreflightItem:
        ok = bool(self._mav and self._mav.is_connected())
        return PreflightItem(
            name="MAVLink connected",
            ok=ok,
            message="link up" if ok else "no MAVLink connection",
        )

    def _check_heartbeat(self) -> PreflightItem:
        try:
            ok = self._safety.watchdog_heartbeat(self._mav.get_last_heartbeat_time())
        except Exception:
            ok = False
        return PreflightItem(
            name="Heartbeat fresh",
            ok=ok,
            message="" if ok else f"no heartbeat in last {cfg.HEARTBEAT_WATCHDOG_S}s",
        )

    def _check_mode_guided(self) -> PreflightItem:
        if self._ground_test:
            return PreflightItem("Flight mode = GUIDED", True, "skipped in --ground-test")
        try:
            mode = self._mav.get_mode()
        except Exception:
            mode = "UNKNOWN"
        ok = (mode == "GUIDED")
        return PreflightItem(
            name="Flight mode = GUIDED",
            ok=ok,
            message="" if ok else f"current mode: {mode}",
        )

    def _check_armed(self) -> PreflightItem:
        if self._ground_test:
            return PreflightItem("Vehicle ARMED", True, "skipped in --ground-test")
        try:
            ok = bool(self._mav.is_armed())
        except Exception:
            ok = False
        return PreflightItem(
            name="Vehicle ARMED",
            ok=ok,
            message="" if ok else "FCU reports disarmed",
        )

    def _check_home_set(self) -> PreflightItem:
        try:
            ok = bool(self._mav.is_home_set())
        except Exception:
            ok = False
        return PreflightItem(
            name="HOME position set",
            ok=ok,
            message="" if ok else "FCU has not advertised HOME yet",
            outdoor_only=True,    # HOME is set when GPS lock acquires
        )

    def _check_gps(self) -> PreflightItem:
        try:
            ok    = bool(self._mav.is_gps_ok())
            fix   = int(self._mav.get_gps_fix())
            hdop  = float(self._mav.get_gps_hdop())
            sats  = int(self._mav.get_sat_count())
            fresh = {
                "GLOBAL_POSITION_INT": bool(self._mav.is_global_position_fresh()),
                "GPS_RAW_INT": bool(self._mav.is_gps_raw_fresh()),
                "EKF_STATUS_REPORT": bool(self._mav.is_ekf_status_fresh()),
            }
        except Exception:
            ok, fix, hdop, sats = False, 0, 99.99, 0
            fresh = {
                "GLOBAL_POSITION_INT": False,
                "GPS_RAW_INT": False,
                "EKF_STATUS_REPORT": False,
            }
        if ok:
            msg = f"fix={fix} sats={sats} HDOP={hdop:.2f}"
        else:
            reasons = []
            for name, is_fresh in fresh.items():
                if not is_fresh:
                    reasons.append(f"{name} stale/missing")
            if fix < cfg.GPS_MIN_FIX_TYPE:
                reasons.append(f"fix={fix}<{cfg.GPS_MIN_FIX_TYPE}")
            if hdop > cfg.GPS_MAX_HDOP:
                reasons.append(f"HDOP={hdop:.2f}>{cfg.GPS_MAX_HDOP}")
            if sats < cfg.GPS_MIN_SATS:
                reasons.append(f"sats={sats}<{cfg.GPS_MIN_SATS}")
            msg = "; ".join(reasons) or "GPS not ready"
        return PreflightItem(
            name="GPS fix OK", ok=ok, message=msg,
            outdoor_only=True,   # indoor multipath = no usable fix
        )

    def _check_sensors(self) -> PreflightItem:
        try:
            fresh = bool(self._mav.is_sys_status_fresh())
            ok = bool(self._mav.is_sensors_healthy())
        except Exception:
            fresh = False
            ok = False
        return PreflightItem(
            name="IMU / mag / baro healthy",
            ok=ok,
            message="" if ok else (
                "SYS_STATUS stale/missing" if not fresh
                else "SYS_STATUS reports a degraded sensor"
            ),
        )

    def _check_rc_link(self) -> PreflightItem:
        """RC transmitter must be ON before takeoff is permitted.

        The FCU forwards RC_CHANNELS whenever the receiver is delivering
        frames. Absence of recent RC_CHANNELS == RC link down, which
        means the operator cannot take manual control if the autonomous
        tracker misbehaves. Arming is refused in this state.

        Ground-test mode relaxes this — the operator may bench-test
        without an RC system powered on.
        """
        if self._ground_test:
            return PreflightItem(
                name="RC transmitter connected",
                ok=True,
                message="skipped in --ground-test",
            )
        try:
            ok = bool(self._mav.is_rc_connected())
        except Exception:
            ok = False
        if ok:
            try:
                count = int(self._mav.get_rc_channel_count())
                rssi  = int(self._mav.get_rc_rssi())
            except Exception:
                count, rssi = 0, 0
            return PreflightItem(
                name="RC transmitter connected",
                ok=True,
                message=f"{count} channels"
                        + (f" · RSSI {rssi}" if rssi > 0 else ""),
            )
        try:
            age = float(self._mav.get_rc_age_s())
        except Exception:
            age = float("inf")
        if age != float("inf"):
            base = f"no RC_CHANNELS for {age:.1f}s - turn on transmitter and check bind"
        else:
            # 'Never received' is also produced when the autopilot isn't
            # streaming RC_CHANNELS over MAVLink at all, even with TX on.
            # Point the operator at the relevant ArduPilot params so they
            # don't waste time on the radio side.
            base = (
                "RC_CHANNELS never received - transmitter off/unbound OR "
                "ArduPilot SR*_RC_CHAN=0 on the TELEM port wired to the "
                "companion (try SR1_RC_CHAN=5 or SR2_RC_CHAN=5)"
            )
        return PreflightItem(
            name="RC transmitter connected", ok=False, message=base,
        )

    def _check_fence(self) -> PreflightItem:
        try:
            ok = not bool(self._mav.is_fence_breached())
        except Exception:
            ok = False
        return PreflightItem(
            name="Geofence not breached",
            ok=ok,
            message="" if ok else "FCU reports fence active",
        )

    def _check_params(self) -> PreflightItem:
        """Aggregate the ArduPilot param verifier into a single checklist item.

        Verifier is optional — if none was injected, this gate auto-passes
        in ground-test mode and otherwise hard-fails so the operator knows
        the verifier was not wired.
        """
        if self._param_verifier is None:
            return PreflightItem(
                name="ArduPilot params correct",
                ok=self._ground_test,
                message="verifier not wired" if not self._ground_test
                                              else "skipped in --ground-test",
            )
        try:
            results = self._param_verifier.run()
        except Exception as exc:
            return PreflightItem(
                name="ArduPilot params correct",
                ok=False,
                message=f"verifier error: {exc}",
            )
        bad = [r for r in results if not r.ok]
        if not bad:
            return PreflightItem("ArduPilot params correct", True,
                                 f"{len(results)} params OK")
        # Enumerate every failing param so the operator can fix them in one
        # pass. Previously truncated to "first + (N more)", which forced the
        # operator to guess what the rest were. Joined with '; ' so the
        # operator-input printer can split them onto separate lines.
        detail = "; ".join(f"{r.name}: {r.message}" for r in bad)
        # If every failure is "not advertised by FCU" the batch prefetch has
        # not finished yet — treat as WAIT so the operator knows to retry
        # rather than run to Mission Planner.
        all_loading = all("not advertised" in r.message for r in bad)
        return PreflightItem(
            name="ArduPilot params correct",
            ok=False,
            message=f"{len(bad)} failing — {detail}",
            outdoor_only=all_loading,
        )

    def _check_battery(self) -> PreflightItem:
        try:
            v = float(self._mav.get_battery_voltage())
            critical = bool(self._safety.is_battery_critical(v))
        except Exception:
            v = 0.0
            critical = False
        ok = (v > 0.0) and not critical
        return PreflightItem(
            name="Battery above critical",
            ok=ok,
            message="" if ok else (
                "no voltage reading yet" if v == 0.0 else f"{v:.2f} V — below critical/cell"
            ),
        )
