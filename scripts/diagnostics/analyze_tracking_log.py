#!/usr/bin/env python3
"""Summarize person-tracking field logs.

Usage:
    python scripts/diagnostics/analyze_tracking_log.py gimbal_track_*.log
    python scripts/diagnostics/analyze_tracking_log.py --baseline old.log new.log
    python scripts/diagnostics/analyze_tracking_log.py --baseline --fail-on-regression old.log new.log
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


PATTERNS = {
    "ids_seen": re.compile(r"\[IDs\] ID "),
    "ids_none": re.compile(r"\[IDs\] \(none detected\)"),
    "lost_predicting": re.compile(r"\[State\] Lost .*PREDICTING"),
    "lost_searching": re.compile(r"\[State\] Lost .*SEARCHING"),
    "found_search": re.compile(r"\[State\] Found during"),
    "reacquired": re.compile(r"\[State\] Re-acquired"),
    "sector_start": re.compile(r"\[Search\] Sector scan started"),
    "expand_start": re.compile(r"\[ExpandSq\]"),
    "lissajous_start": re.compile(r"\[Lissajous\] Long-duration"),
    "edge_exit": re.compile(r"edge:"),
    "follow_enabled": re.compile(r"Drone-body following ENABLED"),
    "follow_stopped": re.compile(r"Drone-body following STOPPED"),
    "battery_critical": re.compile(r"Battery critical"),
}


def analyze(path: Path) -> dict[str, int]:
    counts = {name: 0 for name in PATTERNS}
    for line in path.read_text(errors="replace").splitlines():
        for name, pattern in PATTERNS.items():
            if pattern.search(line):
                counts[name] += 1
    return counts


def metrics(counts: dict[str, int]) -> dict[str, float]:
    loss_total = counts["lost_predicting"] + counts["lost_searching"]
    reacq_total = counts["found_search"] + counts["reacquired"]
    id_total = counts["ids_seen"] + counts["ids_none"]
    none_ratio = counts["ids_none"] / id_total if id_total else 0.0
    return {
        "id_total": float(id_total),
        "none_ratio": none_ratio,
        "loss_total": float(loss_total),
        "reacq_total": float(reacq_total),
        "search_total": float(
            counts["sector_start"] + counts["expand_start"] + counts["lissajous_start"]
        ),
    }


def print_summary(path: Path, counts: dict[str, int]) -> None:
    m = metrics(counts)

    print(f"\n{path}")
    print("-" * len(str(path)))
    print(f"ID reports             : seen={counts['ids_seen']} none={counts['ids_none']} none_ratio={m['none_ratio']:.1%}")
    print(f"Loss transitions       : total={int(m['loss_total'])} predicting={counts['lost_predicting']} searching={counts['lost_searching']}")
    print(f"Reacquisitions         : total={int(m['reacq_total'])} prediction={counts['reacquired']} search={counts['found_search']}")
    print(f"Search starts          : sector={counts['sector_start']} expanding={counts['expand_start']} lissajous={counts['lissajous_start']}")
    print(f"Edge exits             : {counts['edge_exit']}")
    print(f"Follow transitions     : enabled={counts['follow_enabled']} stopped={counts['follow_stopped']}")
    print(f"Battery critical warns : {counts['battery_critical']}")

    if counts["lost_searching"] > max(3, counts["ids_seen"] // 6):
        print("Hint: frequent SEARCHING transitions remain; consider raising LOCK_TARGET_GRACE_S slightly.")
    if m["none_ratio"] > 0.35 and counts["ids_seen"] > 0:
        print("Hint: detector flicker is high; inspect confidence threshold, lighting, and camera latency.")
    if counts["sector_start"] and counts["found_search"] == counts["sector_start"]:
        print("Hint: sector search is recovering targets; current search pitch/sign is likely usable.")
    if counts["battery_critical"]:
        print("Hint: battery failsafe affected this run; tracking quality may be secondary to power state.")


def _fmt_delta(new: float, old: float, suffix: str = "") -> str:
    delta = new - old
    sign = "+" if delta >= 0 else ""
    return f"{new:.1f}{suffix} ({sign}{delta:.1f}{suffix})"


def has_regression(
    old: dict[str, float],
    new: dict[str, float],
    none_ratio_tolerance: float,
    count_tolerance: float,
) -> bool:
    return (
        new["none_ratio"] > old["none_ratio"] + none_ratio_tolerance
        or new["loss_total"] > old["loss_total"] + count_tolerance
        or new["search_total"] > old["search_total"] + count_tolerance
    )


def print_comparison(
    baseline: Path,
    current: Path,
    none_ratio_tolerance: float = 0.0,
    count_tolerance: float = 0.0,
) -> bool:
    old_counts = analyze(baseline)
    new_counts = analyze(current)
    old = metrics(old_counts)
    new = metrics(new_counts)
    regressed = has_regression(old, new, none_ratio_tolerance, count_tolerance)

    print(f"\nComparison: {baseline} -> {current}")
    print("-" * (14 + len(str(baseline)) + len(str(current))))
    print(f"None ratio       : {_fmt_delta(new['none_ratio'] * 100.0, old['none_ratio'] * 100.0, '%')}")
    print(f"Loss transitions : {_fmt_delta(new['loss_total'], old['loss_total'])}")
    print(f"Search starts    : {_fmt_delta(new['search_total'], old['search_total'])}")
    print(f"Reacquisitions   : {_fmt_delta(new['reacq_total'], old['reacq_total'])}")

    print(
        "Tolerance        : "
        f"none_ratio=+{none_ratio_tolerance * 100.0:.1f}% "
        f"counts=+{count_tolerance:.1f}"
    )
    print(f"Overall          : {'needs review' if regressed else 'improved/non-regressed'}")
    return regressed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--baseline",
        action="store_true",
        help="Compare exactly two logs: baseline first, current second",
    )
    parser.add_argument(
        "--fail-on-regression",
        action="store_true",
        help="With --baseline, exit 1 when none ratio, losses, or searches regress",
    )
    parser.add_argument(
        "--none-ratio-tolerance",
        type=float,
        default=0.02,
        help="Allowed none-ratio regression before failing, as a fraction",
    )
    parser.add_argument(
        "--count-tolerance",
        type=float,
        default=2.0,
        help="Allowed count regression before failing",
    )
    parser.add_argument("logs", nargs="+", type=Path, help="Tracking log file(s)")
    args = parser.parse_args()

    if args.baseline and len(args.logs) != 2:
        print("--baseline requires exactly two log files: old.log new.log")
        return 2

    missing = [path for path in args.logs if not path.exists()]
    if missing:
        for path in missing:
            print(f"missing: {path}")
        return 2

    if args.baseline:
        regressed = print_comparison(
            args.logs[0],
            args.logs[1],
            none_ratio_tolerance=args.none_ratio_tolerance,
            count_tolerance=args.count_tolerance,
        )
        return 1 if args.fail_on_regression and regressed else 0

    for path in args.logs:
        print_summary(path, analyze(path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
