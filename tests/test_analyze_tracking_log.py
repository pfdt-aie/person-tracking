"""Tests for the tracking-log analyzer."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from scripts.diagnostics.analyze_tracking_log import analyze, metrics, main


def test_analyze_counts_tracking_events(tmp_path):
    log = tmp_path / "track.log"
    log.write_text(
        "\n".join(
            [
                "[IDs] ID 1@(10,20)",
                "[IDs] (none detected)",
                "[State] Lost \u2192 SEARCHING (no velocity, dir=R)",
                "[Search] Sector scan started: phase 1, dir=R",
                "[State] Found during SEARCHING",
                "[State] Lost \u2192 PREDICTING (vel=0.10/s, edge:right)",
                "[State] Re-acquired from PREDICTING (edge:right)",
                "[Safety] Battery critical: 13.91V",
            ]
        )
    )

    counts = analyze(log)
    m = metrics(counts)

    assert counts["ids_seen"] == 1
    assert counts["ids_none"] == 1
    assert counts["lost_searching"] == 1
    assert counts["lost_predicting"] == 1
    assert counts["sector_start"] == 1
    assert counts["edge_exit"] == 2
    assert counts["battery_critical"] == 1
    assert m["none_ratio"] == 0.5
    assert m["loss_total"] == 2.0


def test_baseline_comparison_cli_reports_improvement(tmp_path, capsys, monkeypatch):
    old = tmp_path / "old.log"
    new = tmp_path / "new.log"
    old.write_text(
        "\n".join(
            [
                "[IDs] ID 1@(10,20)",
                "[IDs] (none detected)",
                "[State] Lost \u2192 SEARCHING (no velocity, dir=R)",
                "[Search] Sector scan started: phase 1, dir=R",
            ]
        )
    )
    new.write_text("[IDs] ID 1@(10,20)\n")
    monkeypatch.setattr(
        sys,
        "argv",
        ["analyze_tracking_log.py", "--baseline", str(old), str(new)],
    )

    assert main() == 0

    out = capsys.readouterr().out
    assert "None ratio" in out
    assert "improved/non-regressed" in out


def test_baseline_gate_fails_on_regression(tmp_path, monkeypatch):
    old = tmp_path / "old.log"
    new = tmp_path / "new.log"
    old.write_text("[IDs] ID 1@(10,20)\n")
    new.write_text(
        "\n".join(
            [
                "[IDs] ID 1@(10,20)",
                "[IDs] (none detected)",
                "[State] Lost \u2192 SEARCHING (no velocity, dir=R)",
                "[Search] Sector scan started: phase 1, dir=R",
            ]
        )
    )
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "analyze_tracking_log.py",
            "--baseline",
            "--fail-on-regression",
            "--none-ratio-tolerance",
            "0",
            "--count-tolerance",
            "0",
            str(old),
            str(new),
        ],
    )

    assert main() == 1


def test_baseline_gate_allows_small_regression_inside_tolerance(tmp_path, monkeypatch):
    old = tmp_path / "old.log"
    new = tmp_path / "new.log"
    old.write_text("[IDs] ID 1@(10,20)\n[IDs] ID 1@(11,20)\n")
    new.write_text("[IDs] ID 1@(10,20)\n[IDs] (none detected)\n")
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "analyze_tracking_log.py",
            "--baseline",
            "--fail-on-regression",
            "--none-ratio-tolerance",
            "0.6",
            "--count-tolerance",
            "0",
            str(old),
            str(new),
        ],
    )

    assert main() == 0
