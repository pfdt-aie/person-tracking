"""
main.py — Entry point for the modular person-tracking drone system.

Usage:
    # Gimbal-only mode (default — safe, backward-compatible):
    python main.py

    # With drone body following (requires ArduPilot on /dev/ttyACM0):
    python main.py --drone

    # With camera calibration file:
    python main.py --drone --calibration /path/to/calib.yaml

    # Override MAVLink serial device:
    python main.py --drone --device /dev/ttyTHS1

Controls (keyboard — display mode only):
    q  = Quit         t  = Toggle tracking    r  = Center gimbal + zoom reset
    s  = Toggle search  i  = Restart init scan  d  = Toggle display
    z  = Zoom in (0.5s) x  = Zoom out           a  = Toggle auto-zoom
    v  = Toggle recording  l  = Toggle live stream
    +/-= Kp gain        [/]= Max speed

Terminal commands (headless / SSH mode):
    track <id>  — lock onto persistent person ID
    unlock      — release lock, revert to largest-person mode
    ids         — print currently detected person IDs
    q           — quit
"""

import argparse
import os
import pathlib
import sys
from datetime import datetime

# Preflight: verify declared dependencies are installed in this interpreter
# BEFORE importing any third-party package. Fails fast with a clear fix
# command instead of a deep stack trace mid-startup.
from utils.preflight import enforce_dependencies
enforce_dependencies(pathlib.Path(__file__).resolve().parent / "requirements.txt")

# Pin Ultralytics settings dir before any ultralytics import so it lands
# in a predictable location regardless of systemd user or read-only home.
_ul_dir = pathlib.Path("/tmp/ultralytics")
_ul_dir.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("YOLO_CONFIG_DIR", str(_ul_dir))

import torch

import config as cfg


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Person-tracking drone system — SIYI A8 mini + ArduPilot"
    )
    p.add_argument(
        "--drone",
        action="store_true",
        default=False,
        help="Enable drone body following via MAVLink (default: gimbal-only).",
    )
    p.add_argument(
        "--device",
        type=str,
        default=cfg.MAVLINK_DEVICE,
        help=f"MAVLink serial device (default: {cfg.MAVLINK_DEVICE}).",
    )
    p.add_argument(
        "--baud",
        type=int,
        default=cfg.MAVLINK_BAUD,
        help=f"Serial baud rate (default: {cfg.MAVLINK_BAUD}).",
    )
    p.add_argument(
        "--calibration",
        type=str,
        default=cfg.CALIBRATION_YAML,
        help="Path to OpenCV camera calibration YAML (default: auto-estimate from specs).",
    )
    p.add_argument(
        "--model",
        type=str,
        default=cfg.MODEL_PATH,
        help=f"Path to YOLO TensorRT engine (default: {cfg.MODEL_PATH}).",
    )
    p.add_argument(
        "--detect-device",
        type=str,
        default=cfg.DETECT_DEVICE,
        help='"auto" | "cpu" | "cuda:0" | "0"  (default: auto)',
    )
    p.add_argument(
        "--stream-host",
        type=str,
        default=cfg.STREAM_HOST,
        help=f'Stream server bind address (default: {cfg.STREAM_HOST} — '
             f'open to LAN/Tailscale; use 127.0.0.1 to bind loopback only).',
    )
    p.add_argument(
        "--stream-token",
        type=str,
        default=cfg.STREAM_TOKEN,
        help="Auth token required on control endpoints. STRONGLY recommended "
             "when --stream-host is 0.0.0.0 (the default).",
    )
    p.add_argument(
        "--ground-test",
        action="store_true",
        default=False,
        help="Dry-run mode: full pipeline runs, telemetry is read, but NO "
             "MAVLink commands (velocity/position/mode) are transmitted. "
             "Use for bench tests with a real person in front of the camera.",
    )
    p.add_argument(
        "--cells",
        type=int,
        default=0,
        choices=[0, 3, 4, 5, 6],
        help="Battery cell count override (3–6). 0 (default) = auto-detect "
             "from pack voltage. Use this when the auto-detection is "
             "ambiguous on a partially-charged pack.",
    )
    return p.parse_args()


def print_banner(args: argparse.Namespace) -> None:
    gpu    = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "N/A"
    cuda   = "YES" if torch.cuda.is_available() else "NO"
    now    = datetime.now().strftime('%Y-%m-%d  %H:%M:%S')
    print()
    print("=" * 62)
    print("  SIYI A8 mini — Person Tracking Drone  v10.0-modular")
    print("=" * 62)
    print(f"  Session  {now}")
    print(f"  CUDA     {cuda}  {gpu}")
    print(f"  Model    {args.model}")
    print("-" * 62)
    print(f"  Kp       {cfg.ADAPT_KP_MIN}–{cfg.ADAPT_KP_MAX}   "
          f"Speed  {cfg.ADAPT_SPEED_MIN}–{cfg.ADAPT_SPEED_MAX}")
    print(f"  Predict  {cfg.PREDICT_DURATION}s + Fade {cfg.PRED_FADE_DURATION}s   "
          f"EdgeBoost  {cfg.EDGE_EXIT_BOOST}×")
    print(f"  Search   INIT-SCAN "
          f"→ SECTOR({cfg.SECTOR_SEARCH_TIMEOUT:.0f}s) "
          f"→ EXP-SQ({cfg.EXPAND_SEARCH_TIMEOUT:.0f}s) "
          f"→ LISSAJOUS")
    print(f"  Zoom     {'ON' if cfg.AUTO_ZOOM_ENABLED else 'OFF'}  (6× digital)")
    print(f"  Record   {'AUTO' if cfg.AUTO_RECORD else 'OFF'}  "
          f"Stream  {'ON' if cfg.STREAM_ENABLED else 'OFF'} port {cfg.STREAM_PORT}")
    if args.drone:
        print(f"  Drone    ON  MAVLink {args.device} @ {args.baud} baud")
        print(f"  Calib    {args.calibration or 'auto-estimate from HFOV=81°'}")
        print(f"  Follow   {cfg.FOLLOW_ALTITUDE_M}m AGL  "
              f"Max speed {cfg.MAX_TRACKING_SPEED_MS}m/s")
        print(f"  AltFloor {cfg.MIN_ALT_M}m  (SAFETY-CRITICAL)")
        print(f"  Failsafe hover@{cfg.TRACKING_LOSS_HOVER_S}s → "
              f"loiter@{cfg.TRACKING_LOSS_LOITER_S}s → "
              f"alert@{cfg.TRACKING_LOSS_ALERT_S}s")
    else:
        print(f"  Drone    OFF (gimbal-only)")
    print("=" * 62)
    print()


def main() -> int:
    # Install session log before any print() so banner + all output go to file.
    from utils.logger import setup_log, teardown_log
    from utils.flight_log import init_flight_log, close_flight_log
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir     = os.path.join(_script_dir, "logs")
    log_path, orig_stdout, orig_stderr, log_file = setup_log(log_dir)
    # S3.2 — structured JSONL event log alongside the stdout capture.
    init_flight_log(log_dir)

    args = parse_args()

    # Resolve model path relative to project root so the process can be
    # launched from any working directory (e.g. systemd service).
    _project_root = pathlib.Path(__file__).resolve().parent
    model_path = pathlib.Path(args.model)
    if not model_path.is_absolute():
        model_path = _project_root / model_path
    if not model_path.exists():
        print(f"[Main] ERROR: model not found: {model_path}", file=sys.stderr)
        return 1

    from config.settings import load_settings
    settings = load_settings(
        mavlink_device   = args.device,
        mavlink_baud     = args.baud,
        calibration_yaml = args.calibration,
        detect_device    = args.detect_device,
        stream_host      = args.stream_host,
        stream_token     = args.stream_token,
        model_path       = str(model_path),
        ground_test      = args.ground_test,
        cells_override   = args.cells,
    )

    if args.ground_test:
        print()
        print("=" * 62)
        print("  ⚠ GROUND-TEST MODE — MAVLINK TX SUPPRESSED")
        print("  Drone will receive ZERO commands. Use for bench tests only.")
        print("=" * 62)
        print()

    print_banner(args)
    print(f"[Log] Session log → {log_path}")

    try:
        from tracker import PersonGimbalTracker
        tracker = PersonGimbalTracker(drone_enabled=args.drone, settings=settings)
        tracker.run()
    finally:
        print(f"\n[Log] Session end: {datetime.now().strftime('%Y-%m-%d  %H:%M:%S')}")
        close_flight_log()
        teardown_log(orig_stdout, orig_stderr, log_file, log_path)

    return 0


if __name__ == "__main__":
    sys.exit(main())



