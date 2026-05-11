# Person Tracking Drone Project Audit Report

> Historical note: this report is superseded by
> `DRONE_PROJECT_TECHNICAL_AUDIT_REPORT_2026-05-08.md`. Several findings in
> this older file were fixed after it was written, including tests, command ACKs,
> target dataclass migration issues, utility config paths, stream-token control
> URLs, and app-level commanded-target geofence checks.

Audit date: 2026-05-07  
Reviewer role: Drone software developer / programmer review, focused on field-readiness, code structure, reuse, safety, and runtime defects.

## Scope And Method

Reviewed the current workspace at `/home/ai-engineer/Akash/Tasks/Akash/AI/Jetson/person_tracking`.

Checks performed:

- Mapped project files, package layout, model assets, scripts, and generated artifacts.
- Read the main execution path: `main.py`, `tracker.py`, `control/`, `gimbal/`, `mavlink_client/`, `safety/`, `tracking/`, `gcs/`, and `utils/`.
- Checked references and duplicated constants with `rg`.
- Ran `python3 -m compileall -q .`; result: passed.
- Checked import availability for `cv2`, `numpy`, `torch`, `ultralytics`, and `pymavlink`; all are installed in this environment.
- Imported `main` and `tracker`; result: passed, but Ultralytics warned that `/home/ai-engineer/.config/Ultralytics` is not writable and fell back to `/tmp`.

Not performed:

- No real SIYI gimbal, RTSP camera, Jetson CUDA performance, MAVLink, ArduPilot, GPS, or flight test was executed.
- No behavioral validation with recorded video was available.
- No unit/integration test suite exists to run.

## Executive Summary

The project has a good high-level modular intent: camera grabbing, detection, tracking, gimbal control, drone control, MAVLink, safety, streaming, and recording are split into separate folders. That is the right direction for a drone stack.

However, the actual runtime center of gravity is still `tracker.py`, a 1,305-line orchestrator that directly owns detection calls, target selection, state transitions, gimbal commands, drone updates, web callbacks, keyboard input, stream dispatch, recording, HUD drawing, logging behavior, and shutdown. In flight software terms, this is a "modular shell around a monolithic mission loop." It can work, but it is harder to test, harder to certify, and risky to modify.

The most important technical risks are:

- Recording writes happen in two places at the same time, which can duplicate frames or corrupt recordings.
- Gimbal angle fallback values are documented as accumulated but are never actually updated, which can break drone yaw recentering and GPS projection when attitude telemetry is stale.
- Drone body following currently commands the drone toward the estimated person position; the documented horizontal standoff is not implemented.
- The web control server binds to `0.0.0.0` and exposes mode, manual gimbal, zoom, lock, and unlock controls without authentication.
- Safety-critical MAVLink mode commands such as LOITER and RTL are sent without COMMAND_ACK confirmation.
- Documentation and utility scripts are stale in several places, including nonexistent files and hardcoded old absolute paths.

## Project Structure Review

Current observed top-level layout:

```text
person_tracking/
|-- main.py
|-- tracker.py
|-- config/
|-- control/
|-- detection/
|-- gcs/
|-- gimbal/
|-- mavlink_client/
|-- models/
|-- safety/
|-- scripts/
|-- tracking/
|-- utils/
|-- readme.md
`-- md.md
```

Approximate Python line counts:

| Area | Approx. LOC | Notes |
|---|---:|---|
| Total Python | 7,157 | Includes production modules and standalone utilities |
| Core runtime modules | 5,779 | Main tracker, control, gimbal, MAVLink, safety, tracking, stream, frame/record/log utilities |
| Standalone diagnostic/test utilities | 1,378 | Mostly scripts under `utils/` |
| `tracker.py` alone | 1,305 | About 18% of all Python LOC and about 23% of core runtime LOC |

### Good Structure Decisions

| Good decision | Evidence | Why it helps |
|---|---|---|
| Domain folders are separated | `control/`, `gimbal/`, `tracking/`, `mavlink_client/`, `safety/`, `gcs/`, `utils/` | Easier for future contributors to find responsibilities |
| Safety logic has a central module | `safety/safety.py` | Good foundation for flight constraints |
| MAVLink access is centralized | `mavlink_client/mavlink_client.py` | Prevents random modules from opening separate MAVLink connections |
| Hardware helpers are isolated | `utils/frame_grabber.py`, `utils/video_recorder.py`, `gimbal/siyi_controller.py` | Reusable across future modes |
| Search algorithms are separated | `control/search_patterns.py` | Cleaner than embedding all scan math in the main loop |
| Persistent person identity is separated | `tracking/person_registry.py` | Correct place for re-identification logic |

### Structural Issues

| Severity | Issue | Evidence | Impact | Recommended fix |
|---|---|---|---|---|
| High | `tracker.py` is still a god-object mission loop | `tracker.py:75`, `tracker.py:953`, 1,305 LOC | Hard to unit test and risky to change because unrelated concerns are coupled | Split into `MissionLoop`, `TargetSelector`, `HudRenderer`, `InputController`, `RecordingManager`, and `StreamAdapter` |
| High | Runtime data uses positional tuples for target data | `target_info` indexes are used throughout `tracker.py` and `control/drone_controller.py` | Easy to swap fields silently, especially bbox and ID values | Replace tuples with a `@dataclass TargetDetection(cx, cy, x1, y1, x2, y2, conf, id)` |
| Medium | Configuration is a mutable global module | `config/__init__.py:2`, `main.py:124-129` | CLI overrides depend on import order; later imports can bind stale defaults | Introduce immutable `Settings` object passed into constructors |
| Medium | No package/dependency manifest | No `requirements.txt`, `pyproject.toml`, `setup.py`, or lockfile found | Reproducibility risk on Jetson deployments | Add `requirements.txt` or `pyproject.toml`; pin critical versions for JetPack |
| Medium | No test structure | No `tests/`, no pytest config | Regression risk is high for flight/safety logic | Add unit tests for safety, PID, state transitions, geofence, EKF, packet CRC, and stream parsing |
| Medium | No lint/type tooling | No Ruff/Mypy/Pyright config found | Type and interface mistakes will remain runtime-only | Add Ruff plus Pyright/Mypy for core modules |
| Medium | Generated artifacts are in source tree | Many `__pycache__/` and `.pyc` files are present | Noise, deployment confusion, and possible stale bytecode | Add `.gitignore`; remove generated caches from source control |
| Medium | Empty `.git` directory exists but repository is invalid | `git status --short` fails; `.git/` is empty | Tooling cannot track changes or blame history | Restore a real Git repo or remove empty `.git` stub |
| Low | Large model binaries live inside the repo | `models/` is 88 MB | Makes source transfer/versioning heavy | Use Git LFS, release artifacts, or deployment asset fetch step |
| Low | Existing `md.md` is stale | `md.md` reports missing GUIDED/armed/fence/finite checks that now exist | Future reviewers may chase fixed issues | Rename/archive it as historical or replace with current report |

## Code Reuse Assessment

Overall reuse level: moderate in the core runtime, weak in diagnostics and configuration handling.

Estimated reuse score: 6/10 for runtime modules, 3/10 for utility scripts, 5/10 overall.

### Where Reuse Is Working

| Reusable component | Current reuse quality | Notes |
|---|---|---|
| `SafetyMonitor` | Good | Central altitude, speed, geofence, watchdog, and battery checks |
| `MAVLinkClient` | Good | Encapsulates pymavlink connection, telemetry cache, and send commands |
| `FrameGrabber` | Good | Dedicated low-latency frame source used by tracker, streamer, and recorder thread |
| `VideoRecorder` | Good concept | Encoder fallback logic is reusable, but concurrent writes are currently unsafe |
| `SIYIController` | Good concept | Packet creation, telemetry polling, speed, center, zoom controls are centralized |
| Search classes | Good | Shared `start/stop/get_command` pattern |
| Tracking classes | Good | Registry, velocity tracker, and geolocation are reusable |
| `config/config.py` | Partial | Central constants exist, but mutable globals and duplicated utility constants weaken it |

### Where Reuse Is Weak

| Weak area | Evidence | Problem |
|---|---|---|
| Detection abstraction leaks | `detection/detector.py:24-27`; `tracker.py:1066-1084` | `Detector` only exposes `.model`; `tracker.py` directly calls Ultralytics APIs and handles fallback |
| Utility scripts duplicate constants | `utils/yolo_gpu_optimized.py:64-66`, `utils/yolo_test_simple.py:12-14`, `utils/diagnostic.py:41`, `utils/diagnostic.py:63` | Camera IP, RTSP URL, and model paths diverge from `config/config.py` |
| Absolute old paths are hardcoded | `/home/ai2/Desktop/person_tracking/bestjetson.pt` appears in several utilities | Scripts will fail on this workspace and on most Jetson users |
| No shared diagnostic library | `utils/yolo_gpu_optimized.py`, `utils/yolo_debug_optimized.py`, `utils/yolo_test_simple.py`, `utils/diagnostic.py` | Similar model/camera/CUDA checks are copied rather than reused |
| No shared DTOs/interfaces | `target_info` tuples and raw dicts passed between systems | Control, overlay, stream, and geolocation depend on implicit tuple positions |
| Config is not consistently consumed | Core mostly uses `cfg`, but utilities often redefine values | Deployment changes must be made in multiple places |

## Technical Bugs And Risks

### High Severity

#### H1. Recording writes are duplicated and concurrent

Evidence:

- Dedicated recorder thread writes frames at `tracker.py:731-756`.
- Main loop also writes frames at `tracker.py:1162-1167`.
- Both call `self.recorder.write(...)` on the same `VideoRecorder` instance.

Impact:

- OpenCV `VideoWriter` is not guaranteed thread-safe.
- Recorded video can contain duplicate frames, timing distortion, or corruption.
- CPU/GPU encode load is higher than expected.

Recommended fix:

- Keep only the dedicated recorder thread.
- Remove the main-loop recording block at `tracker.py:1162-1167`, or disable the recorder thread and keep main-loop recording, but do not do both.
- If multiple writers are ever required, add a single producer/consumer queue and one writer thread.

#### H2. Gimbal angle fallback is broken

Evidence:

- `_cmd_yaw_deg` and `_cmd_pitch_deg` are described as accumulated fallback values at `gimbal/siyi_controller.py:65-67`.
- `gimbal_pan_deg` and `gimbal_tilt_deg` return these fallback values when attitude telemetry is stale at `gimbal/siyi_controller.py:175-194`.
- `set_speed()` updates only `_last_yaw` and `_last_pitch` at `gimbal/siyi_controller.py:302-305`; it never integrates speed into `_cmd_yaw_deg` or `_cmd_pitch_deg`.
- `tracker.py` uses these angles for drone EKF projection and yaw recentering at `tracker.py:1137-1144`.

Impact:

- If SIYI attitude telemetry is delayed or lost, drone controller sees pan/tilt as 0 deg even while the gimbal is moving.
- Drone body yaw recentering may not activate.
- Person geolocation can be badly wrong because projection uses incorrect gimbal angles.

Recommended fix:

- Prefer actual telemetry only; if stale, mark attitude invalid and suppress drone geolocation/following.
- Or implement a bounded command integrator using elapsed time, speed-to-deg/sec calibration, and SIYI mechanical limits.
- Expose `get_attitude_freshness()` so drone control can gate on valid gimbal telemetry.

#### H3. Drone following does not implement the documented horizontal standoff

Evidence:

- README documents `FOLLOW_STANDOFF_M` at `readme.md:219`.
- `FOLLOW_STANDOFF_M` is not defined in `config/config.py`.
- `DroneController.update()` explicitly commands the estimated person NED position as the target at `control/drone_controller.py:278-283`.

Impact:

- The drone can be commanded to converge toward the person, not hold a safe offset.
- This is a safety and behavior mismatch between documentation and flight control.

Recommended fix:

- Add a real `FOLLOW_STANDOFF_M` configuration.
- Compute desired drone position as an offset from person position based on bearing, operator preference, and geofence.
- Enforce a minimum person/drone separation and a no-overfly rule.

#### H4. Web control server exposes flight-relevant actions without authentication

Evidence:

- HTTP server binds to all interfaces at `gcs/stream_server.py:500`.
- Endpoints include `/click`, `/unlock`, `/mode`, `/gimbal`, `/zoom_in`, and `/zoom_out` at `gcs/stream_server.py:401-435`.
- README encourages browser access over LAN/Tailscale.

Impact:

- Anyone on the reachable network can lock targets, switch manual/auto mode, move the gimbal, or interfere with operator control.
- In drone mode, this can become a safety issue.

Recommended fix:

- Bind to `127.0.0.1` by default and require explicit `STREAM_HOST=0.0.0.0` for remote access.
- Add a random session token or HTTP Basic auth at minimum.
- Separate read-only stream access from control endpoints.

#### H5. LOITER and RTL commands are not ACK-confirmed

Evidence:

- `send_loiter()` sends `MAV_CMD_NAV_LOITER_UNLIM` and returns after printing at `mavlink_client/mavlink_client.py:516-533`.
- `send_rtl()` sends `MAV_CMD_NAV_RETURN_TO_LAUNCH` and returns after printing at `mavlink_client/mavlink_client.py:535-552`.
- There is no COMMAND_ACK wait path.

Impact:

- A safety-critical mode command can be rejected or lost without the code knowing.
- DroneController retries some actions based on mode state, but an explicit ACK path is still missing.

Recommended fix:

- Add a thread-safe ACK queue in `MAVLinkClient._rx_loop`.
- Implement `send_command_with_ack(command, retries, timeout)`.
- Use it for LOITER, RTL, GUIDED mode, and other safety-relevant mode changes.

#### H6. YOLO device is hardcoded to GPU 0

Evidence:

- `tracker.py:1072` and `tracker.py:1083` call Ultralytics with `device=0`.
- The fallback path after `model.track()` failure still uses `device=0`.

Impact:

- On systems without CUDA, or with a different GPU index, both primary and fallback inference can fail.
- Makes local development and CPU-only test runs harder.

Recommended fix:

- Add `DETECT_DEVICE` to config/CLI, with values like `auto`, `cpu`, `cuda:0`.
- If CUDA is unavailable, fallback to CPU or fail early with a clear message.

### Medium Severity

#### M1. Stream query parsing is fragile

Evidence:

- Query strings are parsed with `dict(p.split('=') for p in qs.split('&') if '=' in p)` at `gcs/stream_server.py:425`, `431`, `442`, and `454`.

Impact:

- Values containing `=` can raise `ValueError`.
- URL encoding is not decoded.
- A malformed request can crash a handler thread.

Recommended fix:

- Use `urllib.parse.parse_qs` or `urllib.parse.urlparse`.

#### M2. Invalid web gimbal direction can keep old manual motion alive

Evidence:

- `handle_web_gimbal()` handles known directions at `tracker.py:656-667`.
- Unknown directions still set `_manual_key_t = time.time() + 86400.0` at `tracker.py:668` and return success.

Impact:

- A bad `/gimbal?dir=...` request in MANUAL mode can extend the previous movement instead of rejecting the command.

Recommended fix:

- Add an `else` branch that stops or returns an error without extending the hold timer.

#### M3. Auto-zoom state is not updated for relative zoom commands

Evidence:

- Auto zoom checks `self.ctrl.current_zoom` at `gimbal/auto_zoom.py:49-51`.
- `zoom_in()` and `zoom_out()` do not update `current_zoom` at `gimbal/siyi_controller.py:325-329`.
- Only `zoom_absolute()` updates `current_zoom` at `gimbal/siyi_controller.py:344`.

Impact:

- Auto zoom can repeatedly send relative zoom commands because the internal zoom level remains stale.
- Manual web/keyboard zoom can desynchronize controller state.

Recommended fix:

- Parse zoom telemetry if SIYI provides it.
- Otherwise update an estimated zoom state when relative zoom is sent and stopped, with clamping.

#### M4. Documentation references files and config that do not exist

Evidence:

- README references `gimbal/siyi_gimbal.py` at `readme.md:91`; file not found.
- README references `utils/colors.py` at `readme.md:111`; file not found.
- README documents `FOLLOW_STANDOFF_M` at `readme.md:219`; config value not found.

Impact:

- New developers lose trust in documentation.
- Operators may believe safety behavior exists when it does not.

Recommended fix:

- Update README from the actual file tree.
- Add a documentation check to CI or keep architecture docs generated from source.

#### M5. Diagnostic utilities are stale and inconsistent with the modular app

Evidence:

- Several utilities hardcode `CAMERA_IP = "192.168.144.25"` and old `/home/ai2/Desktop/person_tracking/...` model paths.
- `utils/diagnostic.py` title says "YOLO11 + ZR30", while the main project is SIYI A8 mini.
- `utils/yolo_test_simple.py` says "Raspberry Pi 5" while the README targets Jetson.

Impact:

- Utilities will fail in this workspace.
- Troubleshooting can point operators toward the wrong hardware assumptions.

Recommended fix:

- Move all diagnostics to `tools/` or `diagnostics/`.
- Import shared `config`.
- Add CLI flags for camera IP, RTSP URL, model path, and device.

#### M6. Exceptions are swallowed in control-critical paths

Evidence:

- Detection fallback catches all exceptions at `tracker.py:1076` without logging the primary failure.
- EKF update catches all exceptions and silently passes at `control/drone_controller.py:337-338`.
- MAVLink receive loop catches all exceptions and passes at `mavlink_client/mavlink_client.py:284-285`.
- Frame grabber fallback catches exceptions without logging at `utils/frame_grabber.py:88-102`.

Impact:

- Real integration failures can appear as normal degraded behavior.
- Field debugging becomes slow because root causes disappear.

Recommended fix:

- Log exception type and message with rate limiting.
- Use narrow exception types where possible.
- Add health status fields for detector, geolocation, MAVLink RX, and camera.

#### M7. Relative model paths depend on launch directory

Evidence:

- `MODEL_PATH = "models/yolo26s.engine"` in `config/config.py`.
- CLI default uses that relative path.

Impact:

- Running `python3 /path/to/main.py` from a different directory can fail to find the model.

Recommended fix:

- Resolve paths relative to project root in `main.py`.
- Or store absolute paths in a loaded settings object.

#### M8. Stream server restart lifecycle is incomplete

Evidence:

- `StreamServer.stop()` calls `shutdown()` but does not call `server_close()` or join the server thread at `gcs/stream_server.py:505-512`.
- `tracker.py` can stop/start stream with the `l` key at `tracker.py:1281-1287`.

Impact:

- Restart may hit "address already in use" or leave a background server thread alive.

Recommended fix:

- Add idempotent `start()`/`stop()` guards.
- Call `server_close()` and join the thread after shutdown.

#### M9. Import side effect from Ultralytics config location

Evidence:

- Importing `main` and `tracker` succeeded but produced a warning that `/home/ai-engineer/.config/Ultralytics` is not writable and settings were created under `/tmp`.

Impact:

- Under systemd or read-only users, Ultralytics may create settings in unpredictable locations.

Recommended fix:

- Set `YOLO_CONFIG_DIR` to a writable application directory or `/tmp/ultralytics` in launch scripts/services.

### Lower Severity / Maintainability

| Issue | Evidence | Recommendation |
|---|---|---|
| README commands use `python`, but this machine only has `python3` | `python` command failed, `python3` works | Update docs to use `python3`, or ensure deployment image provides `python` alias |
| `Detector` wrapper is too thin | `detection/detector.py:24-27` | Move inference, device fallback, and tracker choice into `Detector.detect_or_track()` |
| Raw dictionaries are shared with web/UI callbacks | `_detected_ids` is copied from multiple threads | Introduce a small thread-safe `TargetStore` |
| HTTP API has no schema/versioning | Stream server returns ad hoc dicts | Define endpoint response shapes and error codes |
| `readme.md` and `md.md` naming are inconsistent | Lowercase README and generic `md.md` | Use `README.md`, `docs/`, and dated reports |

## Safety Review Notes

The current code has improved safety gates compared with the older `md.md` note:

- GUIDED mode gate exists at `control/drone_controller.py:168-173`.
- Arm-state gate exists at `control/drone_controller.py:175-177`.
- Home-position gate exists at `control/drone_controller.py:179-182`.
- Sensor-health gate exists at `control/drone_controller.py:184-190`.
- Geofence return velocity exists at `control/drone_controller.py:227-246`.
- Non-finite MAVLink command guard exists at `mavlink_client/mavlink_client.py:493-496`.
- ArduPilot fence breach handling exists in the MAVLink RX loop.

Remaining safety gaps:

- Confirm LOITER/RTL/GUIDED with COMMAND_ACK.
- Implement actual person standoff and no-overfly behavior.
- Gate drone body following on fresh gimbal attitude, valid GPS, valid EKF, and operator arming intent.
- Add authentication or local-only binding for web control endpoints.
- Add simulator/HIL tests before flight.

## Recommended Refactor Roadmap

### Phase 1: Fix Immediate Runtime Risks

1. Remove duplicate recorder writes.
2. Fix or disable stale gimbal angle fallback for drone geolocation.
3. Add real standoff logic before using drone body follow near people.
4. Add authentication or local-only binding for web controls.
5. Add COMMAND_ACK confirmation for LOITER, RTL, and GUIDED mode commands.
6. Make YOLO device configurable and fail early with a clear message.

### Phase 2: Make The Code Testable

1. Introduce dataclasses for detections, gimbal attitude, drone telemetry, and command outputs.
2. Split `tracker.py` into smaller units:
   - `MissionLoop`
   - `TargetSelector`
   - `StateMachineRunner`
   - `HudRenderer`
   - `InputController`
   - `RecordingManager`
   - `StreamAdapter`
3. Move inference behavior into `Detector`, not `tracker.py`.
4. Replace mutable global config with a `Settings` object.
5. Add unit tests for pure logic modules.

### Phase 3: Deployment Hygiene

1. Add `requirements.txt` or `pyproject.toml`.
2. Add `.gitignore` for `__pycache__/`, logs, recordings, temporary settings, and generated model exports.
3. Move large model files to Git LFS or deployment artifacts.
4. Update README to match actual files and current commands.
5. Convert standalone diagnostics into reusable CLI tools that import shared config.

## Suggested Test Plan

Minimum tests before field flight:

| Test type | What to validate |
|---|---|
| Unit | PID output limits, dead zone, target smoothing, velocity predictor, edge-exit behavior |
| Unit | Safety altitude clamp, speed clamp, geofence distance, battery critical threshold |
| Unit | SIYI CRC test vector and packet construction |
| Unit | Query parsing and web gimbal invalid-command handling |
| Unit | DroneController gates: not GUIDED, not armed, no GPS, no home, battery critical |
| Simulation | MAVLink SITL mode transitions, LOITER/RTL ACK success and failure |
| Simulation | Person lost ladder: EKF coast, zero velocity, LOITER, alert |
| Bench | RTSP reconnect, stream restart, recorder start/stop, no duplicate frames |
| Hardware bench | Gimbal attitude telemetry freshness and fallback behavior |
| Flight readiness | Geofence return, standoff behavior, RC/operator override, web control disabled/authenticated |

## Final Assessment

The project is promising and already has several strong building blocks for a Jetson-based person-tracking drone. The core concepts are sound: separate safety monitor, MAVLink client, gimbal controller, EKF geolocation, search patterns, and stream/record helpers.

The main weakness is integration maturity. Too much behavior still converges inside `tracker.py`, and several interfaces are implicit rather than typed. For gimbal-only demos, the system is close to usable after recorder and config cleanup. For drone body following around people, the standoff gap, stale gimbal attitude fallback, unauthenticated web controls, and missing ACK confirmation should be treated as blockers before real flight.
