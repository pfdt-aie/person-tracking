# Drone Person Tracking Project - Technical Audit Report

Audit date: 2026-05-08  
Reviewer role: Senior drone software developer / programmer review, focused on project structure, code reuse, technical defects, testability, and field-readiness.

Post-fix status: the high-severity runtime issues identified in this report were fixed on 2026-05-08. The code now uses `TargetDetection` fields in web/headless ID paths, `pytest -q` runs cleanly, commanded follow targets are geofence-checked before MAVLink position sends, and stream control URLs propagate `STREAM_TOKEN`. Follow-up structural cleanup was also started: a valid Git repository was initialized, generated cache folders were removed/ignored, dependency metadata was added, utility scripts now reuse central config paths, and several constructors now read config at instantiation instead of binding defaults at import time.

## 1. Executive Summary

The project has a solid direction for a Jetson + SIYI A8 mini + Orange Cube+/ArduPilot person-following system. The major runtime responsibilities are separated into meaningful folders: detection, gimbal control, drone body control, MAVLink, safety, tracking, streaming, recording, and utilities.

The current implementation is no longer just a single script. There is real reuse in `SafetyMonitor`, `MAVLinkClient`, `FrameGrabber`, `VideoRecorder`, `TargetDetection`, the search pattern classes, and the mission stream/recording adapters. That is good engineering progress.

However, the system is still centered around `tracker.py`, which is a 1,072-line orchestrator. It owns target selection, state transitions, gimbal commands, drone updates, web callbacks, terminal input, stream startup, recording startup, HUD coordination, shutdown, and mutable runtime settings. In flight software terms, this is a modular project with a monolithic mission loop.

Most important current issues:

| Severity | Issue | Why it matters |
|---|---|---|
| Critical | `_detected_ids` now stores `TargetDetection` objects but several web/headless paths still index values as tuples | `ids`, headless ID reporting, `/click`, and `/status` paths can crash during operation |
| High | `pytest -q` fails because `utils/camera_test.py` is collected and opens a UDP socket at import time | The test suite is not runnable with the default command |
| High | App-level geofence checks validate current drone position but not the commanded target position | The app can send a position target outside the configured fence and rely on ArduPilot as the backstop |
| High | Web UI does not append `STREAM_TOKEN` to control fetches | If token auth is enabled, browser control buttons break |
| Medium | `config/settings.py` exists but the runtime still mutates global `config` values | CLI/config behavior depends on import order and is harder to test |
| Medium | Utility scripts duplicate model paths, camera IPs, and run hardware/network code at import time | Reuse is weak outside the core runtime; scripts are brittle on new machines |
| Medium | Empty `.git/` directory is present but the folder is not a valid Git repository | Change tracking and deployment hygiene are broken |

Overall field-readiness rating: 6/10 for gimbal-only testing, 4/10 for autonomous drone body following until the high-severity items are fixed and SITL/HIL tests are added.

## 2. Scope And Checks Performed

Workspace reviewed:

```text
/home/ai-engineer/Akash/Tasks/Akash/AI/Jetson/person_tracking
```

Commands/checks run:

| Check | Result |
|---|---|
| Repository/file mapping with `find`, `rg --files`, and line counts | Completed |
| Read core runtime modules: `main.py`, `tracker.py`, `control/`, `tracking/`, `mavlink_client/`, `safety/`, `gimbal/`, `gcs/`, `mission/`, `utils/` | Completed |
| `pytest -q` | Failed during collection because `utils/camera_test.py` opens a socket at import time |
| `pytest -q tests` | Passed: 51 tests in 2.18 seconds |
| `find . -name '*.py' ... python3 -m py_compile` | Passed |
| `python3 -c "import main; import tracker; import control.drone_controller"` | Passed |
| Git status | Failed because `.git/` is empty/invalid |

Not performed:

- No real SIYI gimbal, RTSP camera, MAVLink serial device, GPS, ArduPilot SITL, or flight test was executed.
- No TensorRT inference performance test was run.
- No manual browser test of the GCS stream was run.

## 3. Current Project Structure

Observed top-level layout:

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
|-- mission/
|-- models/
|-- safety/
|-- scripts/
|-- tests/
|-- tracking/
|-- utils/
|-- readme.md
|-- PROJECT_AUDIT_REPORT.md
|-- IMPLEMENTATION_PLAN.md
`-- md.md
```

Selected file sizes:

| File | Lines | Comment |
|---|---:|---|
| `tracker.py` | 1,072 | Main integration hot spot |
| `mavlink_client/mavlink_client.py` | 607 | Central MAVLink interface |
| `control/drone_controller.py` | 555 | Drone body following and failsafe logic |
| `gcs/stream_server.py` | 550 | HTTP/MJPEG UI and API |
| `config/config.py` | 265 | Global constants |
| `config/settings.py` | 116 | Intended immutable settings object, not yet used by runtime |

Model assets currently in the repo:

| File | Size |
|---|---:|
| `models/yolo26s.onnx` | 38.2 MB |
| `models/yolo26s.engine` | 22.5 MB |
| `models/yolo26s.pt` | 20.4 MB |
| `models/best.pt` | 5.4 MB |
| `models/bestjetson.pt` | 5.4 MB |

## 4. Structure Review

### What Is Good

| Area | Positive observation |
|---|---|
| Domain folders | Code is split into `control`, `tracking`, `gimbal`, `mavlink_client`, `safety`, `gcs`, `mission`, and `utils` |
| MAVLink boundary | `mavlink_client/mavlink_client.py` is the single place that opens and manages pymavlink |
| Safety boundary | `safety/safety.py` centralizes altitude, speed, geofence, heartbeat, and battery checks |
| Detection wrapper | `detection/detector.py` hides Ultralytics/ByteTrack calls from the tracker |
| Mission helpers | `mission/recording_manager.py` and `mission/stream_adapter.py` reduce duplicated frame handling |
| Typed target data | `tracking/target_detection.py` replaces fragile positional target tuples with a dataclass |
| Tests now exist | `tests/` has 51 passing tests when run directly |

### What Needs Improvement

| Severity | Issue | Evidence | Impact | Recommended fix |
|---|---|---|---|---|
| High | `tracker.py` remains too large and multi-responsibility | `tracker.py:64`, `tracker.py:204`, `tracker.py:515`, `tracker.py:592`, `tracker.py:740`, `tracker.py:1001` | Bugs in web, input, detection, state, or shutdown all land in one file | Split target selection, input handling, web callbacks, state machine, and mission lifecycle into separate classes |
| Medium | Invalid Git repository | `.git/` exists but `git rev-parse --show-toplevel` fails | No reliable change history, diffs, blame, or deployment provenance | Remove the empty `.git/` stub and initialize or restore a real Git repo |
| Medium | Generated caches are in source tree | `__pycache__/` exists in most packages, `.pytest_cache/` exists | Noise and deployment confusion | Add `.gitignore`; remove caches from versioned/deployable source |
| Medium | No dependency manifest found | No `requirements.txt`, `pyproject.toml`, `setup.py`, lockfile, or Pipfile | Jetson reproduction will be inconsistent | Add a pinned dependency manifest, with JetPack-specific notes for PyTorch/OpenCV/Ultralytics |
| Low | Historical audit files conflict with current code | `PROJECT_AUDIT_REPORT.md`, `IMPLEMENTATION_PLAN.md`, `md.md` contain older/fixed findings | Future developers may chase stale bugs | Mark historical docs as archived or keep only the current audit and current implementation plan |

## 5. Code Reuse Assessment

Estimated reuse score:

| Area | Score | Reason |
|---|---:|---|
| Core runtime | 7/10 | Most hardware/domain concepts are now modules/classes |
| Mission orchestration | 4/10 | `tracker.py` still coordinates too many concerns |
| Configuration | 4/10 | Constants are centralized, but mutable globals are used everywhere |
| Utility scripts | 2/10 | Camera/model constants and hardware probes are duplicated |
| Tests | 5/10 | Basic tests exist, but some duplicate logic instead of testing production classes |
| Overall | 5.5/10 | Good modular foundation, incomplete migration |

### Reuse That Is Working

| Component | Reuse status |
|---|---|
| `SafetyMonitor` | Good. Used by MAVLink and drone control for speed, altitude, geofence, heartbeat, and battery checks |
| `MAVLinkClient` | Good. Encapsulates telemetry cache, command ACKs, mode commands, and position/velocity sends |
| `FrameGrabber` | Good. Shared by main loop, recording, and streaming |
| `VideoRecorder` | Good. Encoder fallback logic is centralized |
| `TargetDetection` | Good direction. Reduces positional tuple mistakes where migration is complete |
| Search pattern classes | Good. `start/stop/get_command` interface is reusable |
| `RecordingManager` and `StreamAdapter` | Moderate. Extracted from `tracker.py`, but still depend on tracker-owned callbacks/state |

### Reuse That Is Weak

| Weak area | Evidence | Recommendation |
|---|---|---|
| Runtime config | `main.py:152-168` mutates `config` globals; many modules import `config as cfg` | Finish the `Settings` migration and inject settings into constructors |
| Utility scripts | Older utility scripts previously hardcoded `/home/ai2/Desktop/person_tracking/...` paths | Fixed for current scripts: utilities now reuse central `config` defaults; next improvement is converting all hardware tools to full CLI apps |
| Test helpers | `tests/test_standoff.py` duplicates standoff math instead of calling `DroneController` behavior | Add pure helper methods or dependency-injected fakes so production code is directly tested |
| Web query parsing tests | `tests/test_query_parse.py` tests `urllib.parse`, not the `StreamServer` handler | Add tests against handler-level parsing/auth behavior |
| Target data migration | `TargetDetection` exists, but old tuple indexing remains in several paths | Finish migration by using named fields everywhere |

## 6. Critical And High-Severity Technical Bugs

### H1. `_detected_ids` Type Mismatch Can Crash Web And Headless Control

Evidence:

- `_detected_ids` comment still says tuple: `tracker.py:115`
- `_select_target()` stores `TargetDetection` values: `tracker.py:236-241`
- Terminal `ids` command uses tuple indexing: `tracker.py:541-542`
- Web click uses tuple indexing: `tracker.py:620`
- Headless ID report uses tuple indexing: `tracker.py:868`
- `TargetDetection` has named fields only and no `__getitem__`: `tracking/target_detection.py`

Impact:

- Terminal command `ids` can raise `TypeError: 'TargetDetection' object is not subscriptable`.
- `/click` can fail when a browser click is processed.
- Headless SSH operation can crash or silently exit the stdin thread depending on where the exception occurs.
- This is especially risky because headless/web control paths are important in field operation.

Recommended fix:

Use named attributes everywhere:

```python
print(f"ID {tid:3d} center=({d.cx:.0f},{d.cy:.0f}) conf={d.conf:.2f}")
x1, y1, x2, y2 = data.x1, data.y1, data.x2, data.y2
```

Also update the `_detected_ids` type annotation to:

```python
self._detected_ids: dict[int, TargetDetection] = {}
```

Add tests for:

- `handle_web_click()`
- `handle_web_click(None, None)` status path
- terminal/headless ID formatting helper after extracting it from `_stdin_reader`

### H2. Default `pytest -q` Fails Due Hardware Side Effects In `utils/camera_test.py`

Evidence:

- `pytest -q` failed during collection.
- Error: `PermissionError: [Errno 1] Operation not permitted`
- Cause: `utils/camera_test.py:57` opens a UDP socket at module import time.

Impact:

- The normal developer test command is broken.
- CI will fail before running the intended tests.
- Hardware/network side effects happen simply by importing/collecting a file.

Recommended fix:

- Rename `utils/camera_test.py` to something not matching pytest collection, for example `utils/siyi_camera_probe.py`.
- Move executable code under:

```python
def main() -> int:
    ...

if __name__ == "__main__":
    raise SystemExit(main())
```

- Add a `pytest.ini` or `pyproject.toml` with:

```ini
[pytest]
testpaths = tests
python_files = test_*.py
```

### H3. App-Level Geofence Checks Current Position, Not Commanded Target

Evidence:

- Current GPS is checked here: `control/drone_controller.py:226`
- If current position is inside the fence, the controller computes a standoff target: `control/drone_controller.py:299-305`
- It sends that target directly: `control/drone_controller.py:307`
- `MAVLinkClient.send_position_velocity_ned()` clamps velocity only: `mavlink_client/mavlink_client.py:531`

Impact:

- If the person is outside or near the edge of the geofence, the app can command a target outside the fence.
- ArduPilot onboard fence is still an important backstop, but the app-level `SafetyMonitor` is documented as the single authority and currently does not enforce target-position safety.
- This is high risk for autonomous following.

Recommended fix:

- Add a safety method that validates/clamps NED targets against the geofence center/radius before sending.
- Convert target NED to GPS with `PersonEKF.ned_to_gps()` or perform a direct NED-radius check when origin and fence center are the same.
- If the target is outside the fence, project it back onto the safe circle minus a margin.
- Add tests for target positions inside, outside, and exactly on the boundary.

### H4. Stream Token Authentication Breaks Browser Controls

Evidence:

- Control endpoints require token when `cfg.STREAM_TOKEN` is set: `gcs/stream_server.py:401-408`
- Browser fetches do not append a token: `gcs/stream_server.py:264`, `gcs/stream_server.py:280`, `gcs/stream_server.py:301`, `gcs/stream_server.py:319-320`

Impact:

- If `--stream-token` is used, the web UI loads but click-to-track, zoom, mode, and gimbal controls receive 403 responses.
- Operators may think the tracker is broken when auth is enabled.

Recommended fix:

- Generate the token into the HTML as a JS constant and append it to control URLs.
- Or require `/?token=...` and store it in browser memory for subsequent fetches.
- Add a test that control URLs include the token when configured.

### H5. Hardware Utility Scripts Are Import-Unsafe And Not Reusable

Evidence:

- `utils/yolo_test_simple.py`, `utils/yolo_debug_optimized.py`, `utils/yolo_gpu_optimized.py`, `utils/diagnostic.py`, `utils/jetson_cuda_diagnostic.py`, and `utils/rtsp_explorer.py` contain top-level execution code.
- Older versions hardcoded old model paths in `utils/yolo_gpu_optimized.py`, `utils/yolo_debug_optimized.py`, `utils/yolo_test_simple.py`, and `utils/jetson_cuda_diagnostic.py`; these have been changed to use `config`.
- Camera/gimbal defaults in utility scripts now come from `config`.

Impact:

- These files cannot be safely imported by tests or other tooling.
- They are still hardware-facing scripts, but they no longer depend on the old `/home/ai2/Desktop/person_tracking/...` path.
- Reuse is low because each script redefines camera/model settings.

Recommended fix:

- Convert each script into a `main()` with `argparse`.
- Import central config defaults.
- Keep hardware calls inside `main()`.
- Move exploratory scripts into `tools/` or `scripts/` and keep `utils/` for import-safe helpers.

## 7. Medium-Severity Technical Issues

### M1. Mutable Global Configuration Is Still The Main Coupling Mechanism

Evidence:

- `config/settings.py` says it should replace direct mutable `config` reads.
- `main.py:152-168` mutates config globals after parsing CLI args.
- Most modules still import `config as cfg`.
- `utils/frame_grabber.py:20` imports `RTSP_URL` directly and binds it as a default at import time.

Impact:

- Import order matters.
- Tests need monkeypatching of module globals.
- Multiple tracker instances with different settings are impossible.
- Future services/systemd deployments can accidentally use stale defaults.

Recommended fix:

- Use `load_settings()` in `main.py`.
- Pass `settings` into `PersonGimbalTracker`, `Detector`, `FrameGrabber`, `SIYIController`, `StreamServer`, `SafetyMonitor`, `MAVLinkClient`, and `DroneController`.
- Keep `config/config.py` as defaults only.

### M2. Tests Exist But Do Not Cover The Main Mission Loop

Current good result:

```text
pytest -q tests
51 passed in 2.18s
```

Remaining gaps:

| Gap | Why it matters |
|---|---|
| No tests for `PersonGimbalTracker._select_target()` | Would catch the `_detected_ids` tuple/dataclass bug |
| No tests for `handle_web_click()` | Would catch web click crash |
| No tests for `StreamServer` token behavior | Would catch auth UX breakage |
| `tests/test_standoff.py` duplicates logic | It may pass even when production `DroneController` changes incorrectly |
| No SITL/MAVLink integration tests | Autonomous drone control remains unvalidated |
| No recorded-video detection regression tests | YOLO/ByteTrack/PersonRegistry behavior can regress silently |

Recommended fix:

- Add pure tests around extracted target selection and web-control helpers.
- Add fake `MAVLinkClient`, fake `SafetyMonitor`, and fake `PersonEKF` tests for `DroneController`.
- Add ArduPilot SITL smoke test for mode gate, velocity sends, LOITER, RTL, and fence breach handling.

### M3. `tracker.py` Still Owns Too Much Runtime State

Evidence:

- `tracker.py:64` defines the main class.
- Target selection: `tracker.py:204`
- Terminal input: `tracker.py:515`
- Web control: `tracker.py:592`
- Main loop: `tracker.py:740`
- Keyboard handler: `tracker.py:1001`

Impact:

- The class is difficult to test without camera/gimbal/MAVLink dependencies.
- Small UI changes can accidentally affect flight control state.
- Shutdown, streaming, detection, and control timing are tightly coupled.

Recommended refactor:

| Extract | Responsibility |
|---|---|
| `TargetSelector` | YOLO result parsing, registry update, lock/largest-person selection |
| `TrackerStateMachine` | WAITING/TRACKING/PREDICTING/SEARCH transitions |
| `OperatorInputController` | keyboard and terminal command handling |
| `WebControlAdapter` | click/mode/gimbal/zoom callbacks |
| `MissionRuntime` | start/stop threads, lifecycle, cleanup |

### M4. Dependency And Deployment Metadata Is Missing

Evidence:

- No `requirements.txt`, `pyproject.toml`, `setup.py`, `Pipfile`, or lockfile was found.
- No repo-root `.gitignore` was found.

Impact:

- Jetson setup depends on scripts and tribal knowledge.
- CI and deployment cannot reliably recreate the runtime.
- Generated bytecode and model artifacts are likely to be mixed with source files.

Recommended fix:

- Add `pyproject.toml` or `requirements.txt`.
- Add JetPack-specific install notes for PyTorch and OpenCV.
- Add `.gitignore` for `__pycache__/`, `.pytest_cache/`, logs, recordings, and local model artifacts.
- Use Git LFS or a deployment artifact fetch step for model binaries.

### M5. Existing Documentation Has Stale Statements

Evidence:

- `PROJECT_AUDIT_REPORT.md` says no tests exist, but `tests/` now exists and passes when invoked directly.
- `md.md` contains older safety-gap notes, some of which are now implemented.
- `utils/diagnostic.py` now points operators to `python3 main.py` for the full tracker.

Impact:

- Developers may implement already-fixed work or run nonexistent commands.
- Field operators may lose trust in the docs.

Recommended fix:

- Archive historical reports under `docs/archive/`.
- Keep one current audit and one current remediation plan.
- Update `readme.md` with the tested command set and known limitations.

## 8. Drone-Specific Safety Review

### Safety Features Currently Implemented

| Feature | Evidence |
|---|---|
| Central safety monitor | `safety/safety.py` |
| Heartbeat watchdog | `safety/safety.py`, `mavlink_client/mavlink_client.py:171` |
| GUIDED mode gate | `control/drone_controller.py:168-174` |
| Armed gate | `control/drone_controller.py:177-178` |
| Home-position gate | `control/drone_controller.py:181-183` |
| Sensor health gate | `control/drone_controller.py:186-192` |
| Battery critical RTL retry | `control/drone_controller.py:195-215` |
| GPS fix gate | `control/drone_controller.py:218-221` |
| Current-position geofence check | `control/drone_controller.py:226-247` |
| ArduPilot fence breach handling | `control/drone_controller.py:250-253` |
| Finite MAVLink command guard | `mavlink_client/mavlink_client.py:543-547` |
| Velocity safety clamp | `mavlink_client/mavlink_client.py:507`, `mavlink_client/mavlink_client.py:531` |
| ACK-confirmed mode commands | `mavlink_client/mavlink_client.py:429`, `mavlink_client/mavlink_client.py:480`, `mavlink_client/mavlink_client.py:569`, `mavlink_client/mavlink_client.py:587` |

### Remaining Drone Safety Gaps

| Severity | Gap | Recommendation |
|---|---|---|
| High | Commanded position targets are not app-geofence checked | Clamp or reject target NED positions before `send_position_velocity_ned()` |
| Medium | Flight-controller parameter assumptions are documented but not verified | Query and validate `GUID_TIMEOUT`, `WPNAV_SPEED`, `FENCE_ENABLE`, `FENCE_ALT_MIN`, and related parameters before enabling drone following |
| Medium | No SITL/HIL test path | Add ArduPilot SITL tests with fake detections and mode/fence/battery transitions |
| Medium | Battery cell auto-detection uses pack voltage heuristics | Allow explicit cell count override and log confidence/ambiguity |
| Medium | Failsafe behavior is not operator-visible beyond terminal prints | Expose drone mode, failsafe state, battery, GPS, and fence status in the web UI/API |

## 9. Recommended Remediation Plan

### P0 - Fix Before Next Field Test

1. Fix `_detected_ids` tuple/dataclass mismatch in `tracker.py`.
2. Make `pytest -q` pass by moving hardware code under `main()` and adding pytest config.
3. Add app-level geofence target validation before position commands.
4. Fix `STREAM_TOKEN` handling in generated web fetch URLs.
5. Add `.gitignore` and restore/init a valid Git repository.
6. Remove or quarantine generated `__pycache__/` and `.pytest_cache/` from source.

### P1 - Improve Testability And Reuse

1. Finish `Settings` injection and stop mutating `config` globals.
2. Extract `TargetSelector`, web callbacks, and terminal command logic from `tracker.py`.
3. Convert hardware diagnostic utilities into import-safe CLI tools.
4. Replace duplicated test logic with tests that call production methods through fakes.
5. Add handler-level tests for stream auth and click-to-track.

### P2 - Flight Readiness

1. Add ArduPilot SITL tests for GUIDED, LOITER, RTL, fence breach, battery critical, heartbeat loss, and GPS loss.
2. Add a preflight readiness checklist in code before drone following can be enabled.
3. Add recorded-video regression tests for target selection and re-identification.
4. Add structured logging for mission events instead of relying on `print()`.
5. Move models to Git LFS or an artifact download/deployment step.

## 10. Quick Win Patch List

These are small, high-impact changes:

| File | Change |
|---|---|
| `tracker.py` | Replace all `_detected_ids` tuple indexing with `TargetDetection` attributes |
| `tracker.py` | Change `_detected_ids` type annotation/comment to `dict[int, TargetDetection]` |
| `utils/camera_test.py` | Rename and wrap in `main()` |
| `pyproject.toml` or `pytest.ini` | Set `testpaths = tests` |
| `gcs/stream_server.py` | Add token to JS fetch URLs when `STREAM_TOKEN` is set |
| `control/drone_controller.py` | Check/clamp target position against geofence before send |
| `.gitignore` | Ignore caches, logs, recordings, local envs, and generated model outputs |
| `requirements.txt` or `pyproject.toml` | Pin runtime/test dependencies |

## 11. Final Assessment

The project has the right building blocks and several important safety improvements already in place. The main risk now is incomplete migration: typed target data exists but old tuple call sites remain; immutable settings exist but mutable globals still drive runtime; mission adapters exist but `tracker.py` still owns most behavior.

For gimbal-only operation, the project is close to practical test readiness after fixing the `_detected_ids` bug and pytest collection. For autonomous drone body following, I would not approve field testing until app-level geofence target validation, stream auth behavior, and SITL/HIL coverage are addressed.
