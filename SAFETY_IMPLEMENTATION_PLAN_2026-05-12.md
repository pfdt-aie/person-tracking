# Safety Hardening Implementation Plan — Real-Person Ground Testing

Plan date: 2026-05-12
Trigger: Safety review of current code in preparation for real-person ground testing.
Scope: Close the 20 gaps identified between current safeguards and the minimum bar for live-subject operation.

---

## 0. Goals

1. **No live-subject test can begin** until Sprint 1 lands (software interlocks + dry-run).
2. **No motors-armed test near a person** until Sprints 1 + 2 land.
3. Every change is **testable on the bench** (mocked MAVLink) before flight.
4. No behavior change unless explicitly enabled by a CLI flag or settings update — defaults remain conservative.

---

## 1. Sprint 1 — Ground-Test Blockers (P0)

These must merge **before** the next real-person test, motors off or on.

### S1.1  Software E-STOP

| Item | Detail |
|---|---|
| Where | [gcs/stream_server.py](gcs/stream_server.py), [gcs/web_control_adapter.py](gcs/web_control_adapter.py), [control/drone_controller.py](control/drone_controller.py) |
| Add | `POST /estop` endpoint → calls `MAVLinkClient.send_brake()` then `send_land()` if BRAKE not acked in 1 s |
| UI | Red full-width E-STOP button at top of web UI, double-tap confirm, keyboard shortcut `Space` while web focused |
| Auth | Always permitted, token-bypass for E-STOP |
| Logging | Structured event `estop.fired` with timestamp + source |
| Test | Unit test for endpoint, integration test against fake MAVLink |

### S1.2  `--ground-test` Dry-Run Mode

| Item | Detail |
|---|---|
| Where | [main.py](main.py), [control/drone_controller.py](control/drone_controller.py), [mavlink_client/mavlink_client.py](mavlink_client/mavlink_client.py) |
| Behavior | When `--ground-test` set: all `send_position_velocity_ned`, `send_velocity_ned`, `send_zero_velocity`, mode-change calls are intercepted, logged with intended args, **never transmitted** |
| Telemetry | MAVLink reads still happen so operator sees real GPS/batt/mode |
| HUD | Banner "GROUND TEST — COMMANDS SUPPRESSED" |
| Test | Test that no MAVLink TX occurs while `_ground_test=True` |

### S1.3  Preflight Readiness Gate

| Item | Detail |
|---|---|
| Where | New `safety/preflight.py`, integrated in [tracker.py](tracker.py) before `drone_tracking_enabled=True` |
| Checks | GUIDED-capable + ARMED + HOME set + GPS fix ≥ 3D + HDOP ≤ 1.5 + sats ≥ 10 + sensors healthy + fence param verified + batt > warn + heartbeat fresh + EKF healthy |
| UX | Web UI shows checklist; "Arm Tracker" button disabled until all green; explicit operator confirmation required |
| Test | Each gate has unit test with mocked telemetry |

### S1.4  Vertical Separation + Retreat Behavior

| Item | Detail |
|---|---|
| Where | [control/drone_controller.py:288](control/drone_controller.py#L288), [config/config.py](config/config.py) |
| Add | `MIN_VERTICAL_SEP_M = 4.0` constant. Compute `alt_above_person = drone_alt_agl − person_terrain_alt` (default person terrain = 0). Reject command if `< MIN_VERTICAL_SEP_M` |
| Retreat | When horizontal sep < `MIN_PERSON_DRONE_SEP_M`, send velocity *away* from person at `RETREAT_SPEED_MS=1.0` instead of hover, until `sep ≥ MIN_PERSON_DRONE_SEP_M + 1 m` hysteresis |
| Test | Test_retreat_when_subject_approaches |

### S1.5  N-Frame Detection Confirm Before Body Movement

| Item | Detail |
|---|---|
| Where | [control/drone_controller.py:131](control/drone_controller.py#L131), new counter in `DroneController` |
| Behavior | Drone body velocity stays 0 until **N consecutive frames** with valid detection (default `BODY_MOVE_CONFIRM_FRAMES=5`). Gimbal still tracks immediately |
| Reset | Counter resets to 0 on tracking loss |
| Test | Test counter requires N frames before nonzero velocity |

### S1.6  Tighten Heartbeat Watchdog

| Item | Detail |
|---|---|
| Where | [config/config.py:216](config/config.py#L216) |
| Change | `HEARTBEAT_WATCHDOG_S` from `3.0` → `1.0`. Add `HEARTBEAT_WARN_S = 0.5` for early operator warning |
| Test | Update existing watchdog tests |

---

## 2. Sprint 2 — Pre-Motors-Armed (P1)

Required before motors-armed test near the subject.

### S2.1  GPS Quality Gate

| Item | Detail |
|---|---|
| Where | [mavlink_client/mavlink_client.py](mavlink_client/mavlink_client.py) `is_gps_ok()` |
| Add | Require fix ≥ 3, HDOP ≤ `GPS_MAX_HDOP=1.5`, sats ≥ `GPS_MIN_SATS=10`, EKF variance ≤ `EKF_MAX_VARIANCE=1.0` |
| Test | Mocked GPS_RAW_INT / EKF_STATUS_REPORT cases |

### S2.2  HOME / Operator Keep-Out Zone

| Item | Detail |
|---|---|
| Where | [safety/safety.py](safety/safety.py) new method `check_home_keepout()` |
| Add | `HOME_KEEPOUT_RADIUS_M = 5.0`; reject target if within radius of HOME (operator stands here) |
| Test | test_target_in_keepout_rejected |

### S2.3  ArduPilot Param Verification At Boot

| Item | Detail |
|---|---|
| Where | New `mavlink_client/param_verifier.py`, called from `main.py` after MAVLink connect |
| Verify | `FENCE_ENABLE=1`, `FENCE_TYPE` bitmask, `FENCE_RADIUS ≥ GEOFENCE_RADIUS_M`, `FENCE_ALT_MIN ≥ MIN_ALT_M`, `RTL_ALT ≥ 20 m`, `BATT_FS_LOW_ACT=2`, `FS_GCS_ENABLE=1`, `GUID_TIMEOUT > 0` |
| Behavior | Refuse to enter "armed tracker" state if any param wrong; print expected vs actual |
| Test | Mocked PARAM_VALUE responses |

### S2.4  EKF Position-Jump Rejection

| Item | Detail |
|---|---|
| Where | [tracking/person_geolocation.py](tracking/person_geolocation.py) `PersonEKF.update()` |
| Add | If incoming measurement is `> EKF_MAX_JUMP_M=10.0` from current state and EKF was valid in prev tick, reject this measurement and log |
| Test | test_ekf_rejects_jump |

### S2.5  Max Horizontal Acceleration Cap

| Item | Detail |
|---|---|
| Where | [control/drone_controller.py:489](control/drone_controller.py#L489) `_apply_smoother()` |
| Add | After jerk limiter, clamp `|a| ≤ MAX_ACCEL_MS2=2.0`. Existing jerk cap stays |
| Test | test_acceleration_capped |

### S2.6  Standoff Bearing Hysteresis

| Item | Detail |
|---|---|
| Where | [control/drone_controller.py:297](control/drone_controller.py#L297) |
| Add | Don't snap-flip the bearing source at the 0.3 m/s threshold. Use slew-rate-limited bearing (max `BEARING_SLEW_DEG_S=30`) and require person velocity above threshold for `BEARING_LATCH_S=1.0` before switching to velocity-based bearing |
| Test | test_bearing_does_not_flip_on_brief_stop |

---

## 3. Sprint 3 — Observability + Resilience (P2)

Quality-of-life and post-incident review. Land after Sprints 1 + 2 are in flight tests.

### S3.1  Failsafe Telemetry In Web UI

| Item | Detail |
|---|---|
| Where | [gcs/stream_server.py](gcs/stream_server.py) `/status` payload, HUD overlay |
| Add | mode, armed, gps_fix/hdop/sats, batt_v/cells/percent, fence_active, ekf_health, last_failsafe_event, tracking_loss_dt |
| Test | test_status_payload_complete |

### S3.2  Structured Flight Log

| Item | Detail |
|---|---|
| Where | New `utils/flight_log.py`. Replace `print()` calls in [safety/](safety/) [control/](control/) [mavlink_client/](mavlink_client/) |
| Format | JSONL, one file per session under `logs/flight_YYYYMMDD_HHMMSS.jsonl` |
| Events | takeoff, mode_change, failsafe, geofence_breach, batt_warn, batt_crit, rtl, loiter, estop, tracking_loss, target_rejected |
| Test | test_log_writes_jsonl |

### S3.3  Max Session-Time RTL

| Item | Detail |
|---|---|
| Where | [control/drone_controller.py](control/drone_controller.py) |
| Add | `MAX_FLIGHT_TIME_S = 600` (10 min default). On expiry → `send_rtl()` |
| Test | test_session_timeout_triggers_rtl |

### S3.4  Battery Cell-Count Override + Confidence

| Item | Detail |
|---|---|
| Where | [safety/safety.py:93](safety/safety.py#L93), [config/config.py](config/config.py) |
| Add | `--cells N` CLI flag overrides auto-detect. Log `cell_count.detected n=X v_per_cell=Y confidence=high|medium|low` |
| Test | test_cli_override_used |

### S3.5  Latency / FPS Floor

| Item | Detail |
|---|---|
| Where | [tracker.py](tracker.py) main loop |
| Add | Rolling EMA of YOLO inference time; if effective FPS < `MIN_TRACKING_FPS=8`, suppress drone body commands and warn |
| Test | test_low_fps_halts_body |

### S3.6  Operator-Visible RC Override Detection

| Item | Detail |
|---|---|
| Where | [mavlink_client/mavlink_client.py](mavlink_client/mavlink_client.py) |
| Add | Subscribe to RC_CHANNELS; detect mode-switch channel change away from GUIDED → log + suppress commands until back in GUIDED + explicit re-arm |
| Test | test_rc_override_suppresses_commands |

---

## 4. Validation Plan

### 4.1 Bench Tests (no aircraft, props off)

1. `pytest -q` — all existing + new unit tests pass.
2. `python3 main.py --ground-test --drone` with mocked MAVLink — verify no TX, all telemetry parsed.
3. Web UI manual test: every button works with token, E-STOP responds < 100 ms.
4. Preflight checklist UX: each gate flips red→green correctly.

### 4.2 SITL (ArduPilot SITL + simulated subject)

1. GUIDED entry, mock detection, verify `BODY_MOVE_CONFIRM_FRAMES` enforced.
2. Trigger geofence breach → drone returns to home.
3. Trigger battery critical → RTL issued and confirmed.
4. Trigger heartbeat loss → drone stops.
5. Trigger E-STOP → BRAKE then LAND.
6. Trigger EKF jump → measurement rejected, no sprint.
7. Trigger RC override → commands suppressed.

### 4.3 First Real-World Test (props off, drone on stand)

1. Full pipeline running, motors disarmed.
2. Walk a person in front of the camera.
3. Observe logged "intended velocity" — verify magnitudes/directions sane.
4. Trigger E-STOP, verify log entry.

### 4.4 First Motors-Armed Test

1. Tethered or weighted, low altitude, no person closer than 15 m.
2. Run for 2 min, verify all failsafes idle.
3. Manually walk into separation envelope — observe retreat.

---

## 5. Risk & Rollback

| Risk | Mitigation |
|---|---|
| New gates lock operator out unintentionally | All gates log the failing condition; web UI shows red item |
| `--ground-test` mode forgotten in real flight | Mode prints banner every 5 s + HUD overlay + log line |
| Param verifier blocks legit setups | Each check has CLI bypass flag, e.g. `--skip-param-check` (never used in production) |
| Retreat behavior pushes drone into obstacle | Retreat is rate-limited (1 m/s) and still subject to geofence checks |

---

## 6. Out Of Scope

These were flagged in the audit but are not part of this plan:

- Refactor of `tracker.py` into smaller classes (separate workstream).
- SITL CI pipeline (separate workstream, blocks Sprint 3).
- Git LFS for model artifacts.
- Documentation cleanup.

---

## 7. Acceptance Criteria

Sprint 1 done when:
- E-STOP button live, < 100 ms response in test.
- `--ground-test` confirmed to suppress all MAVLink TX.
- Preflight checklist blocks "Arm Tracker" until all green.
- Retreat behavior demonstrated on bench with mocked person approach.
- N-frame confirm gate demonstrated.
- Heartbeat watchdog 1.0 s, tests updated.

Sprint 2 done when:
- All P1 unit tests pass.
- SITL run completes the validation sequence in §4.2 without manual intervention.

Sprint 3 done when:
- A real flight produces a complete JSONL log replayable into the web UI.
- Max session-time and FPS floor demonstrated in SITL.
