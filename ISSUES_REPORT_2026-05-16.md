# Field Issues Report — 2026-05-16

> **Historical note (added later):** the tracker commands `arm` / `disarm` quoted
> verbatim from log files below were subsequently renamed to `follow` / `unfollow`
> to eliminate the FCU-arm collision that contributed to the operator confusion
> documented here. Log files from this date still contain the old wording.

Sessions analysed:

| # | Log | Lines | Notes |
|---|---|---|---|
| 1 | `gimbal_track_2026-05-15_17-13-05.log` | 1028 | Pre-fix run. NumPy/SciPy ABI mismatch was the dominant issue. |
| 2 | `gimbal_track_2026-05-16_16-30-14.log` | 1404 (truncated mid-run) | Post-fix flight. MAVLink connected. Three `arm` attempts refused. |
| 3 | `gimbal_track_2026-05-16_16-42-11.log` | 222 | Short flight. GUIDED mode never confirmed. |

## Root cause of the operator's three complaints

### 1. `arm` command refused

Three `arm` attempts in log 2 (lines 174, 272, 331). Each was rejected by the tracker's preflight gate. Reason (verbatim from log 2:174):

```
[Cmd] arm refused: preflight failing: Vehicle ARMED, HOME position set,
                   GPS fix OK, RC transmitter connected, ArduPilot params correct
```

Preflight result snapshot (log 2:100–112):

| Check | Status | Reason |
|---|---|---|
| MAVLink connected | OK | link up |
| Heartbeat fresh | OK | |
| Flight mode = GUIDED | OK | (varies — see #3) |
| Vehicle ARMED | **FAIL** | FCU reports disarmed (RC pilot must arm) |
| HOME position set | **FAIL** | FCU has not advertised HOME yet |
| GPS fix OK | **FAIL** | EKF_STATUS_REPORT stale/missing |
| IMU / mag / baro healthy | OK | |
| RC transmitter connected | **FAIL** | RC_CHANNELS never received |
| Geofence not breached | OK | |
| Battery above critical | OK | |
| ArduPilot params correct | **FAIL** | FENCE_ENABLE=0 (must be 1), +3 more |

The tracker's `arm` is gated on a green preflight. Five of eleven checks were red, so refusal is correct.

### 2. `takeoff` command never reached the parser

Across all three logs, `takeoff` appears only in the startup banner (the help line). There is no `[Cmd] takeoff` line or `[Cmd] takeoff refused: ...` line in any session. The command was either never typed, never submitted (no Enter), or `arm` was refused first and the operator did not attempt `takeoff`.

If `takeoff` had been issued, it would have been refused for the same reasons plus `disarmed` and `not in landed-state ON_GROUND` (the takeoff path requires FCU armed + landed; see `GROUND_TEST_COMMANDS.md` section 8 notes).

### 3. Pressing `t` did not enable tracking

Log 3 line 172:

```
[Cmd] unknown: 't' — type 'help' for the list
```

Cause: `t` is a **display-window keyboard shortcut** (cv2.waitKey), not a stdin command. Both sessions ran headless (`[Display] No X11 display — running headless`), so there is no display window to receive `t`. From stdin the operator must type `track on` / `track off`.

This is a UX trap: the startup banner lists `t=Track` under "Keys:" without saying it only applies to the display window.

## System-level issues observed

### Critical

**C1. ArduPilot mode flapping (log 2 has 100+ transitions).**
Tracker sets GUIDED; autopilot leaves GUIDED within ~1 second; tracker re-sends; cycle repeats. Sample sequence (log 2:114–117):

```
[MAVLink] Flight mode: GUIDED → Mode(0x00000004)
[MAVLink] ⚠ Left GUIDED mode — operator override or failsafe
[MAVLink] Flight mode: Mode(0x00000004) → Mode(0x000000c0)
[MAVLink] Flight mode: Mode(0x000000c0) → GUIDED
```

The RC pilot was flying actively while the tracker tried to hold GUIDED. The two are mutually exclusive: tracker control requires GUIDED with sticks centred; RC stick deflection or a non-GUIDED switch position kicks the autopilot out. Result: even if `arm` had passed, tracker velocity setpoints would have had no effect.

**C2. GUIDED mode not confirmed at all (log 3:36–39).**
Three SET_MODE attempts, all ACK timeouts:

```
[MAVLink] ACK timeout command=176 attempt=1/3
[MAVLink] ACK timeout command=176 attempt=2/3
[MAVLink] ACK timeout command=176 attempt=3/3
[MAVLink] WARNING: GUIDED mode not confirmed by autopilot
```

`command=176` is `MAV_CMD_DO_SET_MODE`. Likely cause: autopilot's RC pilot switch was on a non-GUIDED position when the tracker started, and pilot-switch position wins. Operator must put the RC mode switch on GUIDED (or any non-conflicting position) before launching the tracker, or accept that auto-following will not work until the RC switch is moved.

**C3. RC_CHANNELS never received over MAVLink.**
Operator says they are flying via RC, but the tracker never sees an `RC_CHANNELS` MAVLink message — hence the preflight `RC transmitter connected FAIL`. The transmitter is bound to the autopilot but the autopilot is not streaming RC_CHANNELS to the companion computer. Fix in ArduPilot params:

- `SR1_RC_CHAN` (or `SR2_RC_CHAN` depending on which TELEM port `/dev/ttyTHS1` is wired to) → set to a non-zero Hz, e.g. `5`.
- Confirm with `mavproxy` `set requestdatastream RC_CHANNELS 5 1`.

### High

**H1. ArduPilot `FENCE_ENABLE = 0`.**
Preflight requires `FENCE_ENABLE = 1` (`[FAIL] ArduPilot params correct — FENCE_ENABLE: 0.0 == 1.0 — onboard geofence must be enabled`). +3 other param mismatches not enumerated in the log; run `preflight` with verbose output or check `mavlink_client/param_verifier.py` for the full required-param list. Set the required params via Mission Planner/QGC and write to autopilot.

**H2. No GPS lock indoors.**
`GPS fix OK FAIL: EKF_STATUS_REPORT stale/missing`. The tracker cannot follow without GPS, regardless of the other preflight items. Outdoor test required.

**H3. Battery cell count ambiguous (logged ~70 times in log 2).**
`16367mV / 3700mV = 4.42, confidence=low` — code can't decide between 4S and 5S. Defaults to 4S which is correct (~16.4 V on a 4S LiPo). Spams the log every ~2 s. Operator can suppress with `--cells 4`.

**H4. GStreamer plugins still not installed on the Jetson.**
All three logs show `[GRAB] WARN: GStreamer unavailable — using FFmpeg (higher latency)` and `[REC] HW encoder (nvv4l2h264enc) unavailable → x264enc unavailable → mp4v fallback`. Adds video latency and burns CPU on software H.264 encoding instead of using NVENC.

Fix: `sudo apt install -y gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav python3-gst-1.0` (already documented in the inline error message since commit `cf47f06`).

### Medium

**M1. Log 2 truncated at exactly 102400 bytes.**
The file ends mid-mode-transition with no `[Cleanup]` / `[Log] Session end` lines. Indicates the process was killed (SIGKILL or OOM) rather than shut down normally, or the log file write buffer was lost. No traceback in the log. Worth checking `dmesg` / `journalctl` on the Jetson for OOM-killer activity.

**M2. Unknown flight-mode hex values printed as `Mode(0x00000004)` and `Mode(0x000000c0)`.**
The MAVLink client's mode-name table is missing entries for these values. `0x4` is custom_mode=4 = GUIDED (the named mode the log also prints separately), and `0xc0` looks like a base_mode bitfield (SAFETY_ARMED|MANUAL_INPUT_ENABLED) being printed instead of a custom_mode. Either an intermediate-state print or a mapping bug in the mode decoder. Cosmetic, but makes the flapping log harder to read.

**M3. TensorRT engine portability warning (logs 2 and 3).**
`Using an engine plan file across different models of devices is not recommended`. The `yolo26s.engine` was built on a different Jetson SKU or different TensorRT version. Inference still works but may be sub-optimal. Rebuild on the target Orin.

### Low

**L1. `Ctrl-C` traceback in logs 1, 2, 3.**
Operator quitting with Ctrl-C instead of `q` leaves a cv2 `KeyboardInterrupt` traceback. Cosmetic.

**L2. `[Cmd] unknown: 'trac1'` (log 3:94).**
Typo from interleaved stdin during noisy startup output. Cosmetic.

**L3. Ultralytics settings file recreated each session (log 3:22).**
`/tmp/ultralytics` is tmpfs → wiped on reboot. Cosmetic.

**L4. ArduPilot heartbeat from a single compid (0).**
`sysid=1 compid=0` — autopilot identifies as compid 0 rather than the conventional `MAV_COMP_ID_AUTOPILOT1 = 1`. Pymavlink handles this transparently but it's worth knowing if anything filters by compid.

## What the operator needs to do for arm/takeoff/track to work

Sequence, in order. Skipping any step blocks the next.

1. **Outdoors, GPS lock acquired** before launching the tracker. Indoor bench cannot pass preflight.
2. **RC mode switch on GUIDED** (or any position that does not actively force a different mode). If the RC switch picks `STABILIZE`/`LOITER`/`POSHOLD`, the autopilot ignores `SET_MODE GUIDED`.
3. **ArduPilot params corrected** via Mission Planner / mavproxy:
   - `FENCE_ENABLE = 1`
   - The 3 unnamed param mismatches — list them with `preflight` and fix.
   - `SR1_RC_CHAN` (or `SR2_RC_CHAN`) ≥ 5 so RC_CHANNELS streams over MAVLink.
4. **RC pilot arms the FCU** with the throttle-down-rudder-right gesture (or whatever the airframe uses). The tracker's `arm` does **not** arm the FCU — it only arms the tracker's authority to send velocity setpoints. The FCU must already be armed via RC.
5. **Launch the tracker**: `python3 main.py --drone --stream-host 0.0.0.0 --cells 4`. Wait ~5 s for telemetry to fill in.
6. From stdin: `preflight` — confirm 11/11 OK (or all OK that aren't legitimately failable).
7. From stdin: `arm` — should now succeed.
8. From stdin: `takeoff 5` — FCU climbs to 5 m. The tracker auto-switches to GUIDED if it isn't already.
9. From stdin: `ids` to see persistent IDs, then `track <id>`. The drone will follow the locked person. **Do not press `t`** — use `track on` / `track off` instead.
10. To abort at any point: `estop` from stdin (BRAKE on first press, LAND on second within 3 s), and have the RC pilot ready to flip the mode switch to a manual mode as the primary recovery.

## Summary table

| Severity | Item | Action owner |
|---|---|---|
| Critical | Mode flapping under RC pilot input | Operator (set RC switch to GUIDED) |
| Critical | GUIDED mode not confirmed at startup | Operator (same) |
| Critical | RC_CHANNELS not streamed | ArduPilot params (`SR*_RC_CHAN`) |
| High | `FENCE_ENABLE=0` + 3 other param fails | ArduPilot params |
| High | No GPS lock | Move outdoors |
| High | Battery cell ambiguity spam | Operator (`--cells 4`) |
| High | GStreamer plugins not installed | Host (`sudo apt install ...`) |
| Medium | Log 2 truncated — possible process kill | Investigate `dmesg`/`journalctl` |
| Medium | Mode-name decoder shows raw hex | Code (mavlink_client mode table) |
| Medium | TensorRT engine portability warning | Rebuild engine on target Orin |
| Low | Ctrl-C traceback | Operator (use `q`) |
| Low | `t` keypress unknown in headless mode | Documentation / banner clarification |
| Low | Ultralytics settings in `/tmp` | Code (move to persistent path) |

## Operator FAQ

**Q: I typed `arm` and got refused. What do I do?**
A: Run `preflight` first. Fix every FAIL line before retrying `arm`.

**Q: I pressed `t` to track. Why didn't it work?**
A: `t` only works in the display window (you're running headless). From stdin: `track on`. To lock a specific person: `ids` then `track <id>`.

**Q: I'm flying via RC and the tracker is fighting me — what's happening?**
A: The tracker is sending `SET_MODE GUIDED` every loop; the autopilot is leaving GUIDED because your RC mode switch isn't on GUIDED. You cannot fly RC and have the tracker control simultaneously. Pick one: RC pilot flies → tracker is gimbal-only; tracker flies → RC pilot is hands-off in GUIDED.

**Q: The drone never armed even when I tried.**
A: The tracker's `arm` does not arm the FCU. The RC pilot must arm the FCU first (sticks gesture). The tracker `arm` only enables the tracker to send velocity setpoints to the already-armed FCU.
