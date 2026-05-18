# Person-Tracking Drone — Operations Guide

This document is the canonical procedure for operating the SIYI A8 mini +
ArduCopter person-tracking system. Read sections 1–2 before any flight;
sections 3–4 are the operational workflows; sections 5–9 are reference.

If `python main.py` works on your Jetson, use `python`; otherwise use
`python3` in every example below.

> **TEST MODE — `--stream-token` temporarily disabled (2026-05-14).**
> Control-endpoint token enforcement and the no-token loopback fallback
> in [`gcs/stream_server.py`](gcs/stream_server.py) `_check_token()`
> are bypassed for the current test cycle. The examples below assume no
> token. To re-enable, search the codebase for `[TEST-MODE 2026-05-14]`
> (two hits in `gcs/stream_server.py`), uncomment those blocks, then
> add `--stream-token <secret>` to launches and `?token=<secret>` to
> URLs.

---

## 1. Quick Start

| Scenario | Command |
|---|---|
| Camera + AI only (no flight controller) | `python3 main.py --stream-host 0.0.0.0` |
| Bench test with FCU connected, no MAVLink writes | `python3 main.py --drone --ground-test --stream-host 0.0.0.0 --cells 4` |
| Live flight (FCU commands enabled) | `python3 main.py --drone --stream-host 0.0.0.0 --cells 4` |

Open the web UI at `http://<jetson-ip>:5000/` or
`https://ai2-desktop.tailca9f5b.ts.net/`.

The flags:

| Flag | Purpose |
|---|---|
| `--drone` | Enable MAVLink control of the flight controller (FCU) |
| `--ground-test` | Read telemetry but never transmit MAVLink commands |
| `--stream-host 0.0.0.0` | Bind the web UI to all interfaces (LAN + Tailscale) |
| `--cells 4` | Set battery cell count to 4S (suppresses auto-detect warning) |
| `--device /dev/ttyTHS1` | Override the serial port (default already matches the rig wiring) |

---

## 2. Core Concepts

Read this section in full before flying. Two of the operator complaints
from earlier field tests were caused by misunderstanding these
distinctions.

### 2.1 Gimbal tracking is NOT drone-body following

These are two separate features with two different gates:

| | Gimbal tracking | Drone-body following |
|---|---|---|
| What it does | Camera follows the person | Drone flies after the person |
| Command | `track <id>` | `follow` (after `track <id>` and preflight green) |
| Gates | Tracker running | Preflight 11/11, FCU armed, mode GUIDED |
| Safe on bench | Yes | No — would command motor velocity |

**`track <id>` alone only points the gimbal.** The drone body does not
move until you also type `follow`.

### 2.2 Two state layers (do not confuse them)

| Layer | Meaning | Who controls it |
|---|---|---|
| **FCU armed** | The flight controller will spin motors | RC pilot, via the sticks-gesture (throttle-down + rudder-right) |
| **Tracker following** | The Jetson is allowed to send velocity setpoints to the FCU | Operator, via `follow` after preflight passes |

Both are required for autonomous following:

- FCU armed, tracker not following → drone responds to RC sticks only
- Tracker following, FCU disarmed → tracker's commands are ignored by the FCU

> **Naming note.** The tracker's authority gate was previously called
> `arm` / `disarm`, which collided with FCU arming and confused field
> operators. It was renamed to `follow` / `unfollow` (commit history,
> see [`tracker.py`](tracker.py) `_handle_follow`). Typing the old
> word returns `[Cmd] unknown` with a hint pointing at the new one.

### 2.3 The tracker can only fly the drone in GUIDED mode

If the RC pilot's mode switch is on `STABILIZE` / `LOITER` / `POSHOLD`,
the FCU ignores the tracker's velocity setpoints. The tracker will
repeatedly issue `SET_MODE GUIDED` and the RC switch will repeatedly
override it. This shows up in the log as:

```
[MAVLink] ⚠ Mode flapping detected (N transitions in 10s) — likely RC pilot vs tracker conflict
```

**Correct setup for autonomous following:**

- RC mode switch on `GUIDED`
- RC pilot keeps hands off the sticks
- A different RC position (e.g. `LOITER`) reachable instantly for abort

The mode-switch abort is your fastest recovery — it does not depend on
the Jetson responding.

### 2.4 The `t` key is a display-window shortcut, not an SSH command

From the desktop OpenCV display window, pressing `t` toggles tracking
on/off. From the SSH stdin prompt, `t` returns `[Cmd] unknown: 't'`
because the SSH parser does not alias single-letter keys (a stray `t`
must not silently disable tracking mid-flight).

| Action | Display window | SSH stdin |
|---|---|---|
| Toggle tracking | `t` | `track on` / `track off` |
| Center gimbal | `r` | `center` |
| Quit | `q` | `q` |

The full mapping is in section 6.

---

## 3. Gated Bench → Outdoor → Flight Progression

Walk these gates in order. Each has a pass criterion — if it fails,
stop and fix before moving on. Do not skip gates.

Two prerequisites:

1. **Confirm the safety plan status.** Earlier notes referenced
   `SAFETY_IMPLEMENTATION_PLAN_2026-05-12.md` (20 gaps to close before
   any real-person ground test). That file may not be in this checkout
   — verify the outstanding items via other branches, notes, or commit
   history before treating any of them as closed.
2. **The RC pilot is the primary control.** The Jetson is auxiliary.
   Always have a manual-override mode mapped to an instant-access RC
   switch.

### Gate 0 — Install dependencies (one-time, then on each `git pull`)

```bash
source /home/ai2/ai/bin/activate
cd /home/ai2/person-tracking
git pull origin akash
pip install -r requirements.txt
python3 -c "import pymavlink, cv2, ultralytics; print('deps OK')"
```

**Pass:** prints `deps OK`. Runtime auto-install is intentionally
disabled (see [`utils/preflight.py`](utils/preflight.py)).

### Gate 1 — Camera + gimbal only (no FCU, props not needed)

```bash
python3 main.py --stream-host 0.0.0.0
```

Open `http://<jetson-ip>:5000/`. Stand in view. From the stdin prompt:

```text
ids
track <id from the list>
```

**Pass:** stable video feed for ≥ 2 minutes, gimbal slews to lock onto
the chosen person, no errors in the log. If RTSP fails here, stop —
the flight code depends on this stream.

### Gate 2 — FCU bench test, **propellers OFF**, ground-test mode

Physically remove propellers. Connect the Cube. Then:

```bash
python3 main.py --drone --ground-test --stream-host 0.0.0.0 --cells 4
```

From stdin: `preflight`, then `status`.

**Pass criteria:**

- `[MAVLink] Connected` (not the gimbal-only fallback)
- `[OK]` on MAVLink, heartbeat, IMU/mag/baro, geofence, battery
- `[WAIT]` on GPS / HOME (expected indoors; not a failure)
- `[FAIL]` items on ArduPilot params show the actual value and the
  required value — record these for Mission Planner

Also test the dry-run E-STOP and safe-mode commands while in this gate:

```text
estop          # first press → DRY-RUN BRAKE
estop          # second press within 3s → DRY-RUN LAND
mode brake
mode land
mode rtl
q
```

Each of these will print `[DRY-RUN]` lines and NOT transmit to the FCU.
This is the only mode that exercises the full code path safely.

### Gate 3 — FCU bench test, **propellers STILL OFF**, MAVLink writes enabled

```bash
python3 main.py --drone --stream-host 0.0.0.0 --cells 4
```

From stdin:

```text
preflight
mode auto
follow
unfollow
q
```

**Pass:** the `follow` command transitions through preflight without
errors, `unfollow` is clean, no `[MAVLink] WARNING` lines. (FCU
motors will NOT spin from `follow` alone — that requires the RC pilot
to FCU-arm via sticks; `follow` is only the tracker's software
authority to send velocity setpoints.)

**Do not run `takeoff` in this gate.** Even with propellers off,
`takeoff` commands the FCU to climb and can confuse it on the bench.

### Gate 4 — Outdoor flight pre-checklist

Before any launch command that can fly the drone:

- [ ] Gates 0–3 all passed in this session, on this Jetson, with this build
- [ ] Propellers re-installed, torque-checked, rotation direction verified
- [ ] Open area, no people, no overhead obstacles, RTL altitude tested in previous flights
- [ ] RC transmitter on, bound, mode switch verified; manual-override mode on instant-access position
- [ ] GCS (Mission Planner / QGroundControl) connected on a separate laptop for telemetry monitoring
- [ ] Battery fresh, voltage logged
- [ ] Safety plan items reviewed (see prerequisite 1 above)

Only after every item is checked: proceed to section 4.

---

## 4. Autonomous Following — End-to-End Workflow

Run this only after Gate 4 above. The drone is on the ground with
propellers installed, RC pilot is at the transmitter, operator is at
the SSH/web UI.

### Phase A — Launch and verify preflight

```bash
python3 main.py --drone --stream-host 0.0.0.0 --cells 4
```

Wait ~10 seconds for telemetry to settle. RC pilot sets mode switch to
`GUIDED`. Then from stdin:

```text
preflight
```

**Pass:** `passed=11/11`. If anything is `[FAIL]`, stop and fix. `[WAIT]`
items must be `[OK]` before flight — re-acquire GPS if needed.

### Phase B — Takeoff

Two options. Pick one.

**Option B.1 — Tracker commands takeoff:**

RC pilot arms the FCU via the sticks gesture (motors spin and hold).
Operator from stdin:

```text
takeoff 5
```

Drone climbs to 5 m AGL. Default is 7 m. Bounds are
`MIN_TAKEOFF_ALT_M`–`MAX_TAKEOFF_ALT_M` in
[`config/config.py`](config/config.py).

**Option B.2 — RC pilot takes off manually, then hands off:**

RC pilot takes off in `STABILIZE` or `LOITER`, climbs to a safe
altitude, then flips the mode switch to `GUIDED` and removes hands from
the sticks.

### Phase C — Engage drone-body following

Operator from stdin:

```text
ids                  # list detected people
track 3              # lock GIMBAL onto person ID 3
```

The gimbal now follows the person. **The drone body is still hovering**
— it does not follow yet. Then:

```text
follow               # enable DRONE-BODY following
```

**Pass:** `[Follow] Drone-body following ENABLED — preflight all green`.
The drone now flies after person ID 3 at the configured speed and standoff.

### Phase D — Mid-flight controls

Operator commands while the drone is following:

| Command | Effect |
|---|---|
| `unlock` | Release the person lock; drone holds position, gimbal scans |
| `track <new_id>` | Switch to a different person (no need to `unfollow` first) |
| `unfollow` | Stop drone-body following but keep the tracker running |
| `mode brake` | Stop drone, hold position |
| `mode land` | Land at current XY |
| `mode rtl` | Return to home, then land |
| `estop` | BRAKE; second press within 3 s sends LAND |

RC pilot continuously:
- Watches for unexpected behavior
- Ready to flip the mode switch to `STABILIZE` / `LOITER` for instant manual takeover

### Phase E — Landing

Either:

- RC pilot flips the mode switch to `LOITER` and lands manually, or
- Operator: `mode land` (FCU descends and shuts down at current XY), or
- Operator: `mode rtl` (FCU returns to home, then lands)

Then `q` from stdin to quit the tracker cleanly. Recording auto-saves.

---

## 5. Emergency Stop (E-STOP)

E-STOP is reachable from three places. Each does exactly the same
thing.

| Channel | Action |
|---|---|
| Web UI | Red E-STOP button, or Space key |
| SSH stdin | `estop` |
| HTTP | `curl "http://<jetson-ip>:5000/estop"` |

A single press performs all of:

1. Disables autonomous person tracking (`tracking_enabled = False`)
2. Stops drone-body following; emits structured `unfollow` flight-log event with `reason="estop"`
3. Resets the drone controller state (EMA filter, jerk limiter, retreat latch, etc.)
4. Stops the gimbal — including any manual pan/tilt in progress
5. Sends `BRAKE` to the FCU. A second press within 3 seconds escalates to `LAND`.

Each channel has an independent 3-second window — SSH can always
escalate `BRAKE → LAND` without depending on the browser.

`/estop` is exempt from `--stream-token` enforcement, so it remains
reachable in an emergency regardless of token state.

**Under `--ground-test`:** E-STOP is tested in software but MAVLink is
not transmitted. For real emergency recovery during bench-with-FCU
tests, the RC pilot must switch out of `GUIDED` or use the RC emergency
procedure.

---

## 6. SSH Command Reference

The running tracker reads commands from stdin. Type `help` for the live
list — it is regenerated from the same `_HELP` table the program uses,
so the two cannot drift.

```text
help                   show this command list
status [json]          telemetry snapshot (json = raw dict)
preflight              run the 11-item readiness checklist
ids                    list detected person IDs
track <id>             lock gimbal onto numeric person ID
track on|off           enable/disable autonomous tracking flag
unlock                 release current person lock
mode                   print current tracker mode
mode auto|manual       set tracker mode (AUTO=follow, MANUAL=stick gimbal)
mode brake|land|rtl    request FCU safety mode (--drone required)
follow                 enable drone-body following (--drone, preflight all-green required)
unfollow               stop drone-body following (tracker only; FCU mode unchanged)
takeoff [alt]          command FCU takeoff to alt m AGL (default 7; --drone, FCU armed + landed)
estop                  BRAKE; press again within 3 s for LAND
pan <-100..100>        manual gimbal pan speed (MANUAL mode)
tilt <-100..100>       manual gimbal tilt speed (MANUAL mode)
stop                   stop manual gimbal motion
zoom in|out            0.5 s zoom pulse (also disables auto-zoom)
rec [on|off]           toggle/start/stop recording
stream on|off          start/stop live MJPEG stream server
search on|off|restart  toggle search or restart initial acquisition scan
center                 center gimbal and reset zoom to 1×
autozoom on|off        toggle auto-zoom
q                      quit program
```

Notes:

- Unknown commands print `[Cmd] unknown: ... — type 'help'` and log
  `ssh_unknown_command` to the structured flight log.
- `follow`, `unfollow`, `takeoff`, and `estop` print `[Cmd] ... unavailable`
  when the tracker was launched without `--drone`.
- The pre-rename words `arm` and `disarm` are deliberately NOT aliased
  to `follow`/`unfollow`. Typing them returns `[Cmd] unknown` with a
  rename hint, so a stray keypress during flight cannot accidentally
  toggle drone-body following.
- `takeoff` refuses unless preflight is all-green, the FCU is armed
  (RC pilot's job), and the FCU reports `LANDED`. Auto-switches to
  GUIDED if needed.
- `unfollow` only stops the tracker's drone-body control. To safe-mode
  the FCU itself, use `mode brake` / `mode land` / `mode rtl`.

### Web UI ↔ SSH command parity

Every web UI button has an SSH equivalent (and several SSH commands
have no web counterpart).

| Web UI action | SSH equivalent |
|---|---|
| Click person in video | `ids` then `track <id>` |
| UNLOCK button | `unlock` |
| FOLLOW ▾ → Start Following | `follow` |
| FOLLOW ▾ → Stop Following | `unfollow` |
| Preflight panel | `preflight` |
| Mode AUTO/MANUAL toggle | `mode auto` / `mode manual` |
| BRAKE / LAND / RTL buttons | `mode brake` / `mode land` / `mode rtl` |
| E-STOP button | `estop` |
| Zoom I / O | `zoom in` / `zoom out` |
| Telemetry strip | `status` |
| D-pad ↑/↓/←/→ | `mode manual` then `pan <±>` / `tilt <±>` |
| D-pad ■ stop | `stop` |
| _(no web equivalent)_ | `help`, `status json`, `center`, `rec`, `stream on/off`, `search on/off/restart`, `track on/off`, `autozoom on/off`, `q` |

---

## 7. Display-Window Keyboard Shortcuts

These shortcuts work only when the OpenCV display window is in focus
(i.e. a desktop session with an X11 display). **They do not work via
SSH** — for SSH equivalents see section 6.

```text
m           Toggle AUTO / MANUAL
arrow keys  Move gimbal in MANUAL mode
t           Tracking on/off
r           Center gimbal + reset zoom
s           Toggle search
i           Restart initial acquisition scan
z           Zoom in
x           Zoom out
a           Auto-zoom on/off
v           Recording on/off
l           Live stream on/off
q           Quit
```

---

## 8. HTTP Endpoint Reference

Suitable for `curl`, mobile shortcuts, or third-party integrations.

```text
/                         Web operator page
/status                   Live status JSON
/preflight                Preflight checklist JSON
/click?x=<0-1>&y=<0-1>    Lock person at normalized video coordinate
/unlock                   Release lock
/mode?set=auto|manual     Set tracker mode
/mode?set=brake|land|rtl  Send FCU safety mode (stops tracking first)
/gimbal?dir=up|down|left|right|stop   Manual gimbal direction
/zoom_in / /zoom_out      Zoom pulse
/follow?on=true           Start drone-body following
/follow?on=false          Stop drone-body following
/estop                    E-STOP (first BRAKE, second LAND within 3 s)
```

`/estop` is exempt from `--stream-token` enforcement, so it remains
reachable in an emergency regardless of token state.

---

## 9. Troubleshooting

### `follow` refused

The error lists every failing preflight item with the FCU's actual
value vs the required value. Common causes:

| Symptom | Fix |
|---|---|
| `Vehicle ARMED — FCU reports disarmed` | RC pilot arms the FCU with the sticks gesture |
| `GPS fix OK — fix<3` | Go outdoors with clear sky view; wait for satellites |
| `HOME position set — FCU has not advertised HOME yet` | Same — HOME is set when GPS lock is acquired |
| `RC transmitter connected — never received` | ArduPilot param `SR1_RC_CHAN=5` (or `SR2_RC_CHAN=5` depending on TELEM port) |
| `ArduPilot params correct — FENCE_ENABLE: ...; ...` | Set each named param to its required value in Mission Planner, write, reboot the FCU |

### `follow` accepted but drone doesn't move

- **Mode flapping in log** → RC mode switch is not on `GUIDED`
- **FCU not armed** → RC pilot must FCU-arm via sticks (the tracker's `follow` is software authority only, separate from FCU-arming)
- **No `track <id>` issued** → drone hovers, waiting for a person lock
- **Person locked but `tracking_enabled` is off** → run `track on`
- **Tracker in MANUAL mode** → run `mode auto`

### "Mode flapping detected"

The autopilot is reporting alternating flight modes. Causes, ordered
by likelihood:

1. RC mode switch is fighting `GUIDED` (operator and tracker pulling
   opposite directions). Move the switch to `GUIDED` and stop holding
   the sticks.
2. GCS heartbeat failsafe mis-configured (`FS_GCS_ENABLE`).
3. Two GCS clients connected to the same FCU at once. Disconnect one.

The flap detector prints one summary line when the condition starts
and one when it ends.

### Web UI unreachable

On the Jetson:

```bash
ss -tlnp | grep 5000               # is the tracker listening?
tailscale ip -4                    # which Tailscale IP do you have?
sudo iptables -L -n | grep 5000    # local firewall blocking 5000?
tailscale serve status             # Tailscale Serve proxy configured?
```

Expected:

- `ss` shows `0.0.0.0:5000`. If `127.0.0.1:5000`, the tracker is
  loopback-only — relaunch with `--stream-host 0.0.0.0` to restore
  LAN/Tailscale-IP access. The Tailscale Serve URL still works either
  way.
- `tailscale ip -4` matches the IP you typed in the browser.
- `iptables` is empty or has an explicit `ACCEPT` for 5000.

While the web UI is unreachable, SSH stdin gives full control — see
the parity table in section 6.

### GStreamer fallback warning

```
[GRAB] WARN: GStreamer unavailable — using FFmpeg (higher latency)
```

Means video uses the slower FFmpeg path and recording falls back to
software `mp4v` instead of the Orin's NVENC. Fix:

```bash
sudo apt install -y gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                    gstreamer1.0-libav python3-gst-1.0
python3 -c "import gi; gi.require_version('Gst','1.0'); from gi.repository import Gst; print(Gst.version())"
```

The second line should print a version tuple. If it errors, the venv
was created without `--system-site-packages`.

### TensorRT engine portability warning

```
[TRT] [W] Using an engine plan file across different models of devices is not recommended
```

The `yolo26s.engine` file was built on a different Jetson SKU or
different TensorRT version. Inference still works but may be
sub-optimal. Rebuild on the target Orin:

```bash
yolo export model=models/yolo26s.pt format=engine device=0 half=True
```
