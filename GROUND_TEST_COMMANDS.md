# Ground Test Command Guide

This guide is for ground testing the person-tracking code before real flight.
If `python main.py` works on your Jetson, you can use `python`. If not, use
`python3` in the same commands.

> **TEST MODE — `--stream-token` temporarily disabled (2026-05-14).**
> Control-endpoint token enforcement and the no-token loopback fallback
> in [`gcs/stream_server.py`](gcs/stream_server.py) `_check_token()`
> are bypassed for the current test cycle, at the operator's request.
> All examples below are written as if no token is needed — pass
> `--stream-token` if you like, but it has no effect right now.
>
> To re-enable, search the codebase for `[TEST-MODE 2026-05-14]` (two
> hits in `gcs/stream_server.py`), uncomment those blocks, then put
> `--stream-token <secret>` back in the launches and `?token=<secret>`
> back in the URLs.

## 1. Safe Launch Options

### Camera and gimbal only

Use this when you want person detection, gimbal tracking, and the web UI, but
no drone-body MAVLink control.

```bash
python3 main.py --stream-host 0.0.0.0
```

Open the web UI from your laptop/phone:

```text
http://<jetson-ip>:5000/
# or via Tailscale Serve: https://ai2-desktop.tailca9f5b.ts.net/
```

### Full pipeline ground test with FCU connected

Use this when the flight controller is connected and you want to test telemetry,
preflight, RC link, GPS, battery, and safety checks without sending MAVLink
commands.

```bash
python3 main.py --drone --ground-test --stream-host 0.0.0.0
```

`--ground-test` means the program reads telemetry but does not transmit drone
movement, mode, RTL, BRAKE, or LAND commands.

## 2. Track a Person

### Web UI method

1. Start one of the launch commands above.
2. Open:

```text
http://<jetson-ip>:5000/
# or via Tailscale Serve: https://ai2-desktop.tailca9f5b.ts.net/
```

3. Click on the person in the video.
4. The system locks that person and shows a lock ID.
5. The gimbal tracks the selected person.

### Terminal method

First show available detected person IDs:

```text
ids
```

Then lock one person:

```text
track 2
```

Replace `2` with the ID you want.

## 3. Change the Tracked Person

### Web UI method

1. Click **UNLOCK**.
2. Click the new person in the video.

### Terminal method

```text
unlock
ids
track <new_id>
```

Example:

```text
unlock
ids
track 5
```

## 4. Stop Tracking

Release the current person lock:

```text
unlock
```

Turn tracking on/off from display keyboard:

```text
t
```

Quit the program from terminal:

```text
q
```

## 5. Manual Gimbal Control

Switch to manual mode:

```text
mode manual
```

Pan left/right:

```text
pan -30
pan 30
```

Tilt up/down:

```text
tilt 20
tilt -20
```

Stop gimbal movement:

```text
stop
```

Return to automatic tracking:

```text
mode auto
```

`mode manual` also stops drone-body following and sends zero velocity when
MAVLink drone control is active. Click **Arm Tracker** again before resuming
drone-body following.

## 6. Safety Mode Commands

### What E-STOP does

Whether the operator triggers it from the web UI button, the SSH
`estop` command, or `curl /estop`, a single E-STOP press performs
**all** of the following in order:

1. Disables autonomous person tracking (`tracking_enabled = False`).
2. Disarms the drone-body tracker (`drone_armed = False`) and emits a
   structured `disarm` flight-log event with `reason="estop"`.
3. Resets the drone controller state (EMA filter, jerk limiter,
   bearing latch, retreat latch, etc.).
4. **Stops the gimbal**, including any manual pan/tilt that was in
   progress (`manual_yaw_speed = manual_pitch_speed = 0` plus
   `ctrl.stop()`). Without this, an operator mid-`pan 50` in MANUAL
   mode would keep panning after E-STOP fired.
5. Sends `BRAKE` to the FCU on the first press. A second press within
   3 seconds escalates to `LAND`. Each channel (SSH stdin and the
   browser) keeps its own 3-second window — they do not desync each
   other, so SSH can always escalate without depending on the UI.

`/estop` is the only HTTP endpoint exempt from the `--stream-token`
gate (when enforcement is enabled), so it remains reachable for life
safety regardless of token state.

### Web UI

Use the red **E-STOP** button:

1. First press: sends **BRAKE**.
2. Second press within 3 seconds: sends **LAND**.

You can also press the **Space** key in the web UI.

The web UI also has direct safety buttons:

```text
BRAKE     Stop quickly and hold
LAND      Land at the current position
RTL       Return to launch/home
```

These buttons stop drone-body following before sending the FCU mode command.

### Terminal commands

```text
mode brake     Send BRAKE mode
mode land      Send LAND mode
mode rtl       Send RTL / return-to-home mode
```

These commands stop drone-body following, reset the drone controller, send zero
velocity, and then send the requested FCU mode command.

### HTTP command

First E-STOP press:

```bash
curl "http://<jetson-ip>:5000/estop"
# or via Tailscale Serve:
curl "https://ai2-desktop.tailca9f5b.ts.net/estop"
```

Second press within 3 seconds for LAND:

```bash
curl "http://<jetson-ip>:5000/estop"
# or via Tailscale Serve:
curl "https://ai2-desktop.tailca9f5b.ts.net/estop"
```

Important: if the program was launched with `--ground-test`, E-STOP is tested
in software but MAVLink commands are not transmitted to the flight controller.
For real emergency recovery, the RC pilot must switch out of GUIDED or use the
RC emergency procedure.

## 7. Drone-Body Following

Only use this when you are ready for real drone-body control.

```bash
python3 main.py --drone --stream-host 0.0.0.0
```

Then:

1. RC transmitter ON and bound.
2. Flight controller in `GUIDED`.
3. Open the web UI.
4. Click **ARM**.
5. Wait for all preflight checks to pass.
6. Click **Arm Tracker**.
7. Click the person in the video.
8. Click **Stop Follow** to stop drone-body following.

Do not use real drone-body following during first ground checks unless props are
removed or the airframe is otherwise made safe.

## 8. Available Terminal Commands

The running tracker reads commands from stdin. Type `help` at any time
to print the full list — this section is regenerated from the same
`_HELP` table the program uses, so the two cannot drift.

```text
help                   show this command list
status [json]          current telemetry snapshot (json = raw dict)
preflight              run the preflight checklist
ids                    list detected person IDs
track <id>             lock onto a numeric person ID
track on|off           enable/disable autonomous tracking
unlock                 release current person lock
mode                   print current tracker mode
mode auto|manual       set tracker mode
mode brake|land|rtl    request FCU safety mode (--drone required)
arm                    arm drone-body tracker after preflight (--drone)
disarm                 stop following (tracker only; FCU mode unchanged)
estop                  BRAKE; press again within 3s for LAND
pan <-100..100>        manual gimbal pan speed (MANUAL mode)
tilt <-100..100>       manual gimbal tilt speed (MANUAL mode)
stop                   stop manual gimbal motion
zoom in|out            0.5 s zoom pulse (also disables auto-zoom)
rec [on|off]           toggle/start/stop recording (no arg = toggle)
stream on|off          start/stop live MJPEG stream server
search on|off|restart  toggle search or restart initial acquisition scan
center                 center gimbal and reset zoom to 1x
autozoom on|off        toggle auto-zoom
q                      quit program
```

Notes:

- Unknown commands are answered with `[Cmd] unknown: ... — type 'help'`
  and logged to the structured flight log under `ssh_unknown_command`.
- `arm`, `disarm`, and `estop` print `[Cmd] ... unavailable` when the
  tracker was launched without `--drone`. No MAVLink command is sent.
- `disarm` only stops drone-body following inside the tracker. To
  command the FCU itself, use `mode brake`, `mode land`, or `mode rtl`.
- `estop` double-tap escalation is independent per channel: the SSH
  3-second window is separate from the web UI's. Either channel can
  always escalate BRAKE → LAND on its own.

### Web UI &harr; Terminal parity

Every web button has a stdin equivalent (and several stdin commands
exist that have no UI counterpart).

| Web UI action                | Terminal equivalent           |
|------------------------------|-------------------------------|
| Click person in video        | `ids` then `track <id>`       |
| UNLOCK button                | `unlock`                      |
| ARM &#9662; &rarr; Arm Tracker | `arm`                        |
| ARM &#9662; &rarr; Stop Follow | `disarm`                     |
| Preflight panel              | `preflight`                   |
| Mode toggle AUTO/MANUAL      | `mode auto` &middot; `mode manual` |
| BRAKE button                 | `mode brake`                  |
| LAND button                  | `mode land`                   |
| RTL button                   | `mode rtl`                    |
| E-STOP button                | `estop` (double-tap escalates) |
| Zoom I / O                   | `zoom in` &middot; `zoom out` |
| Telemetry strip              | `status`                      |
| D-pad &#8593;/&#8595;/&#8592;/&#8594; | `mode manual` then `pan <±>` / `tilt <±>` |
| D-pad &#9632; stop           | `stop`                        |
| _(none — terminal-only)_     | `help`, `status json`, `center`, `rec`, `stream on/off`, `search on/off/restart`, `track on/off`, `autozoom on/off`, `q` |

## 9. Available Keyboard Controls

These work when the display window is active.

```text
m           Toggle AUTO / MANUAL
arrow keys  Move gimbal in MANUAL mode
t           Tracking on/off
r           Center gimbal and reset zoom
s           Toggle search
i           Restart initial scan
z           Zoom in  (also disables auto-zoom)
x           Zoom out (also disables auto-zoom)
a           Auto-zoom on/off
v           Recording on/off
l           Live stream on/off
q           Quit from terminal command
```

## 10. Available Web / HTTP Endpoints

> Token enforcement is bypassed during the current test cycle (see the
> banner at the top of this file). All endpoints below are reachable
> without `?token=...`. When the bypass is reverted, append
> `?token=<secret>` to every endpoint except `/estop`.

```text
/                         Web operator page
/status                   Live status JSON
/preflight                Preflight checklist
/click?x=<0-1>&y=<0-1>    Lock person at normalized video coordinate
/unlock                   Release lock
/mode?set=auto            Set AUTO mode
/mode?set=manual          Set MANUAL mode
/mode?set=brake           Stop drone-body following and send BRAKE
/mode?set=land            Stop drone-body following and send LAND
/mode?set=rtl             Stop drone-body following and send RTL
/gimbal?dir=up            Manual gimbal up
/gimbal?dir=down          Manual gimbal down
/gimbal?dir=left          Manual gimbal left
/gimbal?dir=right         Manual gimbal right
/gimbal?dir=stop          Stop manual gimbal
/zoom_in                  Zoom in
/zoom_out                 Zoom out
/arm_tracker?on=true      Arm drone-body following
/arm_tracker?on=false     Stop drone-body following
/estop                    E-STOP; first BRAKE, second LAND
```

`/estop` is also exempt from token enforcement in production, so it
remains reachable in an emergency regardless of test-mode state.

## 11. Recommended First Ground-Test Sequence

1. Start dry-run with FCU connected:

```bash
python3 main.py --drone --ground-test --stream-host 0.0.0.0
```

2. Open:

```text
http://<jetson-ip>:5000/
# or via Tailscale Serve: https://ai2-desktop.tailca9f5b.ts.net/
```

3. Stand in front of the camera.
4. Run:

```text
ids
```

5. Lock a person:

```text
track <id>
```

6. Change person:

```text
unlock
ids
track <new_id>
```

7. Test E-STOP in dry-run:

```bash
curl "http://<jetson-ip>:5000/estop"
curl "http://<jetson-ip>:5000/estop"
# or via Tailscale Serve:
# curl "https://ai2-desktop.tailca9f5b.ts.net/estop"
# curl "https://ai2-desktop.tailca9f5b.ts.net/estop"
```

8. Test direct safety mode commands in dry-run:

```text
mode brake
mode land
mode rtl
```

9. Quit:

```text
q
```

## 12. Troubleshooting

### Web UI not reachable (`http://<tailscale-ip>:5000/` or Tailscale Serve URL shows "site can't be reached")

On the Jetson, run these three commands to localize the failure:

```bash
# (1) Is the tracker actually listening on 5000?
ss -tlnp | grep 5000

# (2) Does Tailscale agree on which IP is yours?
tailscale ip -4

# (3) Is anything firewalling 5000 locally?
sudo iptables -L -n | grep 5000

# (4) Is Tailscale Serve configured to proxy → 127.0.0.1:5000?
tailscale serve status
```

Expected:

- `(1)` shows `0.0.0.0:5000` (or `*:5000`). If it shows `127.0.0.1:5000`
  the tracker is loopback-only — direct-IP access won't work but the
  Tailscale Serve proxy at `https://ai2-desktop.tailca9f5b.ts.net/`
  will still reach it. To restore direct LAN/Tailscale-IP access,
  relaunch with `--stream-host 0.0.0.0`.
  (When token enforcement is re-enabled later, also pass
  `--stream-token <secret>`.)
- `(2)` matches the IP you're typing in the browser URL.
- `(3)` is empty, or has explicit ACCEPT rules for 8080.

While the UI is unreachable, the SSH stdin loop gives you full control
of the tracker — every web button has a terminal equivalent listed in
Section 8 ("Web UI &harr; Terminal parity").
