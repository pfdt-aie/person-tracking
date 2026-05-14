# Ground Test Command Guide

This guide is for ground testing the person-tracking code before real flight.
If `python main.py` works on your Jetson, you can use `python`. If not, use
`python3` in the same commands.

## 1. Safe Launch Options

### Camera and gimbal only

Use this when you want person detection, gimbal tracking, and the web UI, but
no drone-body MAVLink control.

```bash
python3 main.py --stream-host 0.0.0.0 --stream-token MYSECRET
```

Open the web UI from your laptop/phone:

```text
http://<jetson-ip>:8080/?token=MYSECRET
```

### Full pipeline ground test with FCU connected

Use this when the flight controller is connected and you want to test telemetry,
preflight, RC link, GPS, battery, and safety checks without sending MAVLink
commands.

```bash
python3 main.py --drone --ground-test --stream-host 0.0.0.0 --stream-token MYSECRET
```

`--ground-test` means the program reads telemetry but does not transmit drone
movement, mode, RTL, BRAKE, or LAND commands.

## 2. Track a Person

### Web UI method

1. Start one of the launch commands above.
2. Open:

```text
http://<jetson-ip>:8080/?token=MYSECRET
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
curl "http://<jetson-ip>:8080/estop"
```

Second press within 3 seconds for LAND:

```bash
curl "http://<jetson-ip>:8080/estop"
```

Important: if the program was launched with `--ground-test`, E-STOP is tested
in software but MAVLink commands are not transmitted to the flight controller.
For real emergency recovery, the RC pilot must switch out of GUIDED or use the
RC emergency procedure.

## 7. Drone-Body Following

Only use this when you are ready for real drone-body control.

```bash
python3 main.py --drone --stream-host 0.0.0.0 --stream-token MYSECRET
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

```text
ids                 Show detected person IDs
track <id>          Lock onto person ID
unlock              Release person lock
mode auto           Automatic tracking mode
mode manual         Manual gimbal mode
mode brake          Stop drone-body following and send BRAKE
mode land           Stop drone-body following and send LAND
mode rtl            Stop drone-body following and send RTL / return-to-home
pan <speed>         Manual pan speed, -100 to 100
tilt <speed>        Manual tilt speed, -100 to 100
stop                Stop manual gimbal motion
q                   Quit program
```

## 9. Available Keyboard Controls

These work when the display window is active.

```text
m           Toggle AUTO / MANUAL
arrow keys  Move gimbal in MANUAL mode
t           Tracking on/off
r           Center gimbal and reset zoom
s           Toggle search
i           Restart initial scan
z           Zoom in
x           Zoom out
a           Auto-zoom on/off
v           Recording on/off
l           Live stream on/off
q           Quit from terminal command
```

## 10. Available Web / HTTP Endpoints

If `--stream-token MYSECRET` is set, add `?token=MYSECRET` to protected control
endpoints.

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

`/estop` does not require the token, so it remains reachable in an emergency.

## 11. Recommended First Ground-Test Sequence

1. Start dry-run with FCU connected:

```bash
python3 main.py --drone --ground-test --stream-host 0.0.0.0 --stream-token test123
```

2. Open:

```text
http://<jetson-ip>:8080/?token=test123
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
curl "http://<jetson-ip>:8080/estop"
curl "http://<jetson-ip>:8080/estop"
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
