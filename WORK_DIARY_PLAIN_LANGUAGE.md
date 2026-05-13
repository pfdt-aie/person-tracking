# 3 Days of Safety Work — Plain-Language Summary

Three days of safety work on the person-following drone software,
explained in three bullets per day. No code knowledge needed.

---

## Day 1 — Safety brakes and a way to rehearse without flying

- **Big red E-STOP button on the screen.** A new button (and the Space
  bar) on the web page stops the drone instantly. One press = brake in
  mid-air; second press within 3 seconds = land. Before this, the only
  way to stop the drone was to grab the radio controller.

- **Rehearsal mode that doesn't actually fly the drone.** A new
  `--ground-test` option runs the whole system — camera, AI, decision
  logic — but never sends a movement command to the drone. We can put a
  real person in front of the camera and watch what the system *would*
  do, with propellers off.

- **Three pre-takeoff guards that make sure nothing moves until it's
  safe.** (1) A pre-flight checklist on the web page blocks arming
  until nine items are green. (2) The drone backs away on its own at
  1 m/s if a person gets closer than 4 metres. (3) The drone needs five
  steady frames of detection before it will move its body toward a
  target — single false alarms (bushes, dogs, posters) no longer yank
  it. We also tightened the radio-link watchdog from 3 s to 2 s.

---

## Day 2 — Better awareness before and during takeoff

- **Stricter GPS and flight-controller verification.** GPS now has to
  pass four checks (real fix, accuracy ≤ 1.5, ≥ 10 satellites, internal
  confidence). Before arming, the system also reads seven safety
  settings directly from the flight controller (geofence on, fence
  radius, return-home altitude, low-battery action, etc.) and refuses
  to arm if any are wrong.

- **The operator and the launch spot are protected.** A 5-metre no-fly
  bubble around the takeoff point means the drone will never fly over
  the operator who is standing there. Separately, if the AI ever
  "teleports" the person's position by more than 10 metres (a common
  symptom of mistaking one person for another), the bad reading is
  thrown away instead of acted on.

- **Smoother, less twitchy following.** A hard acceleration cap of
  2 m/s² stops the drone from lurching after a brief pause. The
  decision of "which way is the person going" now requires 1 full
  second of sustained motion before changing — and even then the
  turn happens at a controlled rate.

---

## Day 3 — Visibility, recordings, and runtime safety

- **Live status strip + black-box recorder.** A one-line status bar at
  the top of the web page now updates every second with flight mode,
  GPS quality, battery, frame rate, and warnings (turns red on fence
  breach or pilot override). Separately, every safety-relevant event —
  E-STOP, mode change, arm/disarm, return-to-home, slowdowns,
  override — is written to a black-box log file (`logs/flight_*.jsonl`)
  so a flight can be reviewed afterwards.

- **Auto return-home and operator battery override.** A 10-minute
  session timer triggers automatic return-to-home if the drone has been
  actively following that long — protects against a forgotten operator.
  A new `--cells` startup option lets the operator tell the system the
  exact battery cell count instead of guessing, so the low-battery
  trigger fires at the right voltage even on a partially charged pack.

- **The drone holds when it can't trust itself.** If the camera AI's
  effective frame rate drops below 8 frames per second, the drone body
  pauses (camera view keeps tracking) so the drone never acts on stale
  frames. Separately, if the safety pilot takes manual control via the
  radio, auto-follow locks itself out and cannot resume until the
  operator explicitly walks the pre-flight checklist again and presses
  Arm.

---

## Bottom line

- **Before:** start the program → drone immediately controllable. No
  safe way to rehearse. Stopping meant grabbing the radio. Hard to tell
  what the drone "knew" at any moment.

- **After:** rehearsal mode + 9-item pre-flight checklist + 7 flight-
  controller settings verified + big red E-STOP + always-visible status
  strip + full black-box recording of every flight.

- **One number:** **170 automated tests** now run on every change (up
  from 51 on Day 0) — 119 new tests guard this safety behaviour against
  future regressions.
