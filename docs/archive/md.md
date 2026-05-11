> Historical note: this safety analysis is superseded by
> `DRONE_PROJECT_TECHNICAL_AUDIT_REPORT_2026-05-08.md`. Several gaps listed
> below were fixed after this note was written.

I have full context from the audit and the files. Here is the complete safety analysis:

MAVLink Safety Audit — Orange Cube+ / ArduPilot
Currently Implemented (5 rules in safety.py + supporting checks)
#	Rule	Where enforced
1	Altitude floor/ceiling — clamps every commanded alt to 10–80 m AGL	SafetyMonitor.check_altitude()
2	Speed cap — horizontal ≤ 5 m/s, vertical ≤ 2.5 m/s	SafetyMonitor.check_velocity(), called inside every send_*
3	Circular geofence — 500 m radius haversine check	SafetyMonitor.check_geofence()
4	Heartbeat watchdog — stops commands if heartbeat stale > 3 s	SafetyMonitor.watchdog_heartbeat()
5	Battery critical RTL — per-cell < 3.5 V triggers RTL, one-shot	SafetyMonitor.is_battery_critical()
6	GPS fix gate — minimum 3D fix required	MAVLinkClient.is_gps_ok()
7	Tracking-loss failsafe ladder — 0–2 s EKF coast → 2–5 s zero vel → 5 s+ LOITER → 15 s+ alert	DroneController._handle_tracking_loss()
8	ArduPilot GUID_TIMEOUT — FC stops if no command in 5 s (FC-side backstop)	Config note in mavlink_client.py
9	EKF dead-reckoning — person position coasts during brief occlusion	PersonEKF.predict() (now fixed)
Gaps — What Is Missing
🔴 HIGH PRIORITY
G1. No GUIDED mode verification before velocity commands

drone_controller.update() sends SET_POSITION_TARGET_LOCAL_NED without ever checking that the flight controller is currently in GUIDED mode. If the operator manually switches to LOITER, STABILIZE, or AUTO on the RC transmitter, the code keeps sending velocity targets — ArduPilot silently ignores them but the system believes it is controlling the drone.


# mavlink_client.py — _rx_loop already caches self._mode from HEARTBEAT
# Need: DroneController.update() must check mav._mode == "GUIDED"
G2. No ARM state check before sending commands

is_armed() exists in MAVLinkClient but is never called. If the drone is disarmed (on the ground), velocity commands are harmless because ArduPilot ignores them. However the tracking system thinks it is following a flying drone and could enter incorrect state — and when the drone arms later, it could receive stale commands immediately.

G3. Critical commands sent without COMMAND_ACK

send_rtl() and send_loiter() use command_long_send with no acknowledgment check. These are safety-critical commands. If the serial buffer is full or ArduPilot rejects the command (e.g., wrong mode, pre-arm fail), the failure is completely silent. ArduPilot responds with COMMAND_ACK for all MAV_CMD_* commands — this is never read.

G4. ArduPilot fence breach messages not handled

The _rx_loop parses HEARTBEAT, ATTITUDE, LOCAL_POSITION_NED, GLOBAL_POSITION_INT, GPS_RAW_INT, SYS_STATUS, HOME_POSITION — but not FENCE_STATUS. If ArduPilot's onboard geofence triggers (FENCE_ENABLE=1, FENCE_ALT_MIN=10), the fence breach event is never seen. This is a second independent fence layer that should be logged and trigger the same zero-velocity response.

G5. NaN / Inf not guarded before MAVLink send

The EKF, jerk limiter, and projection math can produce NaN or Inf under degenerate conditions (e.g., dt = 0 in PersonEKF.predict, division by near-zero in CameraGeolocation.project). send_position_velocity_ned passes floats directly into a struct pack — struct.pack will silently pack NaN as a bit pattern and ArduPilot will receive invalid targets.

🟠 MEDIUM PRIORITY
G6. Geofence breach only stops — does not return to safe zone

When check_geofence() returns False, drone_controller.update() calls send_zero_velocity() and returns. The drone decelerates and hovers outside or at the fence boundary. It should command a velocity vector pointing back toward the home centre so the drone returns inside the boundary autonomously.

G7. Geofence center uninitialized until HOME_POSITION received

Until HOME_POSITION arrives (which requires ArduPilot to have a valid home set), _fence.centre_lat == 0.0 and _fence.centre_lon == 0.0, so check_geofence() skips the horizontal radius check entirely (line 190 in safety.py). During the first few seconds of flight, or if HOME_POSITION is never received, the drone can fly anywhere horizontally with no geofence active.

G8. EKF origin differs from geofence home

DroneController._origin_lat/lon is set from the first valid GPS reading after MAVLink connect. SafetyMonitor._fence.centre_lat/lon is set from the HOME_POSITION MAVLink message. These are two independent sources that can disagree. All NED math uses the drone-controller origin; safety checks use the fence centre. A position that appears inside the geofence from one reference frame may be outside from the other.

G9. Velocity spike on re-acquisition after LOITER

When tracking is re-acquired after LOITER, DroneController.reset() clears the EMA filter (_ema_vn = 0). However _prev_vn and _prev_an are also reset to 0. If the drone has been drifting in LOITER and the EKF shows a large position error, the first _compute_follow_velocity() call computes a large raw velocity, which after one EMA step may still produce a jerk spike before the limiter catches it. The EMA alpha of 0.25 means the first commanded velocity = 0.25 × raw, which could still be substantial.

🟡 LOWER PRIORITY
G10. No RC takeover detection

ArduPilot sends mode changes in HEARTBEAT when an operator switches mode via RC. The _mode field is cached but never used to suppress commands or log a "operator overrode tracking" event. At minimum this should be logged; ideally tracking should pause when not in GUIDED.

G11. RTL issued once but never confirmed or retried

_rtl_issued = True is set after sending RTL for battery critical. If the command is lost (serial noise, buffer full), RTL never happens and _rtl_issued prevents a retry forever. Critical failsafe commands need confirmation + retry logic.

G12. No pre-flight readiness gate

No check that ArduPilot's pre-arm checks have passed (SYS_STATUS.onboard_control_sensors_health) before enabling drone following. It is possible to start the tracking loop and issue GUIDED-mode velocity commands while the flight controller is still initialising its EKF, compass, or barometer.

Recommended Implementation Plan
The following summarises what to add, in priority order:

#	Where	What to add
G1	drone_controller.update()	if self._mav._mode != "GUIDED": return 0.0 after heartbeat check
G2	drone_controller.update()	if not self._mav.is_armed(): return 0.0
G3	mavlink_client.py	_await_ack(cmd_id, timeout=1.0) helper; call it in send_loiter() and send_rtl() with retry
G4	mavlink_client._rx_loop	Add FENCE_STATUS message handling; call safety.report_fence_breach()
G5	mavlink_client._send_position_target	if not all(math.isfinite(v) for v in [pN,pE,pD,vN,vE,vD]): return
G6	drone_controller._compute_follow_velocity	When fence breached, compute inward velocity toward fence centre instead of zero
G7	drone_controller.update()	Gate commands: if not self._safety._fence.centre_lat: return 0.0
G8	drone_controller + mavlink_client	Use HOME_POSITION lat/lon as EKF NED origin (same source as geofence)
G9	drone_controller.reset()	Seed _prev_vn/_prev_ve from current MAVLink velocity, not zero
G10	mavlink_client._rx_loop	Log mode change; set _mode_changed flag; suppress commands when not GUIDED
G11	drone_controller._handle_tracking_loss	Retry RTL up to 3× on battery critical with 500 ms delay
G12	mavlink_client.connect()	Wait for SYS_STATUS.onboard_control_sensors_health healthy flag before returning True

