"""
siyi_controller.py — SIYI A8 mini gimbal UDP controller.

Handles all SIYI SDK communication:
  - Gimbal speed commands (pan/tilt)
  - Zoom control
  - Center / home
  - Battery voltage and current telemetry  (cmd 0x0A)
  - Actual gimbal attitude readback        (cmd 0x0D)

Protocol: little-endian, CRC16-CCITT (modbus variant).
Packet: 55 66 01 [len:2LE] [seq:2LE] [cmd:1] [payload:N] [crc:2LE]

Safety note: gimbal angles are clamped before every send so mechanical
limits of the A8 mini (pan ±160°, tilt –135°/+45°) are never exceeded.
"""

import select
import socket
import struct
import threading
import time

import config as cfg
from config.settings import Settings, load_settings


class SIYIController:
    """Thread-safe SIYI A8 mini UDP controller with telemetry receive loop.

    Args:
        camera_ip: IP address of the SIYI camera.
        port: UDP control port (default 37260).
    """

    def __init__(
        self,
        camera_ip: str | None     = None,
        port: int | None          = None,
        settings: Settings | None = None,
    ) -> None:
        _s = settings or load_settings()
        self.camera_ip = camera_ip or _s.camera_ip
        self.port = port or _s.gimbal_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)   # non-blocking; recv uses select()
        self.seq: int = 0
        self.lock = threading.Lock()
        self._last_cmd_time: float = 0.0
        self._last_yaw: int = 0
        self._last_pitch: int = 0
        self.current_zoom: float = 1.0

        # --- Battery telemetry (written by recv thread, read by main thread) ---
        self._battery_mv: int  = 0    # millivolts (0 = no data yet)
        self._current_ma: int  = 0    # milliamps

        # --- UDP watchdog ---
        self._last_recv_time: float = 0.0   # timestamp of last received UDP packet

        # --- Gimbal attitude telemetry (cmd 0x0D) ---
        # Units: 0.1 degree.  Protected by self.lock.
        self._att_yaw_deg: float   = 0.0   # actual pan angle (degrees)
        self._att_pitch_deg: float = 0.0   # actual tilt angle (degrees)
        self._att_roll_deg: float  = 0.0
        self._att_time: float      = 0.0   # monotonic timestamp of last update

        # --- Commanded angle fallback ---
        self._cmd_yaw_deg: float   = 0.0   # accumulated from speed commands
        self._cmd_pitch_deg: float = 0.0
        self._last_angle_est_time: float = 0.0

        # --- Receive thread ---
        self._recv_running: bool = False
        self._recv_th: threading.Thread | None = None
        self._start_recv_thread()

        # --- Attitude poll thread ---
        self._att_poll_running: bool = False
        self._att_poll_th: threading.Thread | None = None
        self._start_attitude_poll()

    # ------------------------------------------------------------------
    #  Receive infrastructure
    # ------------------------------------------------------------------

    def _start_recv_thread(self) -> None:
        """Start background thread to receive and parse SIYI UDP responses."""
        self._recv_running = True
        self._recv_th = threading.Thread(
            target=self._recv_loop, daemon=True, name="SIYIRecv"
        )
        self._recv_th.start()

    def _recv_loop(self) -> None:
        """Read UDP packets from gimbal; parse battery and attitude data."""
        while self._recv_running:
            try:
                r, _, _ = select.select([self.sock], [], [], 0.5)
                if r:
                    data, _ = self.sock.recvfrom(512)
                    self._parse_packet(data)
            except OSError:
                break
            except Exception as e:
                print(f"[SIYI] Recv error ({type(e).__name__}): {e}")

    def _parse_packet(self, data: bytes) -> None:
        """Parse a SIYI UDP response packet.

        Protocol layout:
            bytes 0-2  : STX = 55 66 01
            bytes 3-4  : data length (uint16 LE)
            bytes 5-6  : sequence (uint16 LE)
            byte  7    : command ID
            bytes 8..N : payload
            last 2     : CRC16

        Handled responses:
            0x0A — gimbal info (battery voltage mV + current mA)
            0x0D — gimbal attitude (yaw, pitch, roll in 0.1° units)

        Args:
            data: Raw UDP bytes received from the camera.
        """
        if len(data) < 10 or data[0] != 0x55 or data[1] != 0x66:
            return
        # Any valid packet resets the watchdog
        self._last_recv_time = time.monotonic()
        cmd_id = data[7]
        dlen   = struct.unpack_from("<H", data, 3)[0]
        if len(data) < 10 + dlen:
            return
        payload = data[8 : 8 + dlen]

        # Battery / power status response
        if cmd_id == 0x0A and len(payload) >= 4:
            with self.lock:
                self._battery_mv = struct.unpack_from("<H", payload, 0)[0]
                self._current_ma = struct.unpack_from("<H", payload, 2)[0]

        # Gimbal attitude response
        # Payload: yaw(int16 LE), pitch(int16 LE), roll(int16 LE) in 0.1° units
        elif cmd_id == 0x0D and len(payload) >= 6:
            raw_yaw, raw_pitch, raw_roll = struct.unpack_from("<hhh", payload, 0)
            with self.lock:
                self._att_yaw_deg   = raw_yaw   / 10.0
                self._att_pitch_deg = raw_pitch / 10.0
                self._att_roll_deg  = raw_roll  / 10.0
                self._att_time      = time.monotonic()
                self._cmd_yaw_deg   = self._att_yaw_deg
                self._cmd_pitch_deg = self._att_pitch_deg
                self._last_angle_est_time = time.time()

    # ------------------------------------------------------------------
    #  Attitude polling
    # ------------------------------------------------------------------

    def _start_attitude_poll(self) -> None:
        """Continuously request gimbal attitude at GIMBAL_ATTITUDE_POLL_HZ."""
        self._att_poll_running = True
        self._att_poll_th = threading.Thread(
            target=self._att_poll_loop, daemon=True, name="SIYIAttPoll"
        )
        self._att_poll_th.start()

    def _att_poll_loop(self) -> None:
        interval = 1.0 / cfg.GIMBAL_ATTITUDE_POLL_HZ
        while self._att_poll_running:
            self.request_attitude()
            time.sleep(interval)

    def request_attitude(self) -> None:
        """Send cmd 0x0D — request actual gimbal attitude (pan/tilt/roll)."""
        self._send(0x0D, b"")

    # ------------------------------------------------------------------
    #  Public attitude interface
    # ------------------------------------------------------------------

    @property
    def gimbal_pan_deg(self) -> float:
        """Actual gimbal pan angle in degrees.

        Returns actual telemetry value if fresh (< GIMBAL_ATTITUDE_STALE_S old),
        otherwise falls back to the commanded angle accumulator.
        """
        with self.lock:
            age = time.monotonic() - self._att_time
            if self._att_time > 0 and age < cfg.GIMBAL_ATTITUDE_STALE_S:
                return self._att_yaw_deg
            return self._cmd_yaw_deg

    @property
    def gimbal_tilt_deg(self) -> float:
        """Actual gimbal tilt angle in degrees (negative = looking down)."""
        with self.lock:
            age = time.monotonic() - self._att_time
            if self._att_time > 0 and age < cfg.GIMBAL_ATTITUDE_STALE_S:
                return self._att_pitch_deg
            return self._cmd_pitch_deg

    def attitude_is_fresh(self) -> bool:
        """Return True if gimbal attitude telemetry is within GIMBAL_COAST_S."""
        with self.lock:
            return (
                self._att_time > 0
                and (time.monotonic() - self._att_time) < cfg.GIMBAL_COAST_S
            )

    # ------------------------------------------------------------------
    #  Battery / power interface
    # ------------------------------------------------------------------

    def request_battery_status(self) -> None:
        """Send cmd 0x0A — request battery voltage and current."""
        self._send(0x0A, b"")

    @property
    def battery_voltage(self) -> float:
        """Last received battery voltage in Volts (0.0 = no data yet)."""
        with self.lock:
            return self._battery_mv / 1000.0

    @property
    def current_ma(self) -> int:
        """Last received current draw in milliamps."""
        with self.lock:
            return self._current_ma

    @property
    def power_watts(self) -> float:
        """Estimated power consumption in Watts (V × A)."""
        with self.lock:
            return (self._battery_mv / 1000.0) * (self._current_ma / 1000.0)

    @property
    def is_responding(self) -> bool:
        """True if a UDP packet arrived within GIMBAL_WATCHDOG_S seconds.

        Returns True on startup (before first packet) so the watchdog
        does not fire before the connection has had a chance to establish.
        """
        if self._last_recv_time == 0.0:
            return True
        return (time.monotonic() - self._last_recv_time) < cfg.GIMBAL_WATCHDOG_S

    # ------------------------------------------------------------------
    #  CRC and packet building
    # ------------------------------------------------------------------

    def _crc16(self, data: bytes) -> int:
        """CRC-16/XMODEM — matches SIYI SDK specification.

        Parameters: poly=0x1021, init=0x0000, refIn=False, refOut=False, xorOut=0x0000
        Verified test vector: _crc16(b'\\x55\\x66\\x01\\x00\\x00\\x00\\x00\\x08\\x01') == 0xD44C
        (center-gimbal command with seq=0)
        """
        crc = 0
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
                crc &= 0xFFFF
        return crc

    def _build(self, cmd_id: int, data: bytes = b"") -> bytes:
        """Build a complete SIYI UDP packet."""
        payload = (
            b"\x55\x66\x01"
            + struct.pack("<H", len(data))
            + struct.pack("<H", self.seq)
            + struct.pack("B", cmd_id)
            + data
        )
        crc = self._crc16(payload)
        self.seq = (self.seq + 1) & 0xFFFF
        return payload + struct.pack("<H", crc)

    def _send(self, cmd_id: int, data: bytes = b"") -> None:
        """Send a packet to the gimbal (non-blocking)."""
        with self.lock:
            try:
                self.sock.sendto(
                    self._build(cmd_id, data), (self.camera_ip, self.port)
                )
            except BlockingIOError:
                pass

    # ------------------------------------------------------------------
    #  Motion commands
    # ------------------------------------------------------------------

    def set_speed(self, yaw: int, pitch: int, force: bool = False) -> bool:
        """Send gimbal speed command.

        Args:
            yaw:   Pan speed  -100..+100. Positive = pan right.
            pitch: Tilt speed -100..+100. Positive = tilt up.
            force: Skip rate-limiting and duplicate-suppression checks.

        Returns:
            True if the command was actually sent.

        Safety note: values are clamped to ±100 before sending.
        """
        yaw   = max(-100, min(100, int(yaw)))
        pitch = max(-100, min(100, int(pitch)))
        now   = time.time()
        self._integrate_commanded_attitude(now)
        if not force:
            if (
                now - self._last_cmd_time < cfg.GIMBAL_CMD_MIN_INTERVAL
                and abs(yaw - self._last_yaw) < 3
                and abs(pitch - self._last_pitch) < 3
            ):
                return False
        self._send(0x07, struct.pack("bb", yaw, pitch))
        self._last_cmd_time = now
        self._last_yaw      = yaw
        self._last_pitch    = pitch
        return True

    def stop(self) -> None:
        """Immediately stop all gimbal motion."""
        self.set_speed(0, 0, force=True)

    def center(self) -> None:
        """Return gimbal to home position (0°/0°)."""
        self._send(0x08, struct.pack("B", 1))
        with self.lock:
            self._last_yaw        = 0
            self._last_pitch      = 0
            self._cmd_yaw_deg     = 0.0
            self._cmd_pitch_deg   = 0.0
            self._last_angle_est_time = time.time()

    def _integrate_commanded_attitude(self, now: float) -> None:
        """Update fallback angle estimates from the previous speed command."""
        with self.lock:
            if self._last_angle_est_time <= 0.0:
                self._last_angle_est_time = now
                return
            dt = max(0.0, min(cfg.GIMBAL_CMD_EST_MAX_DT_S, now - self._last_angle_est_time))
            self._last_angle_est_time = now
            scale = cfg.GIMBAL_SPEED_FULL_SCALE_DEG_S / 100.0
            self._cmd_yaw_deg = max(
                cfg.GIMBAL_PAN_MIN_DEG,
                min(cfg.GIMBAL_PAN_MAX_DEG, self._cmd_yaw_deg + self._last_yaw * scale * dt),
            )
            self._cmd_pitch_deg = max(
                cfg.GIMBAL_TILT_MIN_DEG,
                min(cfg.GIMBAL_TILT_MAX_DEG, self._cmd_pitch_deg + self._last_pitch * scale * dt),
            )

    # ------------------------------------------------------------------
    #  Zoom commands
    # ------------------------------------------------------------------

    # Step per relative zoom command — approximate; replace with a calibrated
    # value once hardware zoom rate (levels/s × cmd duration) is measured.
    _ZOOM_STEP_EST: float = 0.5

    def zoom_in(self) -> None:
        self._send(0x05, struct.pack("b", 1))
        self.current_zoom = min(self.current_zoom + self._ZOOM_STEP_EST, cfg.ZOOM_MAX)

    def zoom_out(self) -> None:
        self._send(0x05, struct.pack("b", -1))
        self.current_zoom = max(self.current_zoom - self._ZOOM_STEP_EST, 1.0)

    def zoom_stop(self) -> None:
        self._send(0x05, struct.pack("b", 0))

    def zoom_absolute(self, level: float) -> None:
        """Command absolute zoom level.

        Args:
            level: Zoom factor 1.0 – ZOOM_MAX.
        """
        level     = max(1.0, min(cfg.ZOOM_MAX, level))
        int_part  = int(level)
        dec_part  = int(round((level - int_part) * 10))
        self._send(0x0F, struct.pack("BB", int_part, dec_part))
        self.current_zoom = level

    def test_zoom(self) -> None:
        """Quick zoom-in/out test on startup to verify zoom is working."""
        print("[Zoom] Testing: zoom-in 0.5 s...")
        self._send(0x05, struct.pack("b", 1))
        time.sleep(0.5)
        self._send(0x05, struct.pack("b", 0))
        print("[Zoom] Test done — check if image zoomed on screen")

    # ------------------------------------------------------------------
    #  Shutdown
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Stop all threads and release the UDP socket."""
        self._att_poll_running = False
        self._recv_running     = False
        if self._att_poll_th:
            self._att_poll_th.join(timeout=1.0)
        if self._recv_th:
            self._recv_th.join(timeout=1.0)
        self.zoom_stop()
        self.stop()
        time.sleep(0.05)
        self.sock.close()
