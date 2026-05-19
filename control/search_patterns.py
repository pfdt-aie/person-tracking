"""
search_patterns.py — Four-stage gimbal search pattern library.

Research-backed escalation chain (v9):
  1. InitialScanSearch   — 3-level yaw raster on power-on (MDPI Drones 2024)
  2. SectorScanSearch    — ±60°→±90° sector scan biased toward last velocity
  3. ExpandingSquareSearch — IAMSAR open-loop expanding square
  4. LissajousSearch     — Sinusoidal fill with irrational T ratio (Sci.Reports 2024)

All classes share the interface:
    start(...)  → activate and initialise
    stop()      → deactivate
    get_command() → (yaw_speed, pitch_speed) to send to gimbal

Commands are SIYI speed units (-100..+100). Caller must send them to
SIYIController.set_speed() every frame.

GROUND-TARGET CONSTRAINT
------------------------
The camera is mounted on the drone and the target (person) is always on
the ground below. All patterns therefore restrict pitch to the downward
hemisphere only — the camera NEVER tilts above horizontal (never looks
at the sky). SIYI speed convention: positive pitch speed = tilt UP,
negative pitch speed = tilt DOWN toward ground.
"""

import math
import time

import config as cfg


# =============================================================================
#  INITIAL ACQUISITION SCAN  (MDPI Drones 2024)
#  3-level raster: sweeps yaw ±70° at shallow/medium/deep pitch depression.
# =============================================================================

class InitialScanSearch:
    """3-level yaw raster for power-on acquisition.

    Sweeps yaw at INIT_SCAN_SPEED at three increasing pitch depression
    levels. Stops the instant the tracker transitions to TRACKING state.
    """

    PHASES: int = 3  # pitch levels

    def __init__(self) -> None:
        self.active: bool        = False
        self._pitch_level: int   = 0     # 0=shallow-down, 1=mid, 2=steep-down
        self._yaw_dir: int       = 1     # +1=right, -1=left
        self._sweep_done: int    = 0
        self._state: str         = "pretilt"  # 'pretilt'|'sweep'|'pitch_dn'|'pitch_up'
        self._phase_t: float     = 0.0

    def start(self) -> None:
        """Begin acquisition raster.

        Starts with a brief downward pre-tilt so level-0 sweeps within the
        ground search zone rather than at the horizon.
        """
        self.active       = True
        self._pitch_level = 0
        self._yaw_dir     = 1
        self._sweep_done  = 0
        self._state       = "pretilt"   # tilt down into ground zone first
        self._phase_t     = time.time()
        print(
            f"[InitScan] Acquisition raster started "
            f"({self.PHASES} pitch levels × {cfg.INIT_SCAN_SWEEPS_PER_LEVEL} sweeps)"
        )

    def stop(self) -> None:
        self.active = False

    def get_command(self) -> tuple[int, int]:
        """Return (yaw_speed, pitch_speed) for current raster state.

        Pitch is only negative (downward), zero, or positive during the
        bounded return from steep-down back to shallow-down.
        """
        if not self.active:
            return 0, 0

        now     = time.time()
        elapsed = now - self._phase_t

        # Pre-tilt: tilt down into the ground search zone before sweeping.
        if self._state == "pretilt":
            if elapsed >= cfg.INIT_SCAN_PRETILT_TIME:
                self._state   = "sweep"
                self._phase_t = now
            return 0, -cfg.INIT_SCAN_PITCH_SPEED  # negative = down

        # Stepping down to the next pitch level (deeper depression).
        if self._state == "pitch_dn":
            if elapsed >= cfg.INIT_SCAN_PITCH_STEP_TIME:
                self._pitch_level += 1
                self._sweep_done   = 0
                self._state        = "sweep"
                self._phase_t      = now
                print(f"[InitScan] Level {self._pitch_level + 1}/{self.PHASES}")
            return 0, -cfg.INIT_SCAN_PITCH_SPEED   # negative = down

        # Return toward level-0 (shallow-down) — stop after PITCH_RETURN_TIME
        # which is calibrated to equal the total downward travel so the gimbal
        # ends back at the shallow-down starting position, not at the horizon.
        if self._state == "pitch_up":
            if elapsed >= cfg.INIT_SCAN_PITCH_RETURN_TIME:
                self._pitch_level = 0
                self._sweep_done  = 0
                self._state       = "sweep"
                self._phase_t     = now
                print("[InitScan] Raster complete — restarting from top")
            return 0, cfg.INIT_SCAN_PITCH_SPEED  # positive = up (back to shallow-down)

        # Yaw sweep at current pitch level.
        if elapsed >= cfg.INIT_SCAN_SWEEP_TIME:
            self._sweep_done += 1
            self._yaw_dir    *= -1
            self._phase_t     = now
            if self._sweep_done >= cfg.INIT_SCAN_SWEEPS_PER_LEVEL:
                self._sweep_done = 0
                if self._pitch_level < self.PHASES - 1:
                    self._state = "pitch_dn"
                    return 0, -cfg.INIT_SCAN_PITCH_SPEED   # down to next level
                else:
                    self._state = "pitch_up"
                    return 0, cfg.INIT_SCAN_PITCH_SPEED  # back toward shallow-down

        return cfg.INIT_SCAN_SPEED * self._yaw_dir, 0

    @property
    def status(self) -> str:
        arrow = ">>>" if self._yaw_dir > 0 else "<<<"
        return f"L{self._pitch_level + 1}/{self.PHASES} {arrow}"


# =============================================================================
#  SECTOR SCAN  (velocity-biased, three phases)
# =============================================================================

class SectorScanSearch:
    """Three-phase sector scan biased toward the last known travel direction.

    Phase 1: ±60° arc (2 sweeps)
    Phase 2: ±90° arc (2 sweeps)
    Phase 3: Full ±180° sweep (fallback)
    """

    def __init__(self) -> None:
        self.active: bool          = False
        self.phase: int            = 0
        self.yaw_direction: int    = 1
        self.pitch_direction: int  = 0
        self.sweep_start_time: float = 0.0
        self.sweep_count: int      = 0
        self.pitch_scanning: bool  = False
        self.pitch_scan_start: float = 0.0
        self.current_speed: int    = cfg.SEARCH_SPEED

    def start(self, yaw_dir: int = 1, pitch_dir: int = 0) -> None:
        """Begin sector scan.

        Args:
            yaw_dir:   Predicted movement direction (+1=right, -1=left).
            pitch_dir: Predicted pitch direction  (-1=down,  +1=up).
        """
        self.active           = True
        self.phase            = 1
        self.yaw_direction    = yaw_dir if yaw_dir != 0 else 1
        self.pitch_direction  = pitch_dir
        self.sweep_start_time = time.time()
        self.sweep_count      = 0
        self.pitch_scanning   = False
        self.current_speed    = cfg.SEARCH_SPEED
        label = "R" if yaw_dir > 0 else "L"
        print(f"[Search] Sector scan started: phase 1, dir={label}")

    def stop(self) -> None:
        self.active         = False
        self.phase          = 0
        self.pitch_scanning = False

    def get_command(self) -> tuple[int, int]:
        """Return (yaw_speed, pitch_speed) for current search state."""
        if not self.active:
            return 0, 0

        now = time.time()

        if self.pitch_scanning:
            if now - self.pitch_scan_start > cfg.SEARCH_PITCH_SCAN_DURATION:
                self.pitch_scanning   = False
                self.sweep_start_time = now
            return 0, cfg.SEARCH_PITCH_SCAN_SPEED * self.pitch_direction

        elapsed   = now - self.sweep_start_time
        sweep_dur = self._sweep_duration()

        if elapsed >= sweep_dur:
            self.yaw_direction *= -1
            self.sweep_count   += 1
            self.sweep_start_time = now

            if self.sweep_count >= self._max_sweeps():
                self.pitch_scanning   = True
                self.pitch_scan_start = now
                # Always scan downward — the person is on the ground, never in
                # the sky. Negative pitch is down in the SIYI speed convention.
                self.pitch_direction  = -1
                self.sweep_count = 0
                if self.phase < 3:
                    self.phase        += 1
                    self.current_speed = self._speed()
                    print(f"[Search] Phase {self.phase}")
                return 0, cfg.SEARCH_PITCH_SCAN_SPEED * self.pitch_direction

        return self.current_speed * self.yaw_direction, 0

    def _sweep_duration(self) -> float:
        if self.phase == 1:
            return cfg.SEARCH_SWEEP_DURATION
        elif self.phase == 2:
            return cfg.SEARCH_SWEEP_DURATION * 1.5
        return cfg.SEARCH_SWEEP_DURATION * 2.0

    def _max_sweeps(self) -> int:
        if self.phase == 1:
            return cfg.SEARCH_PHASE1_SWEEPS * 2
        elif self.phase == 2:
            return cfg.SEARCH_PHASE2_SWEEPS * 2
        return 4

    def _speed(self) -> int:
        if self.phase == 1:
            return cfg.SEARCH_SPEED
        elif self.phase == 2:
            return cfg.SEARCH_SPEED + 5
        return cfg.SEARCH_FALLBACK_SPEED

    @property
    def phase_name(self) -> str:
        return {0: "OFF", 1: "SECTOR-120", 2: "SECTOR-180", 3: "FULL-SWEEP"}.get(
            self.phase, "?"
        )


# =============================================================================
#  EXPANDING SQUARE  (IAMSAR standard, open-loop time-domain)
# =============================================================================

class ExpandingSquareSearch:
    """IAMSAR expanding-square search.

    Arms alternate yaw/pitch; length doubles every two arms.
    Guaranteed to cover all directions — coverage area ∝ arm² × base_arc².
    """

    def __init__(self) -> None:
        self.active: bool       = False
        self._arm: int          = 0
        self._arm_t: float      = 0.0
        self._yaw_first: int    = 1
        self._pitch_first: int  = -1   # -1 = down (negative = toward ground)

    def start(self, yaw_dir: int = 1, pitch_dir: int = 0) -> None:
        """Begin expanding-square search.

        Args:
            yaw_dir:   Starting yaw direction (+1 or -1).
            pitch_dir: 0 = unknown → default -1 (scan down toward ground).
        """
        self.active       = True
        self._arm         = 0
        self._arm_t       = time.time()
        self._yaw_first   = yaw_dir   if yaw_dir   != 0 else 1
        self._pitch_first = pitch_dir if pitch_dir < 0 else -1  # never use +1 (up)
        print(
            f"[ExpandSq] Expanding-square search started "
            f"(dir={'R' if self._yaw_first > 0 else 'L'}, "
            f"base={cfg.EXP_SQUARE_BASE_TIME}s, "
            f"max_arms={cfg.EXP_SQUARE_MAX_ARMS})"
        )

    def stop(self) -> None:
        self.active = False

    def _arm_duration(self) -> float:
        """Length doubles every 2 arms: 1,1,2,2,3,3,4,4,... × base."""
        return cfg.EXP_SQUARE_BASE_TIME * ((self._arm // 2) + 1)

    def _arm_cmd(self) -> tuple[int, int]:
        """Cycle: yaw+, pitch-down, yaw-, hold.

        The fourth arm (pitch-up in the original IAMSAR pattern) is replaced
        with a neutral hold so the camera never tilts above horizontal.
        Person is on the ground — pitching upward wastes time searching sky.
        """
        mod = self._arm % 4
        if mod == 0:
            return  self._yaw_first * cfg.EXP_SQUARE_SPEED,      0
        elif mod == 1:
            return  0,              -cfg.EXP_SQUARE_PITCH_SPEED   # always down (-)
        elif mod == 2:
            return -self._yaw_first * cfg.EXP_SQUARE_SPEED,      0
        else:
            return  0, 0   # hold — no pitch-up arm

    def get_command(self) -> tuple[int, int]:
        if not self.active:
            return 0, 0
        now = time.time()
        if now - self._arm_t >= self._arm_duration():
            if self._arm < cfg.EXP_SQUARE_MAX_ARMS - 1:
                self._arm += 1
            self._arm_t = now
        return self._arm_cmd()

    @property
    def status(self) -> str:
        labels = {0: "YAW+", 1: "PITCH", 2: "YAW-", 3: "PITCH-"}
        return f"arm {self._arm + 1}/{cfg.EXP_SQUARE_MAX_ARMS} {labels.get(self._arm % 4, '?')}"


# =============================================================================
#  LISSAJOUS LONG-DURATION SEARCH  (Scientific Reports 2024 / IEEE 2023)
# =============================================================================

class LissajousSearch:
    """Sinusoidal velocity commands tracing a Lissajous path.

    yaw_pos(t)   ≈ A_yaw   × sin(2π t / T_yaw)
    pitch_pos(t) ≈ A_pitch × sin(2π t / T_pitch)

    T_yaw/T_pitch = 40/49 ≈ 0.8163 ≈ √2/√3 (irrational).
    Irrational ratio → path never repeats → asymptotically fills workspace.

    Re-centres periodically to correct open-loop drift.
    Proven 80% POD faster than expanding square for moving targets.
    """

    def __init__(self) -> None:
        self.active: bool               = False
        self._t0: float                 = 0.0
        self._last_recenter: float      = 0.0
        self._recentering: bool         = False
        self._recenter_end: float       = 0.0
        self._need_center_cmd: bool     = False

    def start(self) -> None:
        self.active             = True
        self._t0                = time.time()
        self._last_recenter     = self._t0
        self._recentering       = False
        self._need_center_cmd   = False
        ratio = cfg.LISSAJOUS_YAW_PERIOD / cfg.LISSAJOUS_PITCH_PERIOD
        print(
            f"[Lissajous] Long-duration search started "
            f"T_yaw={cfg.LISSAJOUS_YAW_PERIOD}s  "
            f"T_pitch={cfg.LISSAJOUS_PITCH_PERIOD}s  "
            f"ratio={ratio:.4f}≈√2/√3"
        )

    def stop(self) -> None:
        self.active       = False
        self._recentering = False

    @property
    def needs_center_cmd(self) -> bool:
        """True exactly once when a re-centre is triggered."""
        if self._need_center_cmd:
            self._need_center_cmd = False
            return True
        return False

    def get_command(self) -> tuple[int, int]:
        if not self.active:
            return 0, 0

        now = time.time()

        if not self._recentering and now - self._last_recenter >= cfg.LISSAJOUS_RECENTER_PERIOD:
            self._recentering     = True
            self._recenter_end    = now + cfg.LISSAJOUS_RECENTER_DWELL
            self._last_recenter   = now
            self._need_center_cmd = True
            print("[Lissajous] Re-centering to correct drift...")

        if self._recentering:
            if now >= self._recenter_end:
                self._recentering = False
                self._t0          = now
                print("[Lissajous] Re-centre done — resuming")
            return 0, 0

        t     = now - self._t0
        w_y   = 2.0 * math.pi / cfg.LISSAJOUS_YAW_PERIOD
        w_p   = 2.0 * math.pi / cfg.LISSAJOUS_PITCH_PERIOD
        yaw   = int(cfg.LISSAJOUS_YAW_SPEED  * math.cos(w_y * t))
        # Pitch: use (1 - cos) / 2 so it oscillates between 0 (neutral) and
        # -PITCH_SPEED (full down) — never tilts up toward sky.
        pitch = -int(cfg.LISSAJOUS_PITCH_SPEED * (1.0 - math.cos(w_p * t)) / 2.0)
        return yaw, pitch

    @property
    def status(self) -> str:
        if self._recentering:
            return "RE-CENTERING"
        t   = time.time() - self._t0
        pct = (t % cfg.LISSAJOUS_YAW_PERIOD) / cfg.LISSAJOUS_YAW_PERIOD
        return f"t={t:.0f}s cyc={pct:.0%}"
