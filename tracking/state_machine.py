"""
state_machine.py — Tracker state enumeration.

Eight states in priority order (highest to lowest):
  INITIAL_SCAN    : Power-on acquisition raster (runs before any person found).
  TRACKING        : Person detected and gimbal actively centring.
  PREDICTING      : Person lost; full-speed prediction from last velocity (0–3 s).
  PRED_FADE       : Fading prediction (3–6 s after loss); speed tapers to 30%.
  SEARCHING       : Velocity-biased sector scan (escalates after 15 s).
  EXPANDING_SQUARE: IAMSAR expanding-square (escalates after 45 s).
  LISSAJOUS       : Sinusoidal fill search (runs indefinitely until found).
  WAITING         : Tracking disabled or search disabled; gimbal holds still.
"""


class State:
    """Tracker finite state machine states (string constants)."""

    TRACKING         = "TRACKING"
    PREDICTING       = "PREDICTING"
    PRED_FADE        = "PRED_FADE"
    SEARCHING        = "SEARCHING"        # velocity-biased sector scan
    EXPANDING_SQUARE = "EXP_SQUARE"       # IAMSAR expanding square
    LISSAJOUS        = "LISSAJOUS"        # long-duration Lissajous fill
    INITIAL_SCAN     = "INIT_SCAN"        # power-on acquisition raster
    WAITING          = "WAITING"          # idle

    # Convenience sets for isinstance-style membership tests
    SEARCH_STATES: frozenset[str] = frozenset({
        SEARCHING, EXPANDING_SQUARE, LISSAJOUS, INITIAL_SCAN,
    })

    ACTIVE_STATES: frozenset[str] = frozenset({
        TRACKING, PREDICTING, PRED_FADE,
    })
