"""v6-3 sensor-only diagonal turn-in assist.

The v4 command remains authoritative everywhere except a confirmed, gentle
high-visibility diagonal.  In particular, a true straight is passed through
unchanged (rather than filtered), and cones, recovery/blind frames, sharp
corners, or a disagreement between the v4 command and the observed bend all
bypass this module immediately.

The assist adds only a bounded amount of steering in the direction that v4 has
already selected.  It cannot select a side, reverse a command, change speed,
or persist into a corner.  This is intentionally a small predictive supplement
to the existing slope term: it counters the observed tendency to enter a
long diagonal too softly without replacing any v4 safety behaviour.
"""
from __future__ import annotations

from dataclasses import dataclass


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _sign(value: float) -> int:
    return 1 if value > 0 else (-1 if value < 0 else 0)


@dataclass(frozen=True)
class DiagonalAssistConfig:
    """Conservative v6-3 parameters for 0.8 m/s operation."""

    enter_frames: int = 3
    exit_frames: int = 2
    slope_enter: float = 0.055
    slope_hold: float = 0.035
    slope_exit_now: float = 0.18
    view_min: float = 0.64
    raw_steer_min: float = 0.50
    raw_steer_max: float = 9.0
    slope_alpha: float = 0.42
    extra_slope_gain: float = 13.0
    max_assist_deg: float = 2.8
    assist_slew_deg: float = 0.9


class DiagonalAssist:
    """Confirmed same-direction turn-in supplement, never a steering filter."""

    def __init__(self, config: DiagonalAssistConfig) -> None:
        self.config = config
        self.active = False
        self.enter_count = 0
        self.exit_count = 0
        self.direction = 0
        self.slope_ema: float | None = None
        self.assist = 0.0

    def _reset(self) -> None:
        self.active = False
        self.enter_count = 0
        self.exit_count = 0
        self.direction = 0
        self.slope_ema = None
        self.assist = 0.0

    def _candidate(self, *, raw_steer: float, ok: bool, view: float,
                   slope: float, cone_active: bool, threshold: float) -> int:
        """Return the already-agreed steering direction, or zero to bypass."""
        c = self.config
        if (not ok or cone_active or view < c.view_min
                or abs(slope) < threshold or abs(slope) >= c.slope_exit_now
                or abs(raw_steer) < c.raw_steer_min
                or abs(raw_steer) > c.raw_steer_max):
            return 0
        # Positive slope means the lane continues right in image space; the v4
        # steering convention is therefore negative.  Do not assist if v4's
        # offset term disagrees: that may be a recovery/position correction.
        expected = -_sign(slope)
        return expected if _sign(raw_steer) == expected else 0

    def update(self, *, raw_steer: float, ok: bool, view: float, slope: float,
               cone_active: bool) -> tuple[float, bool, float]:
        """Return ``(steering_deg, active, added_deg)`` for this tick."""
        c = self.config
        entering = self._candidate(raw_steer=raw_steer, ok=ok, view=view,
                                   slope=slope, cone_active=cone_active,
                                   threshold=c.slope_enter)
        holding = self._candidate(raw_steer=raw_steer, ok=ok, view=view,
                                  slope=slope, cone_active=cone_active,
                                  threshold=c.slope_hold)
        if not self.active:
            if entering:
                if entering == self.direction:
                    self.enter_count += 1
                else:
                    self.direction = entering
                    self.enter_count = 1
            else:
                self.enter_count = 0
                self.direction = 0
            if self.enter_count < max(1, c.enter_frames):
                return raw_steer, False, 0.0
            self.active = True
            self.exit_count = 0
            self.slope_ema = slope
            self.assist = 0.0

        # Any sharp turn, cone/recovery signal, or directional disagreement
        # releases immediately.  v4 then owns the very same command tick.
        if not holding or holding != self.direction:
            self.exit_count += 1
            immediate = (not ok or cone_active or abs(slope) >= c.slope_exit_now
                         or _sign(raw_steer) != self.direction)
            if immediate or self.exit_count >= max(1, c.exit_frames):
                self._reset()
                return raw_steer, False, 0.0
            return raw_steer, True, 0.0
        self.exit_count = 0

        alpha = _clamp(c.slope_alpha, 0.0, 1.0)
        self.slope_ema = slope if self.slope_ema is None else (
            alpha * slope + (1.0 - alpha) * self.slope_ema)
        desired = self.direction * min(c.max_assist_deg,
                                       abs(self.slope_ema) * c.extra_slope_gain)
        if c.assist_slew_deg > 0:
            desired = _clamp(desired, self.assist - c.assist_slew_deg,
                             self.assist + c.assist_slew_deg)
        self.assist = desired
        return raw_steer + desired, True, desired
