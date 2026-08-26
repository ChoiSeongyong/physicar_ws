"""v6-2-only straight-line steering stabilisation.

This module is deliberately imported only by ``autodrive_v6_2.py``.  It does
not alter the v4/v6-1 control path: corners, recovery, blind-frame handling,
and cone avoidance retain their existing controller command unchanged.
"""
from __future__ import annotations

from dataclasses import dataclass


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class StraightStabilityConfig:
    """Conservative parameters for a low-speed (0.8 m/s) straight section."""

    enter_frames: int = 4
    exit_frames: int = 2
    alpha: float = 0.24
    deadband_deg: float = 1.0
    gain: float = 0.68
    slew_deg: float = 1.1
    reverse_frames: int = 3
    view_enter: float = 0.70
    view_hold: float = 0.62
    view_exit_now: float = 0.54
    slope_enter: float = 0.075
    slope_hold: float = 0.12
    slope_exit_now: float = 0.16
    steer_enter: float = 6.5
    steer_hold: float = 8.5
    steer_exit_now: float = 10.0


class StraightStabilizer:
    """Stateful noise filter with hysteresis, scoped strictly to clear straights.

    A one-frame slope/view glitch must not switch filtering off, because that
    switch itself was a source of alternating wheel commands in v6-1.  In the
    other direction, a clear curve cue releases immediately, so this class
    never delays v4 turn-in or an avoidance/recovery command.
    """

    def __init__(self, config: StraightStabilityConfig) -> None:
        self.config = config
        self.active = False
        self.enter_count = 0
        self.exit_count = 0
        self.ema: float | None = None
        self.output = 0.0
        self.reverse_count = 0
        self.reverse_sign = 0

    def _reset(self) -> None:
        self.active = False
        self.enter_count = 0
        self.exit_count = 0
        self.ema = None
        self.reverse_count = 0
        self.reverse_sign = 0

    def _enter_ok(self, *, ok: bool, cone_active: bool, view: float,
                  slope: float, raw_steer: float) -> bool:
        c = self.config
        return (ok and not cone_active and view >= c.view_enter
                and abs(slope) <= c.slope_enter
                and abs(raw_steer) <= c.steer_enter)

    def _hold_ok(self, *, ok: bool, cone_active: bool, view: float,
                 slope: float, raw_steer: float) -> bool:
        c = self.config
        return (ok and not cone_active and view >= c.view_hold
                and abs(slope) <= c.slope_hold
                and abs(raw_steer) <= c.steer_hold)

    def _exit_now(self, *, ok: bool, cone_active: bool, view: float,
                  slope: float, raw_steer: float) -> bool:
        c = self.config
        return (not ok or cone_active or view < c.view_exit_now
                or abs(slope) >= c.slope_exit_now
                or abs(raw_steer) >= c.steer_exit_now)

    def update(self, *, raw_steer: float, ok: bool, view: float, slope: float,
               cone_active: bool) -> tuple[float, bool]:
        """Return ``(steering_deg, filtered)`` for the current controller tick."""
        c = self.config
        if not self.active:
            if self._enter_ok(ok=ok, cone_active=cone_active, view=view,
                              slope=slope, raw_steer=raw_steer):
                self.enter_count += 1
            else:
                self.enter_count = 0
            if self.enter_count < max(1, c.enter_frames):
                return raw_steer, False
            # Start from the current v4 command.  Thus engaging cannot make a
            # steering step, even when the preceding bend ended with lock held.
            self.active = True
            self.exit_count = 0
            self.ema = raw_steer
            self.output = raw_steer
            self.reverse_count = 0
            self.reverse_sign = 0
            # Engagement itself must be exactly the v4 command; filtering begins
            # on the following stable straight-frame.
            return raw_steer, True

        # Do not preserve any v6 state through an actual curve/avoidance cue.
        if self._exit_now(ok=ok, cone_active=cone_active, view=view,
                          slope=slope, raw_steer=raw_steer):
            self._reset()
            return raw_steer, False
        if self._hold_ok(ok=ok, cone_active=cone_active, view=view,
                         slope=slope, raw_steer=raw_steer):
            self.exit_count = 0
        else:
            self.exit_count += 1
            if self.exit_count >= max(1, c.exit_frames):
                self._reset()
                return raw_steer, False

        alpha = _clamp(c.alpha, 0.0, 1.0)
        self.ema = raw_steer if self.ema is None else (
            alpha * raw_steer + (1.0 - alpha) * self.ema)
        magnitude = abs(self.ema)
        if magnitude <= c.deadband_deg:
            target = 0.0
        else:
            # Soft gain above the deadband removes centring chatter but retains
            # a sustained correction when the vehicle really drifts sideways.
            target = (1.0 if self.ema > 0 else -1.0) * (
                c.deadband_deg + c.gain * (magnitude - c.deadband_deg))

        # A genuine side-to-side correction persists; a one/two-frame camera
        # sign flicker becomes neutral instead of crossing the steering rack.
        if target and self.output and (target > 0) != (self.output > 0):
            sign = 1 if target > 0 else -1
            if sign == self.reverse_sign:
                self.reverse_count += 1
            else:
                self.reverse_sign = sign
                self.reverse_count = 1
            if self.reverse_count < max(1, c.reverse_frames):
                target = 0.0
            else:
                self.reverse_count = 0
                self.reverse_sign = 0
        else:
            self.reverse_count = 0
            self.reverse_sign = 0

        if c.slew_deg > 0:
            target = _clamp(target, self.output - c.slew_deg,
                            self.output + c.slew_deg)
        self.output = target
        return target, True
