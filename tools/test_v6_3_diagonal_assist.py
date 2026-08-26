#!/usr/bin/env python3
"""Deterministic safety tests for the v6-3 diagonal-only supplement."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drive.v6_3_diagonal_assist import DiagonalAssist, DiagonalAssistConfig


def update(assist, raw, *, slope=0.08, view=0.9, ok=True, cone=False):
    return assist.update(raw_steer=raw, slope=slope, view=view, ok=ok,
                         cone_active=cone)


cfg = DiagonalAssistConfig(enter_frames=3, exit_frames=2, slope_enter=0.055,
                           slope_hold=0.035, slope_exit_now=0.18,
                           extra_slope_gain=13.0, max_assist_deg=2.8,
                           assist_slew_deg=0.9)
a = DiagonalAssist(cfg)

# Exact straight: v4 command is literally unchanged, including its small
# centring corrections.  This avoids a second straight-line control law.
out, active, added = update(a, 0.8, slope=0.0)
assert (out, active, added) == (0.8, False, 0.0)

# A rightward diagonal has positive image slope and the established v4 command
# is negative.  It earns a bounded same-direction supplement only after proof.
for _ in range(2):
    out, active, added = update(a, -2.0)
    assert (out, active, added) == (-2.0, False, 0.0)
out, active, added = update(a, -2.0)
assert active and out <= -2.0 and -2.8 <= added <= 0.0, (out, active, added)
out, active, added = update(a, -2.0)
assert active and -2.8 <= added < 0.0 and out < -2.0, (out, active, added)

# Cone handling, blind recovery, sharp corners, and v4 direction disagreement
# all return the original command immediately.
for kwargs in ({"cone": True}, {"ok": False}, {"slope": 0.20}, {"raw": 2.0}):
    raw = kwargs.pop("raw", -7.0)
    out, active, added = update(a, raw, **kwargs)
    assert out == raw and not active and added == 0.0, (kwargs, out, active, added)

# The other diagonal direction is symmetric and still cannot exceed the cap.
a = DiagonalAssist(cfg)
for _ in range(3):
    out, active, added = update(a, 2.0, slope=-0.11)
assert active and 0.0 <= added <= 2.8 and out >= 2.0, (out, active, added)
print("v6-3 diagonal assist tests: PASS")
