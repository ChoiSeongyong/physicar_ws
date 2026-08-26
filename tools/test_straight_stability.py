#!/usr/bin/env python3
"""Small deterministic safety tests for the v6-2 straight-only wrapper."""
from pathlib import Path
import sys

# Allow direct execution from tools/ while importing the workspace package.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drive.straight_stability import StraightStabilityConfig, StraightStabilizer


def update(filter_, raw, *, slope=0.01, view=0.9, ok=True, cone=False):
    return filter_.update(raw_steer=raw, ok=ok, slope=slope, view=view,
                          cone_active=cone)


cfg = StraightStabilityConfig(enter_frames=3, exit_frames=2, alpha=0.24,
                              deadband_deg=1.0, gain=0.68, slew_deg=1.1,
                              reverse_frames=3)
f = StraightStabilizer(cfg)

# Engagement has no command step, then alternating small camera noise settles
# into the neutral zone instead of throwing the steering left/right.
for raw in (1.4, 1.2):
    out, filtered = update(f, raw)
    assert out == raw and not filtered
out, filtered = update(f, 1.3)
assert filtered and abs(out - 1.3) < 1e-9
noise_output = []
for raw in (-1.7, 1.6, -1.5, 1.8, -1.6, 1.5):
    out, filtered = update(f, raw)
    assert filtered
    noise_output.append(out)
assert max(map(abs, noise_output)) <= 1.3, noise_output

# Cone, recovery/blind frame, or a decisive curve instantly returns v4 raw steer.
for kwargs in ({"cone": True}, {"ok": False}, {"slope": 0.20}):
    out, filtered = update(f, 13.0, **kwargs)
    assert out == 13.0 and not filtered, (kwargs, out, filtered)

# Re-enter and ensure a persistent opposite straight correction is allowed,
# but only after its confirmation frames.
f = StraightStabilizer(cfg)
for raw in (2.0, 2.0, 2.0):
    update(f, raw)
first, _ = update(f, -4.0)
second, _ = update(f, -4.0)
third, _ = update(f, -4.0)
assert first >= 0.0 and second >= 0.0 and third <= 0.0, (first, second, third)
print("straight stability tests: PASS")
