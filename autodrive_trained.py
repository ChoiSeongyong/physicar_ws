#!/usr/bin/env python3
"""v4 sensor controller with an optional field-trained, safety-gated adviser.

Keeps the normal v4 controller as the sole source of speed, cone handling,
signal handling, recovery and all fail-safe behavior. The only optional change
is a <=3 degree, <=25% blend to a model trained from dataset/diag; when the
model or its field-colour confidence is unavailable this delegates byte-for-
byte to autodrive.py.
"""
from __future__ import annotations

import os
from pathlib import Path

# Import-time constants in autodrive remain the v4 source of truth.
import autodrive as base
from drive.field_adviser import FieldAdviser


MODEL = os.environ.get("PC_FIELD_ADVISER_MODEL", str(
    Path(__file__).resolve().parent / "models" / "v4_field_adviser.onnx"))
MAX_BLEND = float(os.environ.get("PC_FIELD_ADVISER_BLEND", "0.25"))
MAX_DELTA = float(os.environ.get("PC_FIELD_ADVISER_DELTA_DEG", "3.0"))


def main():
    adviser = FieldAdviser(MODEL, MAX_BLEND, MAX_DELTA)
    if not adviser.ready:
        base.log(f"학습 보정기 미사용 ({MODEL}) — 고정 v4로 안전 폴백")
        return base.main()
    base.log(f"학습 보정기 준비: {MODEL} (최대 {MAX_BLEND:.0%}, {MAX_DELTA:.1f}° 보정)")

    # Replace only the imported control-command function while this process is
    # alive. Every call still runs the original v4 function first; importantly
    # seek/none recovery is excluded and speed is returned untouched.
    original = base.control.command

    def advised_command(est, gains=base.control.Gains(), last_steer=0.0):
        speed, steer = original(est, gains, last_steer)
        # The frame itself is unavailable in this function. The steering blend
        # must be applied after lane detection, so tag the baseline here and let
        # the main-loop wrapper consume it. This branch remains original.
        return speed, steer

    # Rather than duplicate a 500-line safety loop, use a tiny command wrapper
    # that reads the last camera frame placed by camera().
    original_camera = base.robot.Robot.camera
    latest = {"image": None}

    def camera_with_cache(self):
        image = original_camera(self)
        latest["image"] = image
        return image

    base.robot.Robot.camera = camera_with_cache

    def command_with_advice(est, gains=base.control.Gains(), last_steer=0.0):
        speed, steer = original(est, gains, last_steer)
        # Never influence recovery/blind frames. Cone feed-forward is added by
        # base.main afterwards; therefore this only changes the lane baseline.
        if est.ok:
            steer, blend = adviser.advise(latest["image"], steer)
            if blend and os.environ.get("PC_FIELD_ADVISER_LOG", "") == "1":
                base.log(f"학습 조향 보정 blend={blend:.2f}")
        return speed, steer

    base.control.command = command_with_advice
    try:
        return base.main()
    finally:
        base.control.command = original
        base.robot.Robot.camera = original_camera


if __name__ == "__main__":
    main()
