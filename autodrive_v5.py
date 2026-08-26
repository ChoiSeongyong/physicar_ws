#!/usr/bin/env python3
"""v5 SIM-diversity adviser entry point.

The v4 sensor controller remains authoritative.  This entry point only selects
v5_sim_adviser.onnx for the existing bounded, confidence-gated adviser wrapper.
"""
from __future__ import annotations

import os
from pathlib import Path

# Set this before importing autodrive_trained, which reads its configuration at
# import time.  An explicit operator-provided model always wins.
os.environ.setdefault(
    "PC_FIELD_ADVISER_MODEL",
    str(Path(__file__).resolve().parent / "models" / "v5_sim_adviser.onnx"),
)

from autodrive_trained import main


if __name__ == "__main__":
    main()
