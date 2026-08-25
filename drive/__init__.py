"""Lane-following stack for the 2026 AMET hackathon car.

    robot    — web API wrapper (camera in, wheels out)
    lane     — single-frame lane estimate, no cross-frame state
    control  — look-ahead steering + curvature-based speed
"""
from . import control, lane, robot

__all__ = ["control", "lane", "robot"]
