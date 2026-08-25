"""Thin wrapper over the PhysiCar web API.

The API is the same on the simulator and on the real kit, so nothing here is
sim-specific. Two things it takes care of that are easy to get wrong:

* Speed commands expire after the driver's ~1 s watchdog. A control loop must
  keep re-sending, which `drive()` assumes the caller does every tick.
* The API speaks radians; the rest of this package works in degrees, because
  the hardware limit is quoted in degrees (±20°) and mixing the two silently
  produces a car that barely turns.
"""
import math
import os

import cv2
import numpy as np
import requests

BASE_URL = os.environ.get("PHYSICAR_URL", "http://localhost")

STEER_LIMIT_DEG = 20.0      # hardware limit; the API clamps anyway
SPEED_LIMIT = 3.0


class Robot:
    def __init__(self, base_url=BASE_URL, timeout=2.0):
        self.base = base_url.rstrip("/")
        self.timeout = timeout
        self._session = requests.Session()

    # ── sensors ──────────────────────────────────────────────────────────

    def camera(self, width=None, height=None):
        """Latest frame as BGR. None when the stream is not up yet.

        The webserver subscribes to the camera topic on first request, so the
        first call or two after boot legitimately return "Camera not available"
        rather than an image — callers should retry, not crash.
        """
        params = {}
        if width:
            params["width"] = width
        if height:
            params["height"] = height
        r = self._session.get(f"{self.base}/camera", params=params,
                              timeout=self.timeout)
        if not r.ok or not r.content[:2] == b"\xff\xd8":     # not a JPEG
            return None
        return cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)

    def imu(self):
        """Acceleration, gyro and fused orientation.

        Measured against ground truth: gyro z tracks the real yaw rate almost
        exactly (21.2 vs 21.6, 42.8 vs 43.2, 83.6 vs 84.6 deg/s), and a
        collision is unmistakable — horizontal acceleration sits at 0.01 m/s2
        while driving and peaks at 10 m/s2 on contact.
        """
        return self._session.get(f"{self.base}/imu", timeout=self.timeout).json()

    def speed_now(self):
        """Measured forward speed, m/s. None if unavailable.

        This has to come from odometry, not the IMU. Steady driving produces no
        acceleration and no rotation, so an IMU-based "is it moving?" test calls
        a car cruising in a straight line stuck — which it did, firing reverse
        manoeuvres mid-lap and costing ten seconds a run.
        """
        try:
            d = self._session.get(f"{self.base}/odom", timeout=self.timeout).json()
            return float(d["velocity"]["linear"])
        except Exception:                                  # noqa: BLE001
            return None

    def lidar(self, step=None):
        params = {"step": step} if step else {}
        return self._session.get(f"{self.base}/lidar", params=params,
                                 timeout=self.timeout).json()

    # ── actuators ────────────────────────────────────────────────────────

    def drive(self, speed, steering_deg):
        """Send one speed+steering command. Call every control tick."""
        steering_deg = max(-STEER_LIMIT_DEG, min(STEER_LIMIT_DEG, steering_deg))
        speed = max(-SPEED_LIMIT, min(SPEED_LIMIT, speed))
        self._session.post(f"{self.base}/steering",
                           json={"value": math.radians(steering_deg)},
                           timeout=self.timeout)
        self._session.post(f"{self.base}/speed", json={"value": float(speed)},
                           timeout=self.timeout)

    def stop(self):
        """Explicit stop. Safe to call from a finally block."""
        try:
            self.drive(0.0, 0.0)
        except requests.RequestException:
            pass

    def look(self, pan_deg=0.0, tilt_deg=0.0):
        for name, deg in (("pan", pan_deg), ("tilt", tilt_deg)):
            self._session.post(f"{self.base}/camera/{name}",
                               json={"value": math.radians(deg)},
                               timeout=self.timeout)
