"""Camera and lidar, combined: the camera says what, the lidar says how far.

Each sensor is strong exactly where the other is weak, and the measurements say
so rather than the intuition.

The **lidar** measures range to about a centimetre, and inside 2 m it is
dependable — 84-88% precision at 100% recall (tools/cone_eval.py). Past that it
falls apart: 47% precision at 2-3 m and 22% at 3-4 m, because a wall corner or
the edge of the road makes the same narrow, deep local minimum a cone does. That
matters, because 2-4 m is precisely where a reaction has to begin. At 3 m/s a
cone seen at 1 m leaves a third of a second.

The **camera** has the opposite profile. It can tell a cone from a wall at any
range a cone is visible at, which is what the learned detector is for, and it
transfers to a real cone under hall lighting in a way a hand-tuned colour
threshold does not. What it cannot do is measure distance: a bounding box gives
a bearing, not a range.

So: take the bearing from the camera, take the range from the lidar, and use
each detector to veto the other's mistakes. A lidar spike with no cone visible
along that bearing is scenery and is dropped — that is the 2-4 m false positive
rate, removed. A cone the camera sees with no lidar return behind it still gets
a usable range from its box height, since the cone's real height is known.

Nothing here needs the simulator. The same code runs on the real car with the
same two sensors, which is the point.
"""
import math
import os
from dataclasses import dataclass

import numpy as np

from drive import cones as cone_mod

# Camera intrinsics, from the sensor definition: 1.7453 rad across 480 px.
IMG_W, IMG_H = 480, 360
FOCAL = (IMG_W / 2) / math.tan(1.7453 / 2)      # 201.4 px
CONE_H = 0.23                                   # metres

# How far apart in bearing a camera box and a lidar spike may be and still be
# taken for the same object. The calibration reprojects to about 13 px RMS,
# which at this focal length is 3.6 degrees, so 7 is a couple of sigma.
ASSOC_DEG = float(os.environ.get("PC_FUSE_ASSOC_DEG", 7.0))

# Below this range the lidar is trustworthy on its own (measured 84-88%
# precision), and the camera may not even see a cone that close — it passes
# under the frame as the car reaches it. Vetoing there would blind the car at
# exactly the moment it must not be blind.
LIDAR_ALONE_M = float(os.environ.get("PC_FUSE_LIDAR_ALONE", 1.6))

# How far a camera-only detection may be and still be reported.
#
# This was the lidar's own 3.6 m, which threw away exactly the band the camera
# was added for: measured against the same visibility rule the labels use, the
# detector finds 80% of cones at 3-4 m, where the lidar finds none at all. The
# range that comes back with those is rough — it is inferred from box height,
# and a pixel of error is 20 cm at 3 m — but a rough range at 4 m is worth far
# more than an exact one at 1.6 m, which is all the geometry alone ever gave.
CAM_MAX_RANGE = float(os.environ.get("PC_FUSE_CAM_RANGE", 4.5))

CONF = float(os.environ.get("PC_FUSE_CONF", 0.25))
IMGSZ = int(os.environ.get("PC_FUSE_IMGSZ", 416))


@dataclass
class Fused:
    distance: float
    bearing: float          # degrees, + = left
    source: str             # "both" | "lidar" | "camera"
    conf: float = 1.0

    @property
    def forward(self):
        return self.distance * math.cos(math.radians(self.bearing))

    @property
    def lateral(self):
        return self.distance * math.sin(math.radians(self.bearing))


def bearing_of(u):
    """Bearing in degrees of an image column (+ = left)."""
    return math.degrees(math.atan((IMG_W / 2 - u) / FOCAL))


def range_from_height(px_h):
    """Range implied by a box's pixel height, for a cone of known height.

    Only used when the lidar has nothing at that bearing. It is far less
    accurate than a range measurement — a couple of pixels of box error is tens
    of centimetres at 3 m — so it is a fallback, never the primary.
    """
    if px_h <= 1:
        return None
    return FOCAL * CONE_H / px_h


class Detector:
    """Cones from both sensors. Falls back to lidar alone with no model."""

    def __init__(self, weights="", conf=CONF, imgsz=IMGSZ):
        self.model = None
        self.conf = conf
        self.imgsz = imgsz
        if weights and os.path.exists(weights):
            from ultralytics import YOLO
            self.model = YOLO(weights, task="detect")

    def camera_cones(self, img):
        """(bearing_deg, confidence, pixel_height) for each detection."""
        if self.model is None or img is None:
            return []
        res = self.model(img, imgsz=self.imgsz, conf=self.conf,
                         verbose=False)[0]
        out = []
        for b in res.boxes:
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
            out.append((bearing_of((x1 + x2) / 2), float(b.conf[0]), y2 - y1))
        return out

    def detect(self, img, scan):
        lidar = cone_mod.detect(scan) if scan is not None else []
        cam = self.camera_cones(img)

        # With no model there is nothing to fuse; behave exactly as before so
        # the car still drives if the weights are missing on the day.
        if self.model is None:
            return [Fused(c.distance, c.bearing, "lidar") for c in lidar]

        out = []
        used = set()
        for c in lidar:
            j, best = None, ASSOC_DEG
            for i, (bear, conf, _) in enumerate(cam):
                if i in used:
                    continue
                d = abs(bear - c.bearing)
                if d < best:
                    best, j = d, i
            if j is not None:
                used.add(j)
                out.append(Fused(c.distance, c.bearing, "both", cam[j][1]))
            elif c.forward <= LIDAR_ALONE_M:
                # Close in, the lidar is reliable and the camera may have lost
                # the cone below the frame. Believe the range measurement.
                out.append(Fused(c.distance, c.bearing, "lidar"))
            # else: a distant spike the camera cannot confirm — the wall.

        for i, (bear, conf, px_h) in enumerate(cam):
            if i in used:
                continue
            r = range_from_height(px_h)
            if r is not None and r < CAM_MAX_RANGE:
                out.append(Fused(r, bear, "camera", conf))

        out = [f for f in out if abs(f.lateral) <= cone_mod.MAX_LATERAL]
        out.sort(key=lambda f: f.distance)
        return out


def as_cones(fused):
    """Adapt to the Cone shape the existing avoidance already consumes."""
    return [cone_mod.Cone(distance=f.distance, bearing=f.bearing, width=2.0)
            for f in fused]
