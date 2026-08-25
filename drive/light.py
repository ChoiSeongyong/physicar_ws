"""Traffic light: is it green yet?

Only one question matters — go or don't go — so red and yellow are treated
identically: keep waiting. That also means the detector never has to tell red
from yellow, which is the pair most likely to be ambiguous on a phone screen
under venue lighting.

**What the car will actually face is a picture of a signal on a phone.** At the
venue an official holds up an 8 cm phone displaying a traffic light — a lit
circle on a drawn housing, much like the simulated one — in the same place the
simulated light stands. So the *appearance* transfers reasonably well, but the
**scale does not**: the lit circle is only a fraction of an 8 cm screen, where
the simulated lamp is 5.6 cm of solid colour. Size assumptions are the thing
most likely to break, so they are set from that geometry rather than from the
simulator.

Cues that depend on how many lamps there are, or their order, or the housing
being dark, are still rejected — those vary with whatever graphic is displayed.
What survives is:

    a strongly saturated, bright, green patch, of a plausible size,
    roughly where the signal is known to be.

Saturation is doing the real work, and that is a measured result rather than an
intuition. The lit green lamp and the grass beside the track have *almost the
same hue* (60 vs 66-67) — a hue threshold alone fires on the lawn. They are
separated cleanly by saturation and value:

    lit lamp   H 60   S 251   V 248
    grass      H 66   S  75   V 163

With a floor of S>=110 / V>=110 the entire frame yields exactly one blob — the
lamp — and across 390 frames covering the whole lap, none at all.

The simulator does render the light, and this detector reads it correctly:
red gives H=0 at 349 px, green H=60 at 360 px, and the transition between them
shows nothing lit at all.

That took three attempts to establish, because **the rendered lamp lags the API
by about 4.5 seconds**, and a camera frame grabbed after an idle period can
still show the previous state. Twice this was measured as "the camera never
renders a light change" by commanding a state, sleeping, and grabbing a single
frame. Sampling continuously until the render agrees is the only reliable way
to read it — see `settle()` in tools/light_probe.py.
"""
import os
from dataclasses import dataclass

import cv2
import numpy as np

# Where the signal sits in the frame from the start line, measured at the
# official start pose (0.10 m behind waypoint 0): centre (0.885W, 0.610H),
# lamp 22x22 px at 0.85 m. The box below is generous around that, because a
# person holding a phone will not stand to pixel accuracy.
ROI = (
    float(os.environ.get("PC_LIGHT_X0", 0.68)),
    float(os.environ.get("PC_LIGHT_X1", 1.00)),
    float(os.environ.get("PC_LIGHT_Y0", 0.38)),
    float(os.environ.get("PC_LIGHT_Y1", 0.82)),
)

# Hue windows are wide; saturation and value are what actually discriminate.
GREEN_H = (38, 88)
RED_H_LO, RED_H_HI = (0, 10), (168, 180)
# Set from headroom, not from taste. Across 390 frames covering the whole lap
# the largest green blob inside the ROI is 0 px even at a floor of 90, so there
# is room to come down and catch a phone screen washed out by glare or dimmed.
# Grass sits at S 75 / V 163, so 110 keeps clear air above it.
MIN_S = int(os.environ.get("PC_LIGHT_MIN_S", 110))
MIN_V = int(os.environ.get("PC_LIGHT_MIN_V", 110))

# The floor is set by the smallest lamp the venue can plausibly present, not by
# what the simulator shows. At 0.85 m the camera resolves about 3.9 px per cm,
# so a lit circle drawn at a fifth of an 8 cm screen — a three-lamp signal in
# portrait, say — is only 6 px across and covers ~31 px. A floor of 60, which
# the solid-colour assumption made look safe, would have thrown that away.
# 10 px reaches down to a ~0.9 cm lamp — a three-lamp signal drawn small on the
# screen, which is the realistic worst case.
#
# Going this low is safe because the area floor is not the main noise filter:
# the caller requires green on three consecutive frames, and a speckle does not
# land in the same place three times running while a real lamp does. There is
# also no measured false-positive pressure to trade against — across 390 frames
# covering the whole lap the ROI contains no qualifying green blob at all.
MIN_AREA = int(os.environ.get("PC_LIGHT_MIN_AREA", 10))
MAX_AREA_FRAC = float(os.environ.get("PC_LIGHT_MAX_AREA_FRAC", 0.55))

# No morphological opening.
#
# It was costing a 5 px lamp four of its thirteen pixels — enough to drop it
# under the floor — and buying nothing measurable: the lap frames produce zero
# false positives with a 3x3 open, a 2x2 open, or none at all. The reasoning
# also holds in general. Anything that clears this filter is ten-plus connected
# pixels, saturated and bright, inside a narrow ROI, holding still for three
# consecutive frames. That is not speckle; it is an object — and an opening
# would not have removed it either.


@dataclass
class LightReading:
    state: str          # "green" | "stop" | "none"
    area: int           # pixels in the winning blob
    at: tuple           # (x, y) in image coordinates, or (-1, -1)

    @property
    def go(self):
        return self.state == "green"


def _roi_box(h, w):
    x0, x1, y0, y1 = ROI
    return (int(w * x0), min(w, int(w * x1)), int(h * y0), min(h, int(h * y1)))


def _best_blob(mask, limit):
    n, _, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    best = None
    for i in range(1, n):
        a = int(stats[i, cv2.CC_STAT_AREA])
        if a < MIN_AREA or a > limit:
            continue
        if best is None or a > best[0]:
            best = (a, float(cent[i][0]), float(cent[i][1]))
    return best


def detect(img):
    """Read the signal from one frame."""
    h, w = img.shape[:2]
    xa, xb, ya, yb = _roi_box(h, w)
    roi = img[ya:yb, xa:xb]
    if roi.size == 0:
        return LightReading("none", 0, (-1, -1))
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    limit = int(roi.shape[0] * roi.shape[1] * MAX_AREA_FRAC)

    green = cv2.inRange(hsv, (GREEN_H[0], MIN_S, MIN_V), (GREEN_H[1], 255, 255))
    g = _best_blob(green, limit)

    # Red is only read so the logs can show the signal was found and is simply
    # not green yet — it never gates anything, since red and yellow both mean
    # keep waiting.
    red = cv2.bitwise_or(
        cv2.inRange(hsv, (RED_H_LO[0], MIN_S, MIN_V), (RED_H_LO[1], 255, 255)),
        cv2.inRange(hsv, (RED_H_HI[0], MIN_S, MIN_V), (RED_H_HI[1], 255, 255)))
    r = _best_blob(red, limit)

    if g and (not r or g[0] >= r[0]):
        return LightReading("green", g[0], (g[1] + xa, g[2] + ya))
    if r:
        return LightReading("stop", r[0], (r[1] + xa, r[2] + ya))
    return LightReading("none", 0, (-1, -1))


def annotate(img, reading):
    out = img.copy()
    h, w = out.shape[:2]
    xa, xb, ya, yb = _roi_box(h, w)
    cv2.rectangle(out, (xa, ya), (xb - 1, yb - 1), (200, 200, 200), 1)
    colour = {"green": (0, 220, 0), "stop": (0, 0, 230)}.get(reading.state,
                                                            (150, 150, 150))
    if reading.at[0] >= 0:
        cv2.circle(out, (int(reading.at[0]), int(reading.at[1])), 9, colour, 2)
    cv2.putText(out, f"{reading.state} area={reading.area}", (xa, max(14, ya - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)
    return out
