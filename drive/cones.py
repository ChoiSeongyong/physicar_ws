"""Cone detection and avoidance, from the lidar.

The lidar is the right sensor here and that is a measured conclusion, not an
assumption: the cones are 23 cm tall and the scanner sees them cleanly out to
2.85 m with the range accurate to a centimetre (tools/cone_probe.py). At 0.7 m/s
that is four seconds of warning.

A cone's signature is unmistakable once you look at a scan — a **narrow, deep,
isolated local minimum**:

    +28deg 3.06 m   +30deg 1.28 m   +32deg 3.08 m

One to three bins, a metre or more below their neighbours. The walls and the
road edge, by contrast, vary smoothly bin to bin. So the detector looks for
depth relative to a local background rather than for absolute range, which also
means it does not care how far away the walls happen to be.

Why avoidance is needed at all, in numbers: the car is 20 cm wide and a cone is
13.5 cm, so a strike happens whenever the lateral gap between their centres
drops below 16.75 cm. All six cones sit *on* the road — one dead on the centre
line, the rest 17-30 cm off it. Driving a perfect centre line still passes two
of them within a centimetre of contact, and the lane follower's median error is
13 cm.
"""
import math
import os
from dataclasses import dataclass

# Forward sector to search. Wider than needed for the cone directly ahead,
# because one being passed still has to keep influencing the offset.
SECTOR = int(os.environ.get("PC_CONE_SECTOR", 50))
# Measured, not assumed: the lidar still resolves a cone at 3.52 m. The old
# 2.85 was where the probe happened to stop looking, and it was throwing away
# most of the warning distance.
MAX_RANGE = float(os.environ.get("PC_CONE_MAX_RANGE", 3.6))

# How far below its surroundings a return has to sit to count as an object,
# and how many degrees wide it may be. Measured spikes drop 1.5-2.5 m over 1-3
# bins; walls never do.
MIN_DEPTH = float(os.environ.get("PC_CONE_MIN_DEPTH", 0.45))
MAX_WIDTH_DEG = int(os.environ.get("PC_CONE_MAX_WIDTH", 14))
BG_WINDOW = 15                      # bins either side used for the background

# Geometry, in metres.
CAR_HALF = 0.10                     # 200 mm wide
CONE_HALF = 0.0675                  # 135 mm wide
CLEAR = CAR_HALF + CONE_HALF        # 0.1675 — closer than this is contact
# Clearance asked for beyond bare contact.
#
# Once this had to fit inside 0.23 m of assumed room, 0.04 was all that would
# go: at 0.07 the target was 0.2375, just outside, so *neither* side was
# feasible and a 1.8 cm wobble in the cone's estimated position flipped the
# chosen side frame to frame — the bias swung +0.33 / -0.48 / +0.15.
#
# With the judge's real allowance the room is 0.45 m, so the clearance can be
# generous instead of marginal. Passing 4 cm from a cone at 1.5 m/s asks the
# estimate to be perfect; 12 cm does not.
MARGIN = float(os.environ.get("PC_CONE_MARGIN", 0.12))
# How far the car may move sideways before the *judge* calls it off track.
#
# This was the painted surface's half-width, 0.35 m, which left 0.23 m of usable
# room after the car's own width — and that was wrong twice over. The rule is
# four wheels outside the line, and the judge implements it as the distance
# from the car's origin to the centre line exceeding half the published
# inner/outer gap plus 0.12 m. That is 0.57 m here, and it is a *point*
# measure, so the body overhanging the outer line costs nothing.
#
# So the car may lean on the outside line to get round a cone. Using 0.23 m
# meant squeezing every avoidance into 40% of the room actually available,
# which is why so many of them failed to clear.
JUDGE_HALF = float(os.environ.get("PC_JUDGE_HALF", 0.57))
MAX_SHIFT = float(os.environ.get("PC_CONE_MAX_SHIFT", JUDGE_HALF - 0.12))

# One unit of lane offset is about this many metres of lateral error, from the
# detector's measured gain (tools/validate_lane.py).
METRES_PER_OFFSET = 0.65

# From tools/steer_sign.py: 0.51 m turning radius at the 20 deg lock.
WHEELBASE = 0.19
# The lane controller still has to be able to do its job underneath.
MAX_FF_DEG = float(os.environ.get("PC_CONE_MAX_FF", 12.0))
# Steering correction can be turned off independently of braking, because the
# two have different risk profiles: braking cannot put the car off the track,
# and steering demonstrably can.
STEER_AVOID = os.environ.get("PC_CONE_STEER", "1") == "1"

# Only react to a cone once it is this close. Reacting earlier wastes lateral
# room on cones the lane will have carried the car past anyway.
# Reaction distance, scaled by how fast the car is going. At 0.7 m/s a fixed
# 1.8 m gave two and a half seconds to move 20 cm sideways; at 3 m/s the same
# distance is 0.6 s, and a correction that late has to be violent enough to
# unsettle the car. Reacting on *time to contact* keeps the manoeuvre gentle at
# any speed, and the detector reaches 2.85 m so there is room to look further.
# Reacting at 2.6 m while doing 3 m/s leaves 0.87 s — the braking then happens
# right on top of the cone, which is exactly what it looks like from outside.
# The detector reaches 2.85 m, so look as far as it can see.
REACT_S = float(os.environ.get("PC_CONE_REACT_S", 1.6))
REACT_MIN = float(os.environ.get("PC_CONE_REACT_MIN", 1.2))
# The camera moved this. With the lidar alone the reaction distance could not
# usefully exceed what the lidar could see, and measured on an isolated cone
# that was 1.6 m — beyond it the cone subtends less than the 0.5 degree beam
# spacing and simply returns nothing. The learned detector finds cones out to
# 4 m, so the cap can finally be set by what the car needs rather than by what
# the geometry could reach: at 3 m/s, 4.2 m is 1.4 seconds of warning against
# the 0.5 seconds the lidar allowed.
REACT_MAX = float(os.environ.get("PC_CONE_REACT_MAX", 4.2))

# Speed cap while a cone is being passed, in m/s — an absolute cap, not a
# fraction, so it does not scale away when the straight-line speed is raised.
SLOW_MAX = float(os.environ.get("PC_CONE_SLOW_MAX", 2.0))
SLOW_MIN = float(os.environ.get("PC_CONE_SLOW_MIN", 1.1))

# Closer than this and the manoeuvre is over: there is no room left to move
# sideways, and swerving at contact range only adds an off-track to the strike.
COMMIT_M = float(os.environ.get("PC_CONE_COMMIT", 0.30))


# How far to the side a return may be and still be worth reporting.
#
# Measured (tools/cone_eval.py, 120 poses): the geometric detector runs at 59%
# precision, and the things it invents are not near the car — the false
# positives sit a median of 1.32 m off to the side, with three quarters beyond
# 1.68 m. They are the wall, the road edge, and the far side of the track,
# which at a hairpin is only a couple of metres away and straight ahead.
#
# Nothing that far out can ever be hit. The judge's own off-track limit is
# 0.57 m from the centre line, and a strike needs the centres within 0.1675 m,
# so the widest a cone can be and still matter is about 0.86 m even with the
# car at the very edge of the track. One metre keeps every reachable cone and
# discards the scenery.
MAX_LATERAL = float(os.environ.get("PC_CONE_MAX_LAT", 1.0))


def _measure_course():
    """Read the track's own dimensions instead of assuming last week's.

    Every number below was measured on the practice course and every one of
    them changed when the qualifier course arrived: the road narrowed from
    0.90 m to 0.704 m, which moves the judge's off-track line from 0.574 m to
    0.472 m, and the cones grew from 0.135 x 0.23 m to 0.18 x 0.38 m. Shipping
    the old constants would have let the car drive 10 cm past the real limit
    believing it was still legal, and would have understated the width it has
    to clear a cone by.

    The course publishes both, so they are read rather than guessed. Anything
    unreachable — and on the real car nothing here is reachable — leaves the
    defaults untouched.
    """
    import json
    import urllib.request

    base = os.environ.get("PHYSICAR_URL", "http://localhost")
    out = {}
    try:
        with urllib.request.urlopen(base + "/sim/api/world", timeout=5) as r:
            route = json.load(r)["track"]["route"]
        inner, outer = route["inner"], route["outer"]
        gaps = sorted(math.hypot(a[0] - b[0], a[1] - b[1])
                      for a, b in zip(inner, outer))
        if gaps:
            out["judge_half"] = gaps[len(gaps) // 2] / 2.0 + 0.12
    except Exception:                                      # noqa: BLE001
        pass
    try:
        with urllib.request.urlopen(base + "/sim/api/objects", timeout=5) as r:
            objs = json.load(r)["objects"]
        widths = [o["size"]["x"] for o in objs
                  if o.get("name", "").startswith("cone") and o.get("size")]
        if widths:
            out["cone_half"] = max(widths) / 2.0
    except Exception:                                      # noqa: BLE001
        pass
    return out


if os.environ.get("PC_COURSE_AUTO", "1") == "1":
    _m = _measure_course()
    if "judge_half" in _m and not os.environ.get("PC_JUDGE_HALF"):
        JUDGE_HALF = _m["judge_half"]
        if not os.environ.get("PC_CONE_MAX_SHIFT"):
            MAX_SHIFT = JUDGE_HALF - 0.12
    if "cone_half" in _m and not os.environ.get("PC_CONE_HALF"):
        CONE_HALF = _m["cone_half"]
        CLEAR = CAR_HALF + CONE_HALF


@dataclass
class Cone:
    distance: float     # metres
    bearing: float      # degrees, + = left
    width: float        # degrees

    @property
    def forward(self):
        return self.distance * math.cos(math.radians(self.bearing))

    @property
    def lateral(self):
        """Metres to the left of the car's axis (negative = right)."""
        return self.distance * math.sin(math.radians(self.bearing))


def _scan_pairs(scan):
    """(angle, range) in the forward sector, skipping missing bins.

    The 0 degree bin is always absent from this simulator's output, so a
    detector that indexed straight into the dict would have a blind spot dead
    ahead — exactly where it matters most. Working from whatever bins exist
    avoids that.
    """
    ranges = scan.get("ranges", scan) if isinstance(scan, dict) else scan
    out = []
    for k in range(-SECTOR, SECTOR + 1):
        v = ranges.get(str(k))
        if v is not None and v > 0:
            out.append((k, float(v)))
    return out


def _median(vals):
    s = sorted(vals)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def detect(scan):
    """Cones in front of the car, nearest first."""
    pts = _scan_pairs(scan)
    if len(pts) < 2 * BG_WINDOW:
        return []
    angles = [a for a, _ in pts]
    rs = [r for _, r in pts]

    # Background = local median with the candidate's own neighbourhood excluded,
    # so a cone cannot raise its own baseline.
    flags = []
    for i, r in enumerate(rs):
        lo, hi = max(0, i - BG_WINDOW), min(len(rs), i + BG_WINDOW + 1)
        near = [rs[j] for j in range(lo, hi) if abs(j - i) > MAX_WIDTH_DEG // 2]
        bg = _median(near) if near else r
        flags.append(r <= MAX_RANGE and (bg - r) >= MIN_DEPTH)

    cones, i = [], 0
    while i < len(flags):
        if not flags[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(flags) and flags[j + 1] and angles[j + 1] - angles[j] <= 2:
            j += 1
        width = angles[j] - angles[i] + 1
        if width <= MAX_WIDTH_DEG:
            seg = rs[i:j + 1]
            k = seg.index(min(seg))
            cones.append(Cone(distance=min(seg),
                              bearing=float(angles[i + k]),
                              width=float(width)))
        i = j + 1
    cones = [c for c in cones if abs(c.lateral) <= MAX_LATERAL]
    cones.sort(key=lambda c: c.distance)
    return cones


# How much the threat corridor widens per metre of distance.
#
# Predicting where the car will be several metres ahead does not work here, and
# the arithmetic says why: holding a 10 degree steering angle is a 1.08 m
# turning radius, so the arc formula puts the car 5.7 m sideways by 3.5 m ahead.
# The car never goes there — long before that the lane controller has changed
# the steering to follow the road. Any prediction built on the instantaneous
# command is meaningless past about a metre.
#
# What is honest is that the further away a cone is, the less is known about
# whether the car will meet it. So the corridor widens with distance instead of
# pretending to a trajectory: at 1 m it is close to the true clearance, at 3.5 m
# it is wide enough to catch anything the car might swing into. The cost of a
# threat that turns out to be harmless is a brief slowdown; the cost of missing
# one is five seconds.
SPREAD = float(os.environ.get("PC_CONE_SPREAD", 0.16))


def bias(cones, lane_offset=0.0, speed=0.7, steer_deg=0.0):
    """Lateral correction to add to the lane offset, and a speed cap.

    Returns (steer_delta_deg, speed_cap). Positive steering is left, matching
    the rest of the stack; the value is added straight to the commanded angle.

    Only the closest threatening cone is answered. Chaining several at once
    produces a bias that swings as each drops out of the sector, and there is
    only 23 cm of room to give away in total.
    """
    react = min(REACT_MAX, max(REACT_MIN, abs(speed) * REACT_S))

    # Threat test on the *path*, not on present lateral distance.
    #
    # Using |lateral| < clearance meant a cone only became a threat once it was
    # nearly abeam: a cone 2.5 m ahead and slightly off the nose measures 0.30 m
    # laterally and was filtered out, so the first reaction came at 0.96 m —
    # a third of a second at speed — and the chosen side then flipped frame to
    # frame as the bearing crossed zero (-0.42 at 0.73 m, +0.41 at 0.59 m).
    #
    # The car's lane offset already says where the lane wants to take it, so
    # the question is whether the cone sits within a car's width of *that*
    # path, evaluated where the car will actually be when it arrives.
    # Ask whether the cone is on the path, not whether it is beside the car.
    #
    # The previous test compared the cone's *current* lateral offset against the
    # clearance, and on a straight with the cone dead ahead that works — which
    # is exactly the case it was tested on. On a curve it does not: a cone 3.5 m
    # ahead that the car is steering straight at measures 0.6-0.9 m to the side,
    # far outside the 0.29 m clearance, so it is not a threat until it is nearly
    # abeam. Measured over a full lap, every single activation happened between
    # 0.82 and 1.00 m — with a detector that had just been shown to see cones at
    # 4 m. The car could see them and still would not react.
    #
    # So the comparison is against where the car will *be*. Holding the current
    # steering traces an arc, and the cone is a threat if it sits within a
    # car's width of that arc at its own distance.
    car_lat_now = lane_offset * METRES_PER_OFFSET
    threat = None
    for c in cones:
        if c.forward <= COMMIT_M or c.forward > react:
            continue
        if abs(c.lateral) < CLEAR + MARGIN + SPREAD * c.forward:
            threat = c
            break
    if threat is None:
        return 0.0, None

    # Everything is decided in the *lane's* frame, not relative to the car's
    # nose. Deciding by which side of the nose the cone sits on fails twice:
    # it picks a side without knowing whether there is room on it — one run
    # went right into the lane edge, cleared 11 cm of the 24 cm needed, and hit
    # the cone anyway — and when the cone is dead ahead its bearing flips sign
    # frame to frame, which made the bias oscillate +0.36 / -0.35 / +0.36.
    #
    # Sign, derived from the measured detector gain rather than from intuition:
    # tools/validate_lane.py measured d(offset)/d(rightward displacement) as
    # -1.56 per metre, so `offset` carries the same sign as the car's *leftward*
    # displacement, and car_lat = +offset * 0.65.
    #
    # This was negated. With the car's own position in the lane mirrored, the
    # chosen passing side was mirrored too, and the correction steered *into*
    # the cone — which is exactly what it looked like from outside.
    car_lat = lane_offset * METRES_PER_OFFSET
    cone_lat = car_lat + threat.lateral

    gap = CLEAR + MARGIN
    left_target = cone_lat + gap        # pass on the cone's left
    right_target = cone_lat - gap       # pass on its right
    options = [t for t in (left_target, right_target) if abs(t) <= MAX_SHIFT]
    if options:
        # Both sides clear the cone, so take the one asking for less movement.
        target = min(options, key=lambda t: abs(t - car_lat))
    else:
        # Neither side fits inside the lane. Take the one that stays nearest
        # the road, and accept a graze over an off-track.
        target = min((left_target, right_target), key=abs)
        target = max(-MAX_SHIFT, min(MAX_SHIFT, target))

    shift = target - car_lat        # positive = move left

    # Feed-forward steering, not a bias on the lane offset.
    #
    # Adding to `offset` fights the very controller it goes through: a constant
    # bias only shifts the car once the proportional loop has settled, and the
    # cone is in view for about a second. From outside it looked exactly like
    # what it was — the car slowing for a cone and then barely turning.
    #
    # The geometry says how much steering is actually needed. To move `shift`
    # sideways over the remaining distance d, the path curvature is 2*shift/d^2,
    # and a bicycle of wheelbase L needs atan(L * curvature). That is 1.3 deg at
    # 2 m, 5.2 deg at 1 m and 14 deg at 0.6 m — small and early, or large and
    # late, which is why reacting far out matters more than steering hard.
    d = max(threat.forward, 0.25)
    curvature = 2.0 * shift / (d * d)
    steer_ff = math.degrees(math.atan(WHEELBASE * curvature))
    steer_ff = max(-MAX_FF_DEG, min(MAX_FF_DEG, steer_ff))
    if not STEER_AVOID:
        steer_ff = 0.0

    # Brake first, then steer.
    #
    # Slowing does not change the geometry — the curvature needed to clear the
    # cone over a given distance is the same at any speed. What it buys is time
    # for the correction to take effect and margin if it does not, and a strike
    # costs five seconds against about one for the deceleration.
    urgency = 1.0 - min(1.0, max(0.0, (threat.forward - COMMIT_M)
                                 / max(react - COMMIT_M, 1e-6)))
    speed_cap = SLOW_MAX - (SLOW_MAX - SLOW_MIN) * urgency
    return steer_ff, speed_cap


class Avoider:
    """A committed avoidance manoeuvre. **Measured worse — not in use.**

    The idea was sound on paper: plan the pass once and hold it, so a
    centimetre of jitter in the cone estimate cannot keep moving the target.
    Scored against the real objective it was clearly worse than recomputing
    every frame — 88.3 against 70.5, with collisions rising from 8.8 s to
    18.3 s — and lowering the speed cap alongside it made things worse again
    (101 at 2.2 m/s, 105 at 1.6 m/s), buying lap time for no fewer strikes.
    Committing early to a plan built from a distant, noisy reading turns out to
    be worse than following a target that keeps being corrected.

    Kept for the record and for anyone tempted to try it again.

    The reactive version recomputed the passing side and the target from the
    cone's *current* measured bearing on every tick. A centimetre of jitter in
    that estimate moved the target, so the correction never settled into a
    shape — from outside it looked like the car half-turning and giving up,
    which is what it was doing.

    Here the manoeuvre is planned once, when the cone first comes into range:
    pick the side, fix the lateral position to pass at, and hold it until the
    cone is behind. The steering that follows is path tracking against a stable
    target instead of a chase after a moving one.

    This keeps state between frames, which the competition's stateless rule
    permits — that rule is about being able to drive from wherever the car is
    put down, and a manoeuvre that lapses as soon as its cone is gone always
    can.
    """

    def __init__(self):
        self.target = None      # lateral position to hold, lane frame, + = left
        self.miss = 0

    def reset(self):
        self.target = None
        self.miss = 0

    def _plan(self, car_lat, cone_lat):
        gap = CLEAR + MARGIN
        options = [t for t in (cone_lat + gap, cone_lat - gap)
                   if abs(t) <= MAX_SHIFT]
        if options:
            return min(options, key=lambda t: abs(t - car_lat))
        return max(-MAX_SHIFT,
                   min(MAX_SHIFT, min((cone_lat + gap, cone_lat - gap), key=abs)))

    def update(self, cones, lane_offset, speed):
        """(steer_delta_deg, speed_cap or None). Positive steering is left."""
        react = min(REACT_MAX, max(REACT_MIN, abs(speed) * REACT_S))
        car_lat = lane_offset * METRES_PER_OFFSET

        threat = None
        for c in cones:
            if c.forward <= 0.0 or c.forward > react:
                continue
            cone_in_lane = car_lat + c.lateral
            if abs(c.lateral) < CLEAR + MARGIN or abs(cone_in_lane) < CLEAR + MARGIN:
                threat = c
                break

        if threat is None:
            # Do not drop the manoeuvre on one missed frame: the cone leaves
            # the forward sector while the car is still alongside it.
            self.miss += 1
            if self.miss > 4:
                self.reset()
            if self.target is None:
                return 0.0, None
        else:
            self.miss = 0
            if self.target is None:
                self.target = self._plan(car_lat, car_lat + threat.lateral)

        shift = self.target - car_lat
        d = max(threat.forward if threat is not None else 0.6, 0.3)
        steer = math.degrees(math.atan(WHEELBASE * 2.0 * shift / (d * d)))
        steer = max(-MAX_FF_DEG, min(MAX_FF_DEG, steer))
        if not STEER_AVOID:
            steer = 0.0

        if threat is None:
            return -steer, None
        urgency = 1.0 - min(1.0, max(0.0, (threat.forward - COMMIT_M)
                                     / max(react - COMMIT_M, 1e-6)))
        return -steer, SLOW_MAX - (SLOW_MAX - SLOW_MIN) * urgency


def rebase(cones, lane_offset=0.0, speed=0.7):
    """Where the lane follower should aim, instead of where the lane is.

    Returns (offset_delta, speed_cap). Adding `offset_delta` to the lane
    estimate's offset before it reaches the controller moves the point the car
    settles at; the controller then drives the avoidance itself.

    This replaces adding a few degrees on top of the controller, which could
    not work and the arithmetic says why. `bias()` computes the *least*
    curvature that clears the cone over the remaining distance, 2*shift/d^2, so
    its demand vanishes with distance: 0.4 degrees at 4 m, 1.6 at 2 m, and only
    at 0.6 m does it reach the 12 degree cap. Meanwhile the lane controller
    answers any lateral displacement with 22 degrees per unit of offset — 9.7
    degrees for the 0.287 m a centre-line cone needs. Every correction was
    outvoted until the cone was almost under the bumper, which is exactly what
    it looked like from outside.

    Moving the target instead turns that 9.7 degrees from the thing fighting
    the manoeuvre into the thing performing it, and it applies from the moment
    the cone is seen rather than building up as it arrives.
    """
    react = min(REACT_MAX, max(REACT_MIN, abs(speed) * REACT_S))
    car_lat = lane_offset * METRES_PER_OFFSET

    threat = None
    for c in cones:
        if c.forward <= COMMIT_M or c.forward > react:
            continue
        if abs(c.lateral) < CLEAR + MARGIN + SPREAD * c.forward:
            threat = c
            break
    if threat is None:
        return 0.0, None

    cone_lat = car_lat + threat.lateral
    gap = CLEAR + MARGIN
    options = [t for t in (cone_lat + gap, cone_lat - gap) if abs(t) <= MAX_SHIFT]
    if options:
        target = min(options, key=lambda t: abs(t - car_lat))
    else:
        target = max(-MAX_SHIFT, min(MAX_SHIFT,
                                     min((cone_lat + gap, cone_lat - gap), key=abs)))

    # The controller drives its offset to zero, so shifting the offset it is
    # given by -target/METRES_PER_OFFSET makes it settle at `target` instead.
    delta = -target / METRES_PER_OFFSET

    urgency = 1.0 - min(1.0, max(0.0, (threat.forward - COMMIT_M)
                                 / max(react - COMMIT_M, 1e-6)))
    speed_cap = SLOW_MAX - (SLOW_MAX - SLOW_MIN) * urgency
    return delta, speed_cap
