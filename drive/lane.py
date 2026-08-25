"""Lane estimation from a single camera frame — no state carried between frames.

Statelessness is a competition requirement, not a style choice: officials pick
the car up after an off-track and set it down somewhere else, and the logic has
to drive from wherever it lands. Every estimate here comes from the frame in
front of it.

The cue is the **road surface**, not the painted lines. That is the opposite of
the obvious choice and it was decided by measurement (tools/dump_corner.py):

* The white-line threshold claimed 6-16% of every row near the horizon. It was
  locking onto the wall/sky boundary and the far scenery, which look exactly
  like a bright low-saturation stripe. Following that walks the car off the
  road, and it is what ended the first driving attempt two seconds in.
* At a 90 degree corner the road occupies only the bottom third of the frame.
  Any fixed set of sample rows chosen from a straight — the previous design
  sampled 0.58-0.74 of image height — is looking at grass by the time the car
  reaches the first corner.
* The asphalt itself probes as a very tight HSV cluster (tools/probe_hsv.py:
  H 105-107, S 107-124, V 71-83) against green grass, a purple wall and blue
  sky. Nothing else in the scene is that colour.

So: mask the road, keep the blob the car is actually standing on, and read the
lane centre off it. The look-ahead row is then chosen *relative to how far the
road is visible*, which is what makes one set of numbers work on the straight
and in the hairpin — in a corner the road vanishes close to the car, so the
look-ahead automatically pulls in.
"""
import math
import os as _os
from dataclasses import dataclass

import cv2
import numpy as np

# Asphalt as it appears *in this simulator*. Measured, not guessed — but a
# hardcoded colour is exactly the thing that will not survive the venue's
# lighting or a different road surface, so it is only the fallback. The mask
# normally calibrates itself from the frame; see `_adaptive_range`.
ROAD_PRIOR_LO, ROAD_PRIOR_HI = (95, 80, 45), (120, 150, 105)

# The strip immediately in front of the bumper. While the car is on the road
# this is road by definition, which makes it a free, continuously-refreshed
# colour sample of whatever surface the car is actually driving on.
SEED_TOP, SEED_L, SEED_R = 0.90, 0.38, 0.62

# The dashed centre line. At the start pose it sits on the image centre with the
# car centred between the white edges, so it marks the middle of the driving
# lane rather than dividing two lanes.
#
# The hue here was wrong for a long time. Sampling the pixels that actually
# punch holes in the road mask — which is what paint does — puts the line at
# H≈18 with high saturation, while the range being used started at H=20 and
# caught 0.9% of it. That is why the centre line looked like unusable confetti
# and got demoted to a fallback: the measurement was of a broken threshold, not
# of the line.
YELLOW_LO, YELLOW_HI = (8, 100, 120), (26, 255, 255)

# Paint is never a driving surface, and it has to be said explicitly.
#
# Measured on one frame: the asphalt sits at S=113, V=72, while the white edge
# line is S=8, V=218 — no overlap at all. Without the exclusion the self
# calibrating range happily learns whichever of the two is under the bumper,
# and when the car edges out towards the line it learns the line: at 0.40 m
# from the centre the mask picked the white stripe as the road and the car
# followed it, with the visible depth collapsing from 0.97 to 0.24.
#
# That mattered because of the rule. Going off track needs all four wheels out,
# and the judge measures it as 0.574 m from the centre line, while the asphalt
# only reaches about 0.35 m — so a car is allowed to put two wheels over the
# line to get round something, and 0.22 m of legal room was unusable purely
# because the perception mistook the line for road.
WHITE_S_MAX = int(_os.environ.get("PC_WHITE_S_MAX", 60))
WHITE_V_MIN = int(_os.environ.get("PC_WHITE_V_MIN", 140))

# A row needs this much road in it to count as road at all.
MIN_RUN_FRAC = 0.03

# How wide a hole in the surface to treat as still being the same surface.
#
# This wants to be *small*. Road markings are laid out so that a continuous line
# separates things you must not cross and a dashed line separates things you
# may: bridging every gap merges the lane with the one beside it, and where the
# track runs alongside itself the car simply drifted across onto the wrong
# segment and carried on. Leaving continuous lines unbridged makes the
# connected component stop at them, which is exactly the rule the paint encodes.
# The dashed centre line still does not split the lane, because its gaps keep
# the two halves connected.
GAP_PX = int(_os.environ.get("PC_GAP_PX", 3))
CLOSE_W = int(_os.environ.get("PC_CLOSE_W", 3))


@dataclass
class LaneEstimate:
    offset: float      # lateral error at the look-ahead row, -1 (left) .. +1
    slope: float       # (far centre - near centre), same units: which way it bends
    view: float        # how far the road is visible, 0 (blocked) .. 1 (to horizon)
    rows_used: int
    source: str        # "road" | "seek" | "none"

    # The individual cues, before they were blended into `offset`, each in the
    # same units, plus how much each should be believed. `offset` is one fixed
    # recipe for combining them and that recipe is where the remaining failures
    # live: mid-corner the three disagree completely — at one measured frame the
    # fit said +0.20, reach said -0.31 and the chain said 0.00, and the chain,
    # holding 85% of the weight, carried the car straight on through an 89
    # degree turn. Reporting them separately lets a learned controller decide
    # which to trust instead of that fixed recipe.
    cue_fit: float = 0.0
    cue_reach: float = 0.0
    cue_chain: float = 0.0
    has_reach: float = 0.0
    has_chain: float = 0.0
    clipped: float = 0.0      # fraction of rows whose lane edges leave frame
    conf: float = 0.0         # belief the surface underfoot is a road

    @property
    def ok(self):
        return self.source == "road"


def _adaptive_range(hsv):
    """Learn the road's colour from the patch in front of the bumper.

    Self-calibrating rather than hardcoded, so the same code works under
    different lighting and on a different surface — the venue's road will not
    be this simulator's shade of grey, and a fixed threshold would simply stop
    finding it.

    Returns None when that patch is not one uniform surface, which is the case
    when the car has already left the road; the caller then falls back rather
    than happily calibrating itself onto grass.
    """
    h, w = hsv.shape[:2]
    patch = hsv[int(h * SEED_TOP):, int(w * SEED_L):int(w * SEED_R)]
    if patch.size < 600:
        return None
    # Denoise the *statistics* only. Blurring the image the mask is built from
    # softens the road edge into the grass and moves the estimated centre; this
    # patch is one flat surface, so smoothing it costs nothing.
    seed = cv2.medianBlur(patch, 5).reshape(-1, 3).astype(np.int16)
    # Drop the paint before learning anything from this patch.
    #
    # The seed is "road by definition" only while the car is fully on the road.
    # Edging out to use the margin the rules allow puts the bumper over the
    # white line, and the range then calibrates to the line — after which the
    # car follows the paint instead of the asphalt. Removing white here means
    # the worst case is too few pixels to calibrate from, which falls back to
    # the prior and finds the road, instead of confidently locking onto a
    # stripe 10 cm wide.
    keep = ~((seed[:, 1] < WHITE_S_MAX) & (seed[:, 2] > WHITE_V_MIN))
    if keep.sum() < 0.35 * len(seed):
        return None
    seed = seed[keep]
    med = np.median(seed, axis=0)
    mad = np.median(np.abs(seed - med), axis=0)

    # Saturation and value must be consistent; hue of a grey surface is not,
    # so it is only allowed to be a constraint on a surface that has a colour.
    if mad[1] > 30 or mad[2] > 30:
        return None
    # Scaling the tolerance off the spread alone is what made this *worse* than
    # a fixed threshold under sensor noise and blur: both inflate the spread,
    # the window opens, and the mask swallows the grass. Cap it.
    tol = np.clip(mad * 4, (8, 45, 45), (14, 70, 70))
    lo, hi = med - tol, med + tol
    if med[1] < 60:
        lo[0], hi[0] = 0, 180        # near-grey: hue carries no information
    return (np.clip(lo, 0, 255).astype(np.uint8),
            np.clip(hi, 0, 255).astype(np.uint8))


PRIOR = (np.array(ROAD_PRIOR_LO, np.uint8), np.array(ROAD_PRIOR_HI, np.uint8))
# A road that fills most of the lower frame is a mask that has escaped into the
# grass, not a very wide road: at the widest the measured surface is 0.70 m and
# the camera sees well past both edges.
BLEED_FRAC = 0.80

# Belief that the surface underfoot is a road now only *slows the car down*; it
# no longer overrides where the car aims.
#
# Letting it override was the single most expensive mistake in this file. At a
# hairpin the visible road is a thin sliver, so boundedness and paint-hole
# counts both collapse and the score fell below any threshold worth setting —
# while the car sat dead centre in its lane, 0.04 m from the route. The gate
# then handed the aim point to the recovery search, which pointed nearly
# straight on, and drove a perfectly placed car off the track. Every "it will
# not turn at the hairpin" and "it went backwards" symptom traced here.
#
# Disabling the override took the run from two thirds of a lap to two and a
# third laps, with median error falling from 0.31 m to 0.13 m. Set
# PC_CONF_MIN above 0 to re-enable the old behaviour for comparison.
CONF_MIN = float(_os.environ.get("PC_CONF_MIN", 0.0))

# How clipped the road has to get before the aim point starts handing over from
# fitted lane midpoints to "where the road reaches furthest". On the straight
# the near rows are always clipped, so the changeover has to start well above
# zero or every straight would be steered by reach.
# How many chained dashes before the centre line is trusted over the surface
# cues, and how much of the aim point it then owns.
CHAIN_MIN = int(_os.environ.get("PC_CHAIN_MIN", 3))
# How far off the lane blob a dash may sit and still count as on it (the paint
# is masked *out* of the surface, so it needs a little slack), and how strongly
# the chain prefers continuing straight over turning.
DASH_MIN_AREA = int(_os.environ.get("PC_DASH_MIN_AREA", 8))
DASH_ON_LANE = int(_os.environ.get("PC_DASH_ON_LANE", 15))
DASH_ALIGN = float(_os.environ.get("PC_DASH_ALIGN", 1.6))
CHAIN_W = float(_os.environ.get("PC_CHAIN_W", 0.85))
# The chain reaches full weight once it spans this fraction of image height.
# A chain that leads nowhere gets a say in proportion to how far it leads.
CHAIN_EXTENT = float(_os.environ.get("PC_CHAIN_EXTENT", 0.25))

# Visible road depth (as a fraction of image height) below which the lane counts
# as ending ahead rather than merely bending.
#
# Off by default. The idea is right — at the hairpin the lane genuinely ends and
# every other cue says "straight on" — but every threshold tried also fired on
# ordinary corners, where the continuation search then aimed at the wrong
# surface and lost the car at the *first* bend instead of the hairpin. Kept
# because the hairpin still needs solving; enable with PC_DEAD_END to
# experiment.
DEAD_END = float(_os.environ.get("PC_DEAD_END", 0.0))

# Visible-road-depth score below which the lane counts as ending ahead, and how
# big another patch of asphalt must be to be believed as its continuation.
# Measured at the hairpin: approaching it the score is 0.34, at the apex where
# the car goes soft it is 0.05.
CONT_VIEW = float(_os.environ.get("PC_CONT_VIEW", 0.12))
CONT_AREA = float(_os.environ.get("PC_CONT_AREA", 0.004))

CLIP_LO = float(_os.environ.get("PC_CLIP_LO", 0.30))
CLIP_HI = float(_os.environ.get("PC_CLIP_HI", 0.95))


def road_mask(img, adaptive=True):
    """Binary mask of drivable surface."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    rng = _adaptive_range(hsv) if adaptive else None

    not_white = ~((hsv[:, :, 1] < WHITE_S_MAX) & (hsv[:, :, 2] > WHITE_V_MIN))

    def build(r):
        m = cv2.inRange(hsv, r[0], r[1])
        m = cv2.bitwise_and(m, m, mask=not_white.astype(np.uint8))
        # Close along x more than y: the gaps to bridge are painted lines,
        # which run roughly across the image, not blobs.
        return cv2.morphologyEx(m, cv2.MORPH_CLOSE,
                                np.ones((3, CLOSE_W), np.uint8))

    if rng is None:
        return build(PRIOR)
    m = build(rng)
    if m[m.shape[0] // 2:].mean() / 255.0 <= BLEED_FRAC:
        return m
    # Bled. Try once with a tight window before giving up on self-calibration —
    # the prior is this simulator's colour and is the worse answer at the venue.
    mid = (rng[0].astype(np.int16) + rng[1]) // 2
    span = (rng[1].astype(np.int16) - rng[0]) // 4
    tight = build((np.clip(mid - span, 0, 255).astype(np.uint8),
                   np.clip(mid + span, 0, 255).astype(np.uint8)))
    if tight[tight.shape[0] // 2:].mean() / 255.0 <= BLEED_FRAC:
        return tight
    return build(PRIOR)


def _pick_blob(mask):
    """The road the car is on — or, if it is off the road, the road it can see.

    Returning the biggest visible patch when nothing touches the bottom of the
    frame is what gives the controller something to steer *towards* after an
    excursion, instead of a blind fixed-rate turn.
    """
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return None, False
    h = mask.shape[0]
    bottom = labels[int(h * 0.97):]
    touching = [i for i in range(1, n) if np.any(bottom == i)]
    if touching:
        best = max(touching, key=lambda i: stats[i, cv2.CC_STAT_AREA])
        return labels == best, True
    best = max(range(1, n), key=lambda i: stats[i, cv2.CC_STAT_AREA])
    if stats[best, cv2.CC_STAT_AREA] < mask.size * 0.01:
        return None, False
    return labels == best, False


def _spans(blob, anchor):
    """Per-row (y, left, right) of the run nearest `anchor`, bottom row upward.

    Tracking the run nearest the previous row rather than the widest one keeps a
    corner — where the road ahead and the road alongside both appear in the same
    row — from being read as one very wide straight.
    """
    h, w = blob.shape
    out = []
    for y in range(h - 1, -1, -2):
        cols = np.flatnonzero(blob[y])
        if len(cols) < w * MIN_RUN_FRAC:
            if out and h - 1 - y > h * 0.05:
                break          # road genuinely ends here
            continue
        splits = np.flatnonzero(np.diff(cols) > GAP_PX)
        starts = np.concatenate(([0], splits + 1))
        ends = np.concatenate((splits, [len(cols) - 1]))
        runs = [(int(cols[s]), int(cols[e])) for s, e in zip(starts, ends)]
        runs = [r for r in runs if r[1] - r[0] + 1 >= w * MIN_RUN_FRAC]
        if not runs:
            continue
        a, b = min(runs, key=lambda r: abs((r[0] + r[1]) / 2 - anchor))
        out.append((y, a, b))
        anchor = (a + b) / 2
    return out


def _centre(span, w, yellow_x):
    """Lane centre for one row span, preferring the centre line when it is there."""
    a, b = span[1], span[2]
    mid = (a + b) / 2
    # An edge sitting on the frame border is a crop, not a road edge, so the
    # midpoint of the visible part is biased. The centre line, when visible,
    # is immune to that.
    if yellow_x is not None and a - 2 <= yellow_x <= b + 2:
        clipped = a <= 1 or b >= w - 2
        return yellow_x if clipped else 0.5 * mid + 0.5 * yellow_x
    return mid


def _reach(blob, h, w):
    """Column where the road reaches furthest ahead, normalised to pixels.

    At a square corner the road runs off the side of the frame, so every
    sampled row is clipped by the image border and every midpoint lands near
    the image centre — the detector confidently commands "straight on" and the
    car drives across the corner into the grass. Measured against true pure
    pursuit, that was 15% of corner poses steered the wrong way and 24% steered
    at less than half the angle needed.

    Where the road *reaches furthest* does not have that failure: the drivable
    surface extends deepest into the frame in the direction the track goes,
    whether or not its edges are in view.
    """
    top = np.full(w, float(h))
    ys, xs = np.nonzero(blob)
    if not len(xs):
        return None
    np.minimum.at(top, xs, ys.astype(float))
    # Smooth, or a single stray pixel decides where the car goes.
    k = max(3, w // 20)
    sm = np.convolve(top, np.ones(k) / k, mode="same")
    sm[:k] = sm[-k:] = h
    best = sm.min()
    if best >= h:
        return None
    # Centroid of the columns that get within a few pixels of the deepest, so a
    # broad opening is aimed at down its middle rather than at one edge.
    cols = np.flatnonzero(sm <= best + max(2.0, h * 0.02))
    return float(cols.mean()) if len(cols) else None


def _dash_chain(hsv, h, w, blob=None):
    """Follow the dashed centre line forward, dash by dash.

    The road surface says where the car *may* drive. It does not say where the
    car *should* — and at a wide junction, or where the track runs back
    alongside itself, or across the mouth of a U-shaped section, everything in
    view is equally drivable. Steering at the deepest reachable asphalt cuts
    straight across the U and, once off the racing line, happily picks up the
    neighbouring stretch and follows it backwards. Both were observed.

    The centre line is the missing information: it marks the lane, and chaining
    from the dash nearest the car outward follows the branch the car is
    actually on rather than the one that happens to be furthest away.

    Returns the chain bottom-to-top in image coordinates, or None.
    """
    m = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
    n, _, stats, cent = cv2.connectedComponentsWithStats(m, 8)
    # Distant dashes are only a couple of pixels. Requiring 8 threw away every
    # marking at the far U-section, where the largest dash measured 21 px and
    # most were 1-3, so the chain simply did not exist exactly where it was
    # needed most.
    pts = [(float(cent[i][0]), float(cent[i][1])) for i in range(1, n)
           if stats[i, cv2.CC_STAT_AREA] >= DASH_MIN_AREA]

    # Only dashes lying on the lane the car is actually on. Without this the
    # chain happily jumps to a stretch of centre line belonging to another part
    # of the track: at the first corner it linked onto the dashes running
    # straight ahead beyond the bend and drove into the wall. The lane's own
    # markings are the ones that describe where the lane goes.
    if blob is not None:
        near = cv2.dilate(blob.astype(np.uint8),
                          np.ones((DASH_ON_LANE, DASH_ON_LANE), np.uint8))
        pts = [p for p in pts if near[min(h - 1, int(p[1])),
                                      min(w - 1, int(p[0]))] > 0]
    if len(pts) < 2:
        return None

    # "Further along" has to mean further *from the car*, not higher up the
    # image. At a square corner the next dash is off to the side and can sit at
    # the same image row or below it, so a rule that only accepts dashes higher
    # in the frame throws the entire turn away and leaves the chain pointing at
    # whatever lies straight ahead — which at the hairpin is the far side of the
    # track, across the grass. That is exactly the failure that kept ending runs
    # at the same corner.
    ox, oy = w / 2.0, float(h)
    def _from_car(p):
        return ((p[0] - ox) ** 2 + (p[1] - oy) ** 2) ** 0.5

    start = min(pts, key=_from_car)
    chain, used = [start], {start}
    step_cap = w * 0.40
    while len(chain) < 14:
        cur = chain[-1]
        if len(chain) >= 2:
            dx, dy = cur[0] - chain[-2][0], cur[1] - chain[-2][1]
        else:
            dx, dy = cur[0] - ox, cur[1] - oy      # outward from the car
        dnorm = (dx * dx + dy * dy) ** 0.5 or 1.0
        best, best_c = None, None
        for p in pts:
            if p in used or _from_car(p) <= _from_car(cur) + 1.0:
                continue                     # must lead further from the car
            vx, vy = p[0] - cur[0], p[1] - cur[1]
            d = (vx * vx + vy * vy) ** 0.5
            if d > step_cap:
                continue
            # Prefer the dash that continues the line's direction, so the chain
            # does not hop onto a parallel stretch of the same colour.
            align = (vx * dx + vy * dy) / (dnorm * d + 1e-6)
            c = d * (DASH_ALIGN - align)
            if best_c is None or c < best_c:
                best, best_c = p, c
        if best is None:
            break
        chain.append(best)
        used.add(best)
    return chain if len(chain) >= 3 else None


def _bounded(spans, w):
    """Fraction of sampled rows whose road span ends inside the frame.

    This is what tells the road from the lawn, and it is the one thing the
    self-calibrating mask cannot work out from colour: grass is every bit as
    uniform as asphalt, so on the lawn the mask happily re-calibrates onto the
    lawn and reports a confident lane pointing away from the track.

    A road is laterally bounded within the camera's view; an open field is not.
    Measured over 81 poses (tools/road_features.py) this separates the two at
    0.91 — far better than area, depth, how fast the surface narrows, or how
    much bright paint borders it — and it refers to no particular colour, so it
    should hold at a venue that looks nothing like this simulator.

        on the road   p10 0.388   median 0.604
        on the grass  median 0.093   p90 0.297
    """
    return float(np.mean([1.0 if (a > 2 and b < w - 3) else 0.0
                          for _, a, b in spans]))


def _confidence(hsv, blob, spans, w):
    """How much this surface looks like a road rather than open ground. 0..1.

    No single feature is enough. Measured over 106 poses around the lap
    (tools/road_features.py), the best separations between standing on the road
    and standing on the lawn were:

        bright holes in the surface  0.79     road markings are a different
        holes in the surface         0.76     colour, so they punch holes in
                                              the mask; a lawn has none
        bright border                0.62     paint runs along the edges
        laterally bounded            0.60     a road is enclosed in view,
                                              an open field is not

    An earlier version of this trusted boundedness alone, which scored 0.91 on
    twelve poses and 0.60 on sixteen — the first number was luck. Combining the
    three independent ones and treating the result as a degree of belief, rather
    than betting the run on one threshold, is what that mistake argues for.

    Deliberately colour-free: every term is relative to the surface's own
    brightness or is purely geometric, so none of it encodes "asphalt is grey"
    or "grass is green".
    """
    v = hsv[..., 2]
    v_surface = float(np.median(v[blob]))
    area = max(int(blob.sum()), 1)

    b8 = blob.astype(np.uint8) * 255
    filled = cv2.morphologyEx(b8, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
    holes = (filled > 0) & ~blob
    hole_px = v[holes]
    hole_bright = (float((hole_px > v_surface + 30).sum()) / area
                   if len(hole_px) else 0.0)

    ring = cv2.dilate(b8, np.ones((9, 9), np.uint8)) - b8
    ring_px = v[ring > 0]
    bright_ring = float((ring_px > v_surface + 30).mean()) if len(ring_px) else 0.0

    # Each term is scored against the median seen on the road, so 1.0 means
    # "as road-like as a typical on-track frame".
    return float(np.mean([
        min(1.0, hole_bright / 0.030),
        min(1.0, _bounded(spans, w) / 0.48),
        min(1.0, max(0.0, (bright_ring - 0.45) / 0.36)),
    ]))


def _other_surface(mask, blob, h, w):
    """Centre of the most road-like surface that is *not* the one underfoot.

    If the car is standing on grass, the road is still in frame — it is simply
    what the mask rejected. This gives the controller somewhere to aim instead
    of a blind fixed-rate turn.

    Candidates are *ranked* by how enclosed they look rather than filtered by
    it: seen from off the track the road usually runs out of one side of the
    frame, so demanding it be fully enclosed rejects the very thing we are
    looking for. That bug made this return nothing at all every time it was
    called.
    """
    # Only surfaces the mask *rejected*. Widening this to also consider other
    # components of the mask itself was tried, to reach the continuation at a
    # hairpin: it made the recovery aim worse everywhere else and the car was
    # lost at the first bend instead of the hairpin (lap progress 50% -> 24%).
    other = cv2.bitwise_not(mask)
    other[:int(h * 0.42)] = 0          # sky and wall are never drivable
    if blob is not None:
        other[blob] = 0
    other = cv2.morphologyEx(other, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, labels, stats, cent = cv2.connectedComponentsWithStats(other, 8)
    best = None
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < mask.size * 0.015:
            continue
        sp = _spans(labels == i, cent[i][0])
        if len(sp) < 4:
            continue
        rank = (_bounded(sp, w), int(stats[i, cv2.CC_STAT_AREA]))
        if best is None or rank > best[0]:
            best = (rank, float(cent[i][0]))
    return best[1] if best else None


def _lane_continuation(mask, blob, h, w):
    """Where the road picks up again, when the lane underfoot is about to end.

    Only other components of the *road mask* — deliberately narrow. An earlier
    version searched everything the mask rejected, which at a hairpin is the
    grass, and a version that searched both made the recovery aim worse
    everywhere and cost more than it gained.

    At the hairpin the car steers correctly into the bend (offset -0.64) and
    then goes soft at the apex (-0.32) because its own lane has run out and
    there is nothing left to fit. The continuation is sitting in plain view as a
    separate patch of asphalt across the grass on the inside of the bend.
    """
    src = mask.copy()
    src[blob] = 0
    src[:int(h * 0.40)] = 0            # sky and wall are never drivable
    src = cv2.morphologyEx(src, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, stats, cent = cv2.connectedComponentsWithStats(src, 8)
    best = None
    for i in range(1, n):
        a = int(stats[i, cv2.CC_STAT_AREA])
        if a < mask.size * CONT_AREA:
            continue
        if best is None or a > best[0]:
            best = (a, float(cent[i][0]))
    return best[1] if best else None


def _yellow_at(img, blob, y, band):
    """Median x of centre-line pixels near row `y`, inside the road blob."""
    h, w = blob.shape
    lo, hi = max(0, y - band), min(h, y + band + 1)
    hsv = cv2.cvtColor(img[lo:hi], cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
    # Dilate the blob down-band so the line, which sits *on* the road but is
    # masked out of it, still counts as inside.
    near = cv2.dilate(blob[lo:hi].astype(np.uint8), np.ones((5, 5), np.uint8))
    xs = np.flatnonzero((m > 0).any(axis=0) & (near > 0).any(axis=0))
    return float(np.median(xs)) if len(xs) >= 3 else None


def detect(img, lookahead=0.72):
    """Estimate the lane from one BGR frame.

    `lookahead` is a fraction of the *visible* road depth, not of the image. On
    a straight the road runs to the horizon and the aim point is far; in a
    corner the road vanishes a metre ahead and the aim point comes with it.
    """
    h, w = img.shape[:2]
    mask = road_mask(img)
    blob, under_bumper = _pick_blob(mask)
    if blob is None:
        return LaneEstimate(0.0, 0.0, 0.0, 0, "none")

    if not under_bumper:
        # The car is not standing on any surface we recognise. Steer at the
        # centroid of the largest one we can see.
        _, xs = np.nonzero(blob)
        return LaneEstimate(offset=float((xs.mean() - w / 2) / (w / 2)),
                            slope=0.0, view=0.0, rows_used=0, source="seek")

    spans = _spans(blob, w / 2)
    if len(spans) < 4:
        return LaneEstimate(0.0, 0.0, 0.0, len(spans), "none")

    conf = _confidence(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), blob, spans, w)
    if conf < CONF_MIN:
        # Standing on something that does not look like a road — a lawn, or the
        # apron beside the track. If a more road-like surface is in frame, go to
        # it. Otherwise keep using this one, slowly: a false alarm that stops
        # the car costs the whole run, while a false alarm that only slows it
        # costs a second.
        target = _other_surface(mask, blob, h, w)
        if target is not None:
            return LaneEstimate(offset=float((target - w / 2) / (w / 2)),
                                slope=0.0, view=0.0, rows_used=0, source="seek")

    depth = (h - 1 - spans[-1][0]) / h          # how far ahead we can see

    # Fit the lane centre over every visible row rather than reading it off two.
    # Two-row differencing put single-row noise straight into the curvature
    # term, which is multiplied by the largest gain in the controller: the car
    # sawed at full lock and eventually sawed itself off the track.
    #
    # Rows whose span runs off the side of the frame are dropped when there are
    # enough that do not — a clipped span's midpoint is pulled towards the
    # middle of the image and is not the middle of the road.
    inside = [s for s in spans if s[1] > 2 and s[2] < w - 3]
    fit_on = inside if len(inside) >= 5 else spans
    ys = np.array([s[0] for s in fit_on], float)
    xs = np.array([(s[1] + s[2]) / 2 for s in fit_on], float)

    # Both aim points come from rows that were actually fitted, so the curve is
    # never evaluated outside the data it was built from. Taking the near point
    # from the full span list instead let the quadratic extrapolate into the
    # clipped bottom rows and report bends of ±3 — into a term with gain 14.
    far = fit_on[min(len(fit_on) - 1, int(len(fit_on) * lookahead))]
    near = fit_on[min(len(fit_on) - 1, int(len(fit_on) * lookahead * 0.4))]
    if len(fit_on) >= 8 and np.ptp(ys) > h * 0.05:
        # Quadratic: a road bends, and forcing a straight line through a bend
        # biases the aim point towards the outside of the corner.
        coef = np.polyfit(ys, xs, 2)
        cf, cn = float(np.polyval(coef, far[0])), float(np.polyval(coef, near[0]))
    else:
        cf, cn = _centre(far, w, None), _centre(near, w, None)

    # As the lane edges leave the frame, the fitted midpoints stop meaning
    # anything and the aim point hands over to where the road reaches furthest.
    # Blended rather than switched, so a corner does not arrive as a step change
    # in the command.
    cue_fit = cf                      # before any blending
    cue_reach = cue_chain = None

    clipped = 1.0 - _bounded(spans, w)
    reach = _reach(blob, h, w)
    cue_reach = reach
    if clipped > CLIP_LO:
        if reach is not None:
            k = min(1.0, (clipped - CLIP_LO) / (CLIP_HI - CLIP_LO))
            cf = (1.0 - k) * cf + k * reach
            cn = (1.0 - k) * cn + k * (0.5 * reach + 0.5 * cn)

    # Dead end: the lane runs out within a metre or so. At a hairpin the road
    # the car is on simply stops, the continuation is off to one side separated
    # by the grass on the inside of the bend, and every cue above — fitted
    # midpoints, reach, the dashes on this lane — describes a lane that ends,
    # so they all agree on "straight ahead" and the car drives off the end.
    #
    # This is the same search the recovery uses, run one step earlier: while
    # still on the road rather than after leaving it. Getting there first is the
    # whole point; recovery from the grass at this corner did steer the right
    # way, several metres too late.
    if depth < DEAD_END:
        ahead = _other_surface(mask, blob, h, w)
        if ahead is not None:
            cf = cn = ahead

    # The centre line, chained dash to dash, is the only cue that says which way
    # the *route* goes rather than merely where asphalt is. Where it is readable
    # it outranks everything above, because the failures the surface cues cannot
    # see — cutting the mouth of a U-section, and rejoining the neighbouring
    # stretch backwards — are both cases where the surface is perfectly good and
    # simply belongs to a different part of the lap.
    # How nearly the lane has run out. Everything below keys off this.
    raw_view = float(np.clip((depth - 0.30) / 0.16, 0.0, 1.0))

    # Normally the chain may only use dashes lying on the lane the car is on,
    # which is what stops it hopping onto a parallel stretch. But when the lane
    # is ending, its own dashes are exactly the ones that point off the end, and
    # the markings that matter are on the continuation across the gap. So drop
    # the restriction precisely then, and let the chain's own preference for
    # continuing its direction pick which continuation.
    dead_end = raw_view < CONT_VIEW
    chain = _dash_chain(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), h, w,
                        None if dead_end else blob)
    if chain is not None and len(chain) >= CHAIN_MIN:
        i_far = min(len(chain) - 1, max(1, int((len(chain) - 1) * lookahead)))
        i_near = min(len(chain) - 1, max(1, int((len(chain) - 1) * lookahead * 0.4)))

        # Weighting the chain by how far it physically leads was tried and
        # reverted: it did not rescue the frame it was aimed at (the chain
        # there spans a long way and still points straight on) and it cost
        # accuracy everywhere else — the offline aim score went 0.795 -> 0.993.
        cue_chain = chain[i_far][0]
        cf = CHAIN_W * chain[i_far][0] + (1 - CHAIN_W) * cf
        cn = CHAIN_W * chain[i_near][0] + (1 - CHAIN_W) * cn
    else:
        # Not enough dashes to chain. Fall back to the single nearest reading,
        # which still pins the aim point to the lane middle on a straight.
        yf = _yellow_at(img, blob, far[0], band=max(3, h // 40))
        if yf is not None and far[1] - 2 <= yf <= far[2] + 2:
            cf = 0.5 * cf + 0.5 * yf

    # The lane is running out and nothing left in it says which way to go. Hand
    # the aim point over to the next stretch of road, in proportion to how
    # nearly it has ended, so an ordinary bend is untouched and only a genuine
    # dead end swings the car.
    if dead_end and chain is None:
        cont = _lane_continuation(mask, blob, h, w)
        if cont is not None:
            k = 1.0 - raw_view / CONT_VIEW
            cf = (1.0 - k) * cf + k * cont
            cn = cf

    return LaneEstimate(
        offset=float((cf - w / 2) / (w / 2)),
        slope=float((cf - cn) / (w / 2)),
        # Raw depth only ranges about 0.38 (hairpin) to 0.47 (straight) — in a
        # corner you still see road, just the road across the corner. Stretch
        # that narrow band so it can actually drive the speed law. An unbounded
        # surface reports no view at all, which pins the car to its slowest
        # speed for as long as it is unsure what it is driving on.
        # Scaled by belief as well as by depth, so an uncertain surface is
        # driven slowly even when the fallback found nothing better to aim at.
        view=float(np.clip((depth - 0.30) / 0.16, 0.0, 1.0) * conf),
        rows_used=len(spans),
        source="road",
        cue_fit=float((cue_fit - w / 2) / (w / 2)),
        cue_reach=(0.0 if cue_reach is None
                   else float((cue_reach - w / 2) / (w / 2))),
        cue_chain=(0.0 if cue_chain is None
                   else float((cue_chain - w / 2) / (w / 2))),
        has_reach=0.0 if cue_reach is None else 1.0,
        has_chain=0.0 if cue_chain is None else 1.0,
        clipped=float(clipped),
        conf=float(conf))


def annotate(img, est, lookahead=0.72):
    """Draw what the detector used, for the live view and for saved frames."""
    out = img.copy()
    h, w = out.shape[:2]
    blob, on_road = _pick_blob(road_mask(img))
    if blob is not None:
        out[blob] = (0, 110, 0)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    out[cv2.inRange(hsv, YELLOW_LO, YELLOW_HI) > 0] = (0, 255, 255)
    cv2.line(out, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)

    if blob is not None and on_road:
        spans = _spans(blob, w / 2)
        for y, a, b in spans[::4]:
            cv2.line(out, (a, y), (b, y), (255, 120, 0), 1)
        if len(spans) >= 4:
            i = min(len(spans) - 1, int(len(spans) * lookahead))
            y = spans[i][0]
            cv2.circle(out, (int(w / 2 + est.offset * w / 2), y), 6, (0, 0, 255), -1)
    cv2.putText(out, f"{est.source} n={est.rows_used} off={est.offset:+.2f} "
                     f"sl={est.slope:+.2f} view={est.view:.2f}",
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                cv2.LINE_AA)
    return out
