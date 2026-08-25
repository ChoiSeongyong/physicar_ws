"""Lane-following control law.

Steering is pure pursuit expressed in image space: aim at the lane centre a
fixed distance ahead and turn in proportion to how far off it is. The gain is
not a knob that was twiddled until it looked right — tools/steer_sign.py
measured the car's response as 3.40 deg/s of yaw per degree of steering at
0.6 m/s, perfectly linear, saturating at exactly 20 degrees. That implies a
wheelbase near 0.19 m and a 0.51 m turning radius at full lock, which puts the
geometrically correct look-ahead gain at about 22 deg per unit of offset.

Speed is where lap time actually comes from. Scoring is `time + penalties` over
a single lap, so a controller holding one safe speed everywhere loses to one
that opens up on the straights — as long as it slows enough not to collect
+5 s off-track penalties. The car brakes on three cues, and takes the most
pessimistic: how far the road is visible ahead, how hard the lane is bending,
and how much steering is being asked for right now. The first two are
predictive, which is what lets it slow *before* the corner rather than during.
"""
import os

from dataclasses import dataclass


@dataclass
class Gains:
    # These are the values that actually completed laps, settled by driving —
    # not by the offline pose score. The offline optimum was 28/16, and driven
    # in closed loop it circled on the spot: a static score cannot see
    # oscillation. Whatever a sweep says, gains get confirmed on the car.
    steer_p: float = 22.0       # deg per unit of look-ahead offset (see above)
    steer_d: float = 14.0       # deg per unit of lane bend (curvature feed-forward)
    steer_limit: float = 20.0   # measured hardware lock; beyond this is a no-op

    # Set by scoring runs, not by caution. 0.70 was chosen while "off track"
    # was being judged against the 0.35 m painted half-width; the official rule
    # is half the gap between the published inner and outer edges plus 0.12 m,
    # which is 0.57 m — 1.6x more room than assumed. Swept against the real
    # score, raising the cap cut the lap from 65.7 s to 33.8 s while penalties
    # went *down* (20-25 s to 10-15 s), taking the score from 84 to ~45.
    #
    # 3.0 is also the car's ceiling: commanding 4.0 still tops out at 3.15 m/s.
    speed_max: float = 3.00     # straight-line speed, m/s
    speed_min: float = 0.42     # through the tightest hairpin
    view_floor: float = 0.35    # below this much visible road, run at speed_min
    corner_gain: float = 1.30   # how sharply steering demand cuts speed
    bend_gain: float = 1.60     # how sharply lane bend cuts speed

    seek_speed: float = 0.35    # crawl while recovering off-road
    seek_steer: float = 45.0    # how hard to turn towards the road we can see
    seek_floor: float = 12.0    # minimum turn once it is clear which way to go
    seek_dead: float = 0.08     # ...but go straight when the road is dead ahead


# How much the steering gain rises as the road ahead disappears.
#
# The aim point sits at a fixed fraction of the *visible* road depth, which is
# the right idea on a straight and backwards in a corner. Measured at the two
# places the car keeps leaving the track: the course turns 102 degrees at 63%
# of the lap and 95 degrees at 80%, and the controller asked for 11.9 and 9.4
# degrees against a 20 degree lock. The car can make both — its minimum radius
# is 0.51 m — but it was never told to.
#
# The cause is a feedback loop in the wrong direction. A sharp corner hides the
# road, so `view` collapses to about 0.37; a low `view` pulls the aim point in
# close; the road has barely bent that near the car, so `offset` comes out
# small; and small offset means small steering. The tighter the corner, the
# less the controller asks for.
#
# So the gain is scaled up as visibility falls, which is the one moment it is
# certain a corner is there. On a clear straight (view near 1) nothing changes.
VIEW_BOOST = float(os.environ.get("PC_VIEW_BOOST", 1.6))
VIEW_REF = float(os.environ.get("PC_VIEW_REF", 0.75))


def _view_gain(view, g):
    """Multiplier on the steering demand, 1.0 with the road wide open."""
    if view >= VIEW_REF:
        return 1.0
    t = max(0.0, min(1.0, (VIEW_REF - view) / VIEW_REF))
    return 1.0 + (VIEW_BOOST - 1.0) * t


def steering(est, g=Gains()):
    """Steering angle in degrees. Positive is left — measured, not assumed."""
    # The minus sign: a lane centre seen to the right of the image centre is a
    # positive offset, and reaching it means turning right, which is negative.
    s = -(g.steer_p * est.offset + g.steer_d * est.slope)
    s *= _view_gain(est.view, g)
    return max(-g.steer_limit, min(g.steer_limit, s))


def speed(est, steer_deg, g=Gains()):
    """Slowest of the three brake cues."""
    # How far we can see. Predictive: the road stops being visible some way
    # ahead of a corner, while the car is still on the straight.
    v_view = g.speed_min + (g.speed_max - g.speed_min) * _ramp(est.view, g.view_floor)
    # How hard the lane bends between the near and far aim points. Also
    # predictive, and it leads the steering demand by roughly a car length.
    v_bend = _cut(abs(est.slope) * g.bend_gain, g)
    # What the car is being asked to do right now. Catches everything the two
    # predictive cues miss, including a bad recovery.
    v_steer = _cut(abs(steer_deg) / g.steer_limit * g.corner_gain, g)
    return min(v_view, v_bend, v_steer)


def _ramp(x, floor):
    if x <= floor:
        return 0.0
    return (x - floor) / (1.0 - floor)


def _cut(load, g):
    return g.speed_max - (g.speed_max - g.speed_min) * min(1.0, max(0.0, load))


def command(est, g=Gains(), last_steer=0.0):
    """(speed, steering_deg) for one tick.

    `last_steer` is the angle commanded on the previous tick. It is used
    only when the frame contains nothing recognisable, and it is what
    carries the car through the tight U-section — see below.
    """
    if est.source == "seek":
        # Off the road, but road is visible somewhere in frame. Crawl towards
        # it. A stopped car scores nothing and officials only reposition on
        # request, so recovering under its own power is worth a lot.
        #
        # Committed, not proportional. A gentle proportional nudge produced 6
        # degrees of steering while the car rolled on at a third of a metre per
        # second: it drifted further out than it turned back, and never
        # recovered. At full lock the turning circle is half a metre.
        if abs(est.offset) < g.seek_dead:
            return g.seek_speed, 0.0
        turn = max(g.seek_floor, min(g.steer_limit, g.seek_steer * abs(est.offset)))
        return g.seek_speed, -turn if est.offset > 0 else turn
    if not est.ok:
        # Nothing recognisable at all. Hold the turn already being made
        # rather than straightening up.
        #
        # Creeping straight was the old answer, and on a straight it is
        # the right one — the road comes back within a metre. In a corner
        # it walks the car off the track, and a corner is where sight is
        # actually lost. Traced through the U-section: visible depth fell
        # to zero, the estimate came back as "none" with its offset
        # defaulting to 0.0, the controller read that as the road being
        # dead ahead, and the car drove on with the steering at exactly
        # zero while its distance from the centre line grew 0.13 -> 0.45 m
        # and kept growing. An offset of zero there did not mean centred;
        # it meant nothing had been measured.
        #
        # Holding the last angle costs nothing on a straight, where it is
        # already near zero, and in a corner it carries the turn through
        # the blind patch. It is one tick of memory, which the stateless
        # rule allows: that rule asks the car to be able to drive from
        # wherever it is put down, and a car starting here simply begins
        # with zero.
        return g.seek_speed, max(-g.steer_limit,
                                 min(g.steer_limit, last_steer))
    s = steering(est, g)
    return speed(est, s, g), s


# Reversing the steering costs a confirmation. Measured at the hairpin.
#
# Traced through lap 63-65% at speed, the car was not too fast and it was not
# under-steering: it sat at 0.4 m/s and alternated between the two locks —
# -20.0, -20.0, +20.0, +20.0, -20.0 — while its distance from the centre line
# grew 0.18 -> 0.95 m and the lap made no progress for two seconds. It sawed
# itself sideways off the track.
#
# The cause is that the estimator has two branches that disagree by sign. With
# the car half on the road the "is there road under the bumper" test flickers
# frame to frame, and each answer produces the opposite command: on the road
# the offset reads +0.67 and asks for -20, off it the recovery reads -0.12 and
# asks for +12. Neither is wrong about what it sees; they simply cannot both
# be acted on at 15 Hz.
#
# So a reversal has to be meant. A large command may not flip sign until the
# new direction has survived FLIP_FRAMES frames — 0.2 s, short enough that a
# real S-bend is unaffected, long enough that a single ambiguous frame cannot
# throw the wheel across. Small commands are left alone: the problem is the
# full-lock sawing, not ordinary centring.
FLIP_GUARD_DEG = float(os.environ.get("PC_FLIP_GUARD", 8.0))
FLIP_FRAMES = int(os.environ.get("PC_FLIP_FRAMES", 3))


def settle(steer, last_steer, pending):
    """(steering_to_use, new_pending).

    `pending` counts how many frames the proposed reversal has survived; pass
    0 initially and feed the returned value back next tick.
    """
    if FLIP_FRAMES <= 1 or abs(last_steer) < FLIP_GUARD_DEG:
        return steer, 0
    if steer == 0.0 or (steer > 0) == (last_steer > 0):
        return steer, 0
    pending += 1
    if pending >= FLIP_FRAMES:
        return steer, 0
    # Not yet convinced. Hold the previous direction, but ease off so a genuine
    # reversal is not fought at full lock while it is being confirmed.
    return last_steer * 0.5, pending
