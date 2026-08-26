#!/usr/bin/env python3
"""Competition entry point. Started once by run.sh, immediately before the lap.

Written around the two rules that shape everything else:

* **One attempt.** There is no re-run, and a reboot mid-lap voids the record. So
  this loop is built not to die: every tick is wrapped, a failed camera read or
  a failed command is survived rather than raised, and the process keeps
  driving. A crash here is worth more lost time than any gain from tidier code.
* **Stateless.** The logic must drive from wherever the car is put down, so
  nothing is carried between frames. Restarting this program mid-lap is
  therefore harmless, which is what lets run.sh supervise it.

No simulator-only imports: `tools/` uses ground-truth pose and teleporting and
must never be reachable from here.
"""
import csv
import math
import os
import signal
import sys
import threading
import time
from dataclasses import replace

from drive import cones, control, fusion, lane, light, robot

HZ = float(os.environ.get("PC_HZ", 15.0))

WAIT_FOR_GREEN = os.environ.get("PC_WAIT_GREEN", "1") == "1"

# Consecutive green frames before pulling away. At 15 Hz this costs about a
# fifth of a second and rules out a single-frame glitch starting the car on a
# red light, which is a +10 s penalty.
GREEN_FRAMES = int(os.environ.get("PC_GREEN_FRAMES", 3))

# Give up waiting and go anyway after this long.
#
# Both ways of being wrong cost, and they cost differently. The official clock
# starts the moment the light turns green, whether or not the car noticed — so
# every second spent waiting after green is a second on the score, one for one.
# Starting *before* green is a false start: a flat +10 s, and the clock starts
# even earlier. The light is scripted to go green 2-5 s after the run begins,
# so anything still dark at 10 s means the detector has failed, not that the
# light is late — and by then driving beats sitting.
GREEN_TIMEOUT = float(os.environ.get("PC_GREEN_TIMEOUT", 10.0))

# Local rehearsal only — never set for a real run. See `_rehearsal`.
REHEARSE = os.environ.get("PC_REHEARSE", "") == "1"
REHEARSE_GREEN_AT = float(os.environ.get("PC_REHEARSE_GREEN_AT", 6.0))

# Testing aid only. Unset for the real run so the car keeps going.
MAX_SECONDS = float(os.environ.get("PC_MAX_SECONDS", 0) or 0)

# How long the car may command speed without moving before it is called stuck,
# and how long it then reverses. Long enough not to fire on a slow crawl out of
# a corner; short enough that a wall costs a second, not forty.
# Off by default, and that is a measured decision rather than caution.
# It was written for a real failure — a run that spent forty seconds pinned to
# a wall — but scored against the real objective it made things clearly worse:
# 63.0 average against 49.9 without it, with penalties rising from 10-20 s to
# 25-30 s. It fires when the car is legitimately crawling and the reverse
# manoeuvre then puts it off the track, which costs +5 s each time. At the
# speeds the car now runs it does not get pinned in the first place. Set
# PC_STUCK_SECONDS above 0 to re-enable — worth revisiting on the real car,
# where nobody teleports it out of trouble.
STUCK_SECONDS = float(os.environ.get("PC_STUCK_SECONDS", 0))

# Cone avoidance. Detection is solid (27/27 with no actionable false
# positives); it is the steering correction on top that has to earn its place,
# measured against the score rather than against lap progress.
AVOID_CONES = os.environ.get("PC_AVOID_CONES", "1") == "1"
CONE_LOG = os.environ.get("PC_CONE_LOG", "") == "1"
# Rebasing the lane target instead of adding a correction. Measured worse and
# off by default: 72.5 against 43.6 over three runs each, with collisions
# rising from 1-2 to 3 every run and one lap collecting seven excursions.
#
# The reasoning behind it still looks right — the additive correction is
# outvoted by the lane controller until the cone is 0.6 m away — but handing
# the manoeuvre full authority the moment anything is flagged turns every
# distant or marginal detection into a full-width swerve. Ramping in with
# proximity, which is what the additive version does by accident, is evidently
# worth more than starting early. Kept switchable rather than deleted.
REBASE = os.environ.get("PC_CONE_REBASE", "") == "1"
# Hold the last steering while blind. Mechanically sound and traced to a
# real failure, but NOT shown to help the score: 51.3 against 48.0 over
# four runs each, which is inside the run-to-run spread. Switchable.
HOLD_BLIND = os.environ.get("PC_HOLD_BLIND", "1") == "1"
# Require a reversal of a large steering command to be confirmed over a
# few frames. See control.settle.
SETTLE = os.environ.get("PC_SETTLE", "1") == "1"
# Weights for the learned cone detector. Empty, missing, or unloadable and the
# car falls back to the lidar alone — which is what it did before this existed,
# so a bad path on the day costs the improvement, not the run.
# The learned cone detector, off by default.
#
# It was trained on the practice course's orange cones and the qualifier's are
# green: it finds nothing at all on this map. That would be merely useless if
# fusion did not also use it as a veto — a lidar return beyond 1.6 m that the
# camera cannot confirm is discarded as scenery, so a blind model throws away
# exactly the detections the lidar now provides.
#
# And it provides a lot more than it used to. The new cones are 0.38 m tall
# against the old 0.23 m, so the beam at 0.182 m cuts the wide part of the cone
# instead of grazing its tip, and the measured detection range went from 1.6 m
# to 3.5 m. Scored on the qualifier course: with the model 24.21 and 56.31,
# without it 23.94 and 24.49, no collisions either way.
#
# Set PC_CONE_MODEL to a path to re-enable it — worth doing only after
# retraining on cones the same colour as the ones on the day.
CONE_MODEL = os.environ.get("PC_CONE_MODEL", "")
REVERSE_SECONDS = float(os.environ.get("PC_REVERSE_SECONDS", 0.8))

# 실차 고도화는 한 번의 인상적인 주행보다 반복 가능한 근거가 중요하다. 환경변수로
# 지정하면 매 제어 주기의 센서 추정·명령을 CSV에 남긴다. 비어 있으면 기존과 같은
# 저부하 동작이며, 기록 실패가 주행을 중단시키지 않는다.
TELEMETRY_CSV = os.environ.get("PC_TELEMETRY_CSV", "")

# A cone detector normally stops producing a steering correction at COMMIT_M.
# That is correct for the cone's centre, but not necessarily for the car's rear:
# the simulator repeatedly showed a car clearing a cone with its nose and then
# clipping it while immediately returning to the lane centre.  On the real car
# we cannot use a route/object identity, so retain only the *last measured*
# sensor-based correction for a short, ramped time after the cone passes under
# the front sensor.  It is opt-in for a staged real-car candidate; zero keeps
# the established controller byte-for-byte equivalent in behaviour.
CONE_EXIT_HOLD_S = float(os.environ.get("PC_CONE_EXIT_HOLD_S", 0) or 0)

# v3: a time-only hold changes its physical meaning whenever the speed cap
# changes. At 0.60 m/s the v2 0.18 s hold is only 10.8 cm; during cone braking
# it is less, even though the rear-clearance problem is geometric. This optional
# distance mode integrates already-commanded forward motion after the LiDAR
# correction ends, so it survives a cone-induced speed change without using a
# SIM route, object identity, or pose. If both modes are given, distance wins.
CONE_EXIT_HOLD_M = float(os.environ.get("PC_CONE_EXIT_HOLD_M", 0) or 0)

# v6-1 straight-line stabiliser.  Real-camera estimates on a straight vary by
# a few hundredths frame to frame; with v4's full gain that became alternating
# ±1–3 degree commands at 1.30 m/s.  The filter is deliberately eligible only
# for a clear, nearly straight road with no active cone correction.  On a bend,
# recovery, blind frame, or cone manoeuvre, v4's steering is passed unchanged.
STRAIGHT_STABILIZE = os.environ.get("PC_STRAIGHT_STABILIZE", "1") == "1"
STRAIGHT_VIEW_MIN = float(os.environ.get("PC_STRAIGHT_VIEW_MIN", 0.70))
STRAIGHT_SLOPE_MAX = float(os.environ.get("PC_STRAIGHT_SLOPE_MAX", 0.07))
STRAIGHT_STEER_MAX = float(os.environ.get("PC_STRAIGHT_STEER_MAX", 7.0))
STRAIGHT_ENTER_FRAMES = int(os.environ.get("PC_STRAIGHT_ENTER_FRAMES", 3))
STRAIGHT_DEADBAND_DEG = float(os.environ.get("PC_STRAIGHT_DEADBAND_DEG", 0.75))
STRAIGHT_EMA_ALPHA = float(os.environ.get("PC_STRAIGHT_EMA_ALPHA", 0.42))
STRAIGHT_SLEW_DEG = float(os.environ.get("PC_STRAIGHT_SLEW_DEG", 1.8))


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


class Telemetry:
    """Optional CSV logger that can never become a control-loop failure."""

    fields = ("elapsed_s", "source", "offset", "slope", "view", "conf",
              "speed_cmd", "steer_cmd_deg", "raw_steer_cmd_deg",
              "straight_stabilized", "cone_count", "cone_distance_m",
              "cone_bearing_deg", "cone_ff_deg", "cone_exit_ff_deg",
              "cone_speed_cap_mps")

    def __init__(self, path):
        self.file = None
        self.writer = None
        if not path:
            return
        try:
            self.file = open(path, "w", newline="", encoding="utf-8")
            self.writer = csv.DictWriter(self.file, fieldnames=self.fields)
            self.writer.writeheader()
            self.file.flush()
            log(f"텔레메트리 기록: {path}")
        except OSError as exc:
            log(f"텔레메트리 기록 비활성화: {exc}")
            self.close()

    def write(self, row):
        if self.writer is None:
            return
        try:
            self.writer.writerow(row)
        except OSError as exc:
            log(f"텔레메트리 기록 오류(비활성화): {exc}")
            self.close()

    def close(self):
        if self.file is not None:
            try:
                self.file.close()
            except OSError:
                pass
        self.file = None
        self.writer = None


def _rehearsal(img, t):
    """Paint a synthetic signal onto the frame. Local testing only.

    The simulator's light works fine and the gate should normally be tested
    against it. This exists because the rendered lamp lags a state command by
    about 4.5 seconds, which makes each real red-to-green cycle slow to set up,
    and because it can hold red for an exact, chosen number of seconds.

    Off unless PC_REHEARSE is set, and it only ever touches the frame the light
    detector reads while waiting at the line. Never set it for a scored run.
    """
    import cv2
    import numpy as np
    h, w = img.shape[:2]
    out = img.copy()
    green = t >= REHEARSE_GREEN_AT
    body = (34, 32, 30)

    # Blank out the simulator's own lamp first, wherever it happens to be. It is
    # permanently green, so leaving it anywhere in the ROI makes the rehearsal
    # read green from the first frame no matter what the synthetic signal shows
    # — which is exactly what happened when the car parked 10 cm off the
    # evaluation start pose and the lamp slid out from under the overlay.
    xa, xb, ya, yb = light._roi_box(h, w)
    roi_hsv = cv2.cvtColor(out[ya:yb, xa:xb], cv2.COLOR_BGR2HSV)
    existing = cv2.inRange(roi_hsv, (light.GREEN_H[0], light.MIN_S, light.MIN_V),
                           (light.GREEN_H[1], 255, 255))
    existing = cv2.dilate(existing, np.ones((9, 9), np.uint8))
    out[ya:yb, xa:xb][existing > 0] = body
    pw = int(8.0 * 3.9)                       # an 8 cm phone at 0.85 m
    ph = int(pw * 1.9)
    cx, cy = int(w * 0.885), int(h * 0.610)
    x0, y0 = cx - pw // 2, cy - ph // 2
    cv2.rectangle(out, (x0, y0), (x0 + pw, y0 + ph), body, -1)
    d = int(pw * 0.40)
    for k, lit_col in enumerate(((0, 0, 235), (0, 230, 0))):
        yc = y0 + ph // 3 * (k + 1)
        on = (k == 1) if green else (k == 0)
        col = lit_col if on else tuple(int(c * 0.12) for c in lit_col)
        cv2.circle(out, (x0 + pw // 2, yc), d // 2, col, -1)
    return out


def wait_for_green(car):
    """Hold at the line until the signal reads green.

    The car must not creep while waiting: the judge treats 3 cm of movement as
    the start, and moving before green is the false start. So a zero command
    goes out every tick rather than relying on the driver's watchdog.
    """
    if not WAIT_FOR_GREEN:
        log("신호등 대기 비활성 (PC_WAIT_GREEN=1 로 활성). 즉시 출발.")
        return

    log("신호등 대기 중..." + ("  [리허설 모드: 합성 신호]" if REHEARSE else ""))
    t0 = time.time()
    run = 0
    last_state = None
    while time.time() - t0 < GREEN_TIMEOUT:
        try:
            car.drive(0.0, 0.0)
            img = car.camera()
            if img is None:
                time.sleep(0.1)
                continue
            if REHEARSE:
                img = _rehearsal(img, time.time() - t0)
            r = light.detect(img)
            if r.state != last_state:
                log(f"  신호 {r.state} (area={r.area})")
                last_state = r.state
            run = run + 1 if r.go else 0
            if run >= GREEN_FRAMES:
                log(f"초록 확인 ({time.time() - t0:.1f}s 대기) — 출발")
                return
        except Exception as exc:                  # noqa: BLE001
            log(f"신호등 판독 오류(무시): {type(exc).__name__}: {exc}")
        time.sleep(1 / HZ)

    log(f"경고: {GREEN_TIMEOUT:.0f}초 안에 초록을 못 봤습니다. "
        f"기다리는 비용이 더 크므로 출발합니다.")


def main():
    log("PhysiCar autodrive 시작")
    running = [True]

    def stop(*_):
        running[0] = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    car = robot.Robot()
    gains = control.Gains()

    # The learned cone detector, if it is there.
    #
    # Loading it here rather than per tick keeps the 23 ms inference off the
    # control loop's critical path on the first frame, and it means a missing
    # or broken model is discovered now — at the line, with time to see the log
    # — instead of on the first cone at speed. Either way the car drives: with
    # no detector the avoidance falls back to the lidar, which is what it used
    # before this existed.
    # Load it in the background, because none of it is needed to leave the line.
    #
    # Measured: importing ultralytics costs 776 ms and the first inference
    # another 769 ms, and both were happening before the car so much as looked
    # at the traffic light. The car moved 2.05 s after the lamp went green with
    # the camera reading green the whole time — the delay was entirely this,
    # paid straight into the lap time.
    #
    # Starting only needs the light, which is a hue threshold, and the lane,
    # which is OpenCV. The cone detector has until the first cone to be ready,
    # which is several seconds away, so it is built on a thread and picked up
    # once it is. Until then the avoidance runs on the lidar, exactly as it
    # would if the weights were missing.
    holder = {"detector": None}
    if AVOID_CONES and CONE_MODEL:
        def _load():
            try:
                d = fusion.Detector(CONE_MODEL)
                if d.model is None:
                    log(f"고깔 모델 없음 ({CONE_MODEL}) — 라이다 단독으로 진행")
                    return
                # Warm it up here too; the first inference is the slow one.
                import numpy as _np
                d.camera_cones(_np.zeros((360, 480, 3), _np.uint8))
                holder["detector"] = d
                log(f"고깔 모델 준비 완료: {CONE_MODEL}")
            except Exception as exc:                       # noqa: BLE001
                log(f"고깔 모델 로드 실패 ({type(exc).__name__}) — 라이다 단독")
        threading.Thread(target=_load, daemon=True).start()
    # Field-adjustable without a code change. The competition allows no tuning
    # on the day, but the 29 hours with the real car before hand-in are exactly
    # when a speed cap needs to move.
    for name, var in (("speed_max", "PC_SPEED_MAX"), ("speed_min", "PC_SPEED_MIN"),
                      ("steer_p", "PC_STEER_P"), ("steer_d", "PC_STEER_D")):
        v = os.environ.get(var)
        if v:
            setattr(gains, name, float(v))
            log(f"{name} = {v} ({var})")

    # Camera straight ahead. tools/tilt_sweep.py measured that tilting down,
    # which looks like it should help, measurably does not: level keeps both
    # lane edges in frame deepest and keeps the traffic light visible.
    try:
        car.look(0.0, 0.0)
    except Exception as exc:                      # noqa: BLE001
        log(f"카메라 자세 설정 실패(무시): {exc}")

    # Don't pull away blind. The webserver subscribes to the camera topic on
    # first request, so the first reads after boot are legitimately empty.
    for i in range(50):
        try:
            if car.camera() is not None:
                log(f"카메라 준비 완료 ({i + 1}회 시도)")
                break
        except Exception:                         # noqa: BLE001
            pass
        time.sleep(0.2)
    else:
        log("경고: 카메라 프레임을 받지 못했습니다. 그래도 계속 시도합니다.")

    wait_for_green(car)

    t0 = time.time()
    telemetry = Telemetry(TELEMETRY_CSV)
    n = miss = errors = 0
    last_log = 0.0

    # Stuck detection, from odometry. Commanding speed while the car is not
    # actually moving means it is up against something — a wall, usually — and
    # nothing in the vision stack notices: one earlier run spent over forty
    # seconds pinned to a wall while the lane logic happily reported a road
    # ahead. The judge only rescues the car for leaving the track or touching an
    # obstacle, so a wall is on us, and on the real car nobody rescues it.
    stuck_since = None
    reversing_until = 0.0
    last_speed = 0.0
    last_steer = 0.0
    flip_pending = 0
    # Retains an avoidance direction only after a cone was genuinely close.
    # It is bounded by time or travelled distance and fades to zero, so a missed
    # LiDAR frame cannot turn into a permanent steering memory.
    cone_exit_ff = 0.0
    cone_exit_until = 0.0
    cone_exit_remaining_m = 0.0
    cone_exit_last_tick = None
    # The state is only used after several consecutive clear-straight frames.
    # Resetting it on every corner/avoidance keeps v4's responsive turn-in.
    straight_frames = 0
    straight_ema = None
    try:
        while running[0]:
            tick = time.time()
            if MAX_SECONDS and tick - t0 >= MAX_SECONDS:
                log(f"PC_MAX_SECONDS={MAX_SECONDS} 도달, 종료")
                break
            try:
                img = car.camera()
                if img is None:
                    miss += 1
                    car.drive(0.0, 0.0)
                    time.sleep(0.1)
                    continue
                est = lane.detect(img)
                seen = []
                cff, cs = 0.0, None
                if AVOID_CONES:
                    try:
                        detector = holder["detector"]
                        if detector is not None:
                            seen = fusion.as_cones(
                                detector.detect(img, car.lidar()))
                        else:
                            seen = cones.detect(car.lidar())
                        if REBASE:
                            doff, cs = cones.rebase(seen, est.offset,
                                                    last_speed)
                            cff = 0.0
                            if doff:
                                est = replace(est, offset=est.offset + doff)
                        else:
                            _, base_steer = control.command(
                                est, gains, last_steer)
                            cff, cs = cones.bias(seen, est.offset, last_speed,
                                                 base_steer)
                        if CONE_LOG:
                            log(f"라이다 검출 {len(seen)}개")
                        if CONE_LOG and seen:
                            near = seen[0]
                            log(f"고깔 {near.distance:.2f}m @{near.bearing:+.0f}도 "
                                f"측방{near.lateral:+.2f} -> 조향보정 {cff:+.1f}도 "
                                f"속도상한 {'없음' if cs is None else f'{cs:.2f}'}")
                    except Exception as exc:               # noqa: BLE001
                        cff, cs = 0.0, None
                        if CONE_LOG:
                            log(f"고깔 처리 오류: {type(exc).__name__}: {exc}")
                else:
                    cff, cs = 0.0, None
                speed, steer = control.command(
                    est, gains, last_steer if HOLD_BLIND else 0.0)
                if cs is not None:
                    speed = min(speed, cs)
                if cff:
                    steer = max(-gains.steer_limit,
                                min(gains.steer_limit, steer + cff))

                # Keep the just-computed cone-avoidance direction through the
                # rear-clearance interval. `cones.bias()` has already decided
                # the correction from camera/LiDAR geometry; this adds no SIM
                # pose, route, or object information. A current measurement
                # always wins. The hold begins only after the correction ends,
                # avoiding a stale command while the cone is still approaching.
                #
                # v2 used a fixed time. v3 optionally holds through a *distance*
                # travelled at the commanded speed, which keeps the physical
                # rear-clearance margin stable when cone braking changes speed.
                now = time.time()
                if cff:
                    cone_exit_ff = cff
                    cone_exit_until = 0.0
                    cone_exit_remaining_m = 0.0
                    cone_exit_last_tick = None
                elif cone_exit_ff and not cone_exit_until and not cone_exit_remaining_m:
                    if CONE_EXIT_HOLD_M > 0:
                        cone_exit_remaining_m = CONE_EXIT_HOLD_M
                        cone_exit_last_tick = now
                    elif CONE_EXIT_HOLD_S > 0:
                        cone_exit_until = now + CONE_EXIT_HOLD_S
                exit_ff = 0.0
                if cone_exit_remaining_m > 0:
                    dt = max(0.0, min(now - (cone_exit_last_tick or now), 0.25))
                    cone_exit_last_tick = now
                    cone_exit_remaining_m = max(0.0, cone_exit_remaining_m - speed * dt)
                    remaining = cone_exit_remaining_m / CONE_EXIT_HOLD_M
                    exit_ff = cone_exit_ff * max(0.0, min(1.0, remaining))
                elif cone_exit_until > now:
                    remaining = (cone_exit_until - now) / CONE_EXIT_HOLD_S
                    exit_ff = cone_exit_ff * max(0.0, min(1.0, remaining))
                elif cone_exit_until:
                    cone_exit_ff = 0.0
                    cone_exit_until = 0.0
                if exit_ff:
                    steer = max(-gains.steer_limit,
                                min(gains.steer_limit, steer + exit_ff))
                elif not cone_exit_remaining_m and not cone_exit_until:
                    cone_exit_ff = 0.0
                    cone_exit_last_tick = None
                # Preserve the exact v4 command everywhere except a confirmed,
                # clear straight.  Cone feed-forward is explicitly excluded:
                # it is a safety manoeuvre, not centring noise.
                raw_steer = steer
                straight_ok = (
                    STRAIGHT_STABILIZE and est.ok and not cff and not exit_ff
                    and est.view >= STRAIGHT_VIEW_MIN
                    and abs(est.slope) <= STRAIGHT_SLOPE_MAX
                    and abs(raw_steer) <= STRAIGHT_STEER_MAX
                )
                if straight_ok:
                    straight_frames += 1
                else:
                    straight_frames = 0
                    straight_ema = None

                straight_stabilized = False
                if straight_frames >= max(1, STRAIGHT_ENTER_FRAMES):
                    alpha = max(0.0, min(1.0, STRAIGHT_EMA_ALPHA))
                    straight_ema = (raw_steer if straight_ema is None else
                                    alpha * raw_steer + (1.0 - alpha) * straight_ema)
                    steer = 0.0 if abs(straight_ema) < STRAIGHT_DEADBAND_DEG else straight_ema
                    # Camera→HTTP→servo delay makes a one-frame reversal on a
                    # fast straight physically arrive too late.  Bound only the
                    # v6 straight command; corner/recovery commands are intact.
                    if STRAIGHT_SLEW_DEG > 0:
                        steer = max(last_steer - STRAIGHT_SLEW_DEG,
                                    min(last_steer + STRAIGHT_SLEW_DEG, steer))
                    straight_stabilized = True

                if SETTLE:
                    steer, flip_pending = control.settle(
                        steer, last_steer, flip_pending)
                last_speed = speed
                last_steer = steer

                now = time.time()
                if now < reversing_until:
                    # Back out of whatever we hit, turning as we go so the next
                    # attempt does not repeat the same line.
                    car.drive(-0.45, -steer)
                    n += 1
                    time.sleep(max(0.0, 1 / HZ - (now - tick)))
                    continue

                v = car.speed_now() if STUCK_SECONDS > 0 else None
                if speed > 0.25 and v is not None and abs(v) < 0.08:
                    stuck_since = stuck_since or now
                    if now - stuck_since > STUCK_SECONDS:
                        log(f"끼임 감지 ({now - stuck_since:.1f}초) — 후진 탈출")
                        reversing_until = now + REVERSE_SECONDS
                        stuck_since = None
                else:
                    stuck_since = None

                car.drive(speed, steer)

                near = seen[0] if seen else None
                telemetry.write({
                    "elapsed_s": f"{tick - t0:.3f}",
                    "source": est.source,
                    "offset": f"{est.offset:.4f}",
                    "slope": f"{est.slope:.4f}",
                    "view": f"{est.view:.4f}",
                    "conf": f"{est.conf:.4f}",
                    "speed_cmd": f"{speed:.3f}",
                    "steer_cmd_deg": f"{steer:.3f}",
                    "raw_steer_cmd_deg": f"{raw_steer:.3f}",
                    "straight_stabilized": int(straight_stabilized),
                    "cone_count": len(seen),
                    "cone_distance_m": "" if near is None else f"{near.distance:.3f}",
                    "cone_bearing_deg": "" if near is None else f"{near.bearing:.3f}",
                    "cone_ff_deg": f"{cff:.3f}",
                    "cone_exit_ff_deg": f"{exit_ff:.3f}",
                    "cone_speed_cap_mps": "" if cs is None else f"{cs:.3f}",
                })

                if tick - last_log >= 2.0:
                    last_log = tick
                    tag = " straight-filter" if straight_stabilized else ""
                    log(f"{est.source:5s} off={est.offset:+.2f} "
                        f"bend={est.slope:+.2f} view={est.view:.2f} -> "
                        f"steer={steer:+6.1f}deg raw={raw_steer:+6.1f}deg "
                        f"speed={speed:.2f}{tag}")
                n += 1
            except Exception as exc:              # noqa: BLE001
                # One bad frame or one dropped request must not end the run.
                errors += 1
                if errors <= 5 or errors % 50 == 0:
                    log(f"틱 오류 {errors}회: {type(exc).__name__}: {exc}")
                time.sleep(0.05)
            time.sleep(max(0.0, 1 / HZ - (time.time() - tick)))
    finally:
        try:
            car.stop()
        except Exception:                         # noqa: BLE001
            pass
        telemetry.close()
        dt = max(time.time() - t0, 1e-6)
        log(f"정지. {n} ticks / {dt:.1f}s ({n / dt:.1f} Hz), "
            f"프레임 없음 {miss}회, 오류 {errors}회")


if __name__ == "__main__":
    main()
    sys.exit(0)
