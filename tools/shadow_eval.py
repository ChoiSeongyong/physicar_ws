#!/usr/bin/env python3
"""Shadow judge for real-car candidates — SIM-only development tool.

Runs the EXACT same entry point run_real.sh uses — `race/run.py` with
PC_TARGET=real, which immediately hands off to `autodrive.main()` — as a
subprocess against a live SIM world, while this script independently scores
the run from ground truth (`/sim/api`) by re-implementing — line for line —
the world's published evaluation script (see `sim_evaluation`): start-line
false-start detection, off-track excursions (per-waypoint half-gap + 0.12 m,
with the same "teleport back 0.3 m past the excursion" recovery), cone hits
(1 cm displacement, 1 s re-arm, restore + resume-30cm-past recovery), and lap
completion via start-line crossing after half the track has been covered.

This never runs on the real car and is never imported by autodrive.py,
race/run.py or drive/*: it is invoked directly, e.g.

    python3 tools/shadow_eval.py --label v3 \
        --set PC_SPEED_MAX=0.60 --set PC_SPEED_MIN=0.32 \
        --set PC_CONE_SLOW_MAX=0.55 --set PC_CONE_SLOW_MIN=0.32 \
        --set PC_CONE_EXIT_HOLD_M=0.11

Every --set is exported into the child's environment; anything not given
keeps autodrive.py's own default (same behaviour as the run_real*.sh
wrappers, which this script is meant to pre-screen). Results are appended to
race/results/shadow_eval.jsonl (gitignored) and summarised on stdout.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import signal
import subprocess
import sys
import time
from pathlib import Path

import requests

WORKSPACE = Path(__file__).resolve().parents[1]
BASE = os.environ.get("PHYSICAR_URL", "http://localhost")
SIM = f"{BASE}/sim/api"

HZ = 20.0
FRONT = 0.12                    # front-point offset used by the judge, metres
BACK = 0.10                     # start pose: this far behind waypoint 0
OFFTRACK_MARGIN = 0.12
CONE_MOVE_TOL = 0.01
CONE_Z_DROP = 0.04
REARM_S = 1.0
RESUME_AHEAD_M = 0.3
GREEN_DELAY_RANGE = (2.0, 5.0)
HARD_CUTOFF_S = 210.0            # harness safety net beyond the 180s rule

RESULTS = WORKSPACE / "race" / "results" / "shadow_eval.jsonl"


def sget(path):
    r = requests.get(f"{SIM}{path}", timeout=5)
    r.raise_for_status()
    return r.json()


def spost(path, payload, retries=10, retry_delay=0.3):
    # Right after /respawn (or during a light's yellow transition) the sim can
    # answer transient 409s for a few hundred ms — retry instead of aborting
    # the whole run over a race condition that isn't a real failure.
    last_exc = None
    for attempt in range(retries):
        try:
            r = requests.post(f"{SIM}{path}", json=payload, timeout=5)
            if r.status_code == 409 and attempt < retries - 1:
                time.sleep(retry_delay)
                continue
            r.raise_for_status()
            return r.json() if r.content else {}
        except requests.RequestException as exc:
            last_exc = exc
            if attempt < retries - 1:
                time.sleep(retry_delay)
                continue
            raise
    raise last_exc


def drive_stop():
    for ep in ("speed", "steering"):
        try:
            requests.post(f"{BASE}/{ep}", json={"value": 0}, timeout=2)
        except requests.RequestException:
            pass


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Track:
    """Mirrors the waypoint-index helpers in the world's evaluation script."""

    def __init__(self):
        route = sget("/route")
        self.wp = [tuple(map(float, p)) for p in route["waypoints"]]
        self.inner = [tuple(map(float, p)) for p in route.get("inner", [])]
        self.outer = [tuple(map(float, p)) for p in route.get("outer", [])]
        self.n = len(self.wp) - 1   # closed loop: last point repeats the first

    def nearest_idx(self, x, y):
        """Global nearest waypoint — used where the judge never windows."""
        return min(range(self.n),
                   key=lambda i: (self.wp[i][0] - x) ** 2 + (self.wp[i][1] - y) ** 2)

    def nearest_idx_near(self, x, y, frm, back, fwd):
        best_i, best_d = frm, float("inf")
        for k in range(-back, fwd + 1):
            i = (frm + k) % self.n
            d = (self.wp[i][0] - x) ** 2 + (self.wp[i][1] - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def fwd_index(self, frm, distance):
        travelled, i = 0.0, frm
        while travelled < distance:
            j = (i + 1) % self.n
            travelled += dist(self.wp[i], self.wp[j])
            i = j
            if i == frm:
                break
        return i

    def half_gap(self, i):
        if not self.inner or not self.outer:
            return 0.45
        return dist(self.inner[i], self.outer[i]) / 2.0

    def heading_at(self, i):
        j = (i + 1) % self.n
        return math.atan2(self.wp[j][1] - self.wp[i][1], self.wp[j][0] - self.wp[i][0])


def crossed_start_line(prev, f, a, b):
    if prev is None:
        return False
    d = (f[0] - prev[0]) * (b[1] - a[1]) - (f[1] - prev[1]) * (b[0] - a[0])
    if abs(d) < 1e-12:
        return False
    t = ((a[0] - prev[0]) * (b[1] - a[1]) - (a[1] - prev[1]) * (b[0] - a[0])) / d
    u = ((a[0] - prev[0]) * (f[1] - prev[1]) - (a[1] - prev[1]) * (f[0] - prev[0])) / d
    return 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0


def ensure_light_red():
    lights = sget("/traffic_lights")
    state = None
    for l in lights.get("lights", lights if isinstance(lights, list) else []):
        if l.get("name") == "light1":
            state = l.get("state")
    if state == "green":
        spost("/traffic_lights/light1", {"state": "red"}) if False else None
        # green->red passes through 3s yellow during which POSTs 409; retry.
        for _ in range(60):
            try:
                requests.post(f"{SIM}/traffic_lights/light1", json={"state": "red"},
                              timeout=5)
                break
            except requests.RequestException:
                time.sleep(0.2)
        for _ in range(60):
            lights = sget("/traffic_lights")
            cur = next((l.get("state") for l in lights.get("lights", [])
                       if l.get("name") == "light1"), None)
            if cur == "red":
                break
            time.sleep(0.2)
    elif state != "red":
        requests.post(f"{SIM}/traffic_lights/light1", json={"state": "red"}, timeout=5)
    # The rendered lamp mesh lags the API state by ~4.5s (see autodrive.py's
    # _rehearsal comment) — the light API can say "red" while the camera the
    # real controller reads still shows the old colour. Racing autodrive.py
    # right after the API confirms red reproduces a false start that has
    # nothing to do with the controller: give the render time to catch up
    # before the subprocess starts watching the lamp.
    time.sleep(5.0)


def kill_process(proc):
    if proc.poll() is not None:
        return
    try:
        proc.send_signal(signal.SIGTERM)
    except ProcessLookupError:
        return
    for _ in range(25):
        if proc.poll() is not None:
            return
        time.sleep(0.2)
    try:
        proc.kill()
    except ProcessLookupError:
        pass


def run_once(label, env_overrides, log_path, hard_cutoff_s=HARD_CUTOFF_S,
             entry="race/run.py"):
    # Persist the immutable world identity with every score. This prevents
    # results from different installed AMET/practice worlds being compared as
    # though they came from the same course.
    world_info = sget("/world")
    print(f"[shadow] world={world_info.get('display', world_info.get('world'))} "
          f"id={world_info.get('world_id')} rev={world_info.get('rev')}", flush=True)
    print(f"[shadow] respawning world for '{label}' ...", flush=True)
    requests.post(f"{SIM}/respawn", timeout=30).raise_for_status()
    # A full world reload takes several seconds server-side (switching=true
    # the whole time, during which /pose etc. answer 409) — poll status
    # instead of guessing a fixed sleep.
    for _ in range(100):
        try:
            st = requests.get(f"{SIM}/status", timeout=5).json()
        except requests.RequestException:
            st = {}
        if st.get("running") and not st.get("switching"):
            break
        time.sleep(0.2)
    time.sleep(1.0)   # let physics settle once more after it reports ready

    ensure_light_red()
    track = Track()

    # initialize(): teleport to `BACK` metres behind waypoint 0, facing wp1.
    heading = track.heading_at(0)
    sx = track.wp[0][0] - math.cos(heading) * BACK
    sy = track.wp[0][1] - math.sin(heading) * BACK
    spost("/pose", {"x": sx, "y": sy, "yaw": heading})
    time.sleep(0.3)

    objects = sget("/objects").get("objects", [])
    cone_base = {o["name"]: dict(o["origin"]) for o in objects
                if o.get("type") == "object" and o.get("movable")}

    env = dict(os.environ)
    env.update({
        "PYTHONPATH": str(WORKSPACE),
        "PYTHONUNBUFFERED": "1",
        "PYTHONDONTWRITEBYTECODE": "1",
        "YOLO_OFFLINE": "1",
        "PC_WAIT_GREEN": "1",
        "PC_COURSE_AUTO": "0",   # matches run_real.sh: real car has no /sim/api
        "PC_TARGET": "real",    # forces race/run.py's real-mode branch, same as run_real.sh
    })
    env.update({k: str(v) for k, v in env_overrides.items()})
    env.pop("PC_MAX_SECONDS", None)   # the judge decides when the run ends

    # Run the selected sensor-controller entry point with PC_TARGET=real.
    # The default remains race/run.py, matching run_real.sh.  Versioned v6
    # candidates are passed directly so shadow results test their actual code.
    entry_path = WORKSPACE / entry
    if not entry_path.is_file():
        raise FileNotFoundError(f"entry point not found: {entry}")
    log = open(log_path, "w", encoding="utf-8")
    proc = subprocess.Popen([sys.executable, "-u", entry],
                            cwd=WORKSPACE, env=env, stdout=log, stderr=subprocess.STDOUT)

    def cone_moved(cur, base):
        return (dist((cur["x"], cur["y"]), (base["x"], base["y"])) > CONE_MOVE_TOL
                or abs(cur["z"] - base["z"]) > CONE_Z_DROP)

    def restore_cone(name, base):
        spost(f"/models/{name}/pose", {"x": base["x"], "y": base["y"],
                                        "yaw": base.get("yaw", 0.0)})

    prev = None
    last_idx = None
    adv = 0
    hit = {}
    off_latch = False
    red_t = None
    green_delay = random.uniform(*GREEN_DELAY_RANGE)
    green_sent = False
    car_start = None
    start_t = None
    penalty = 0
    cones_hit = set()
    offtrack_events = 0
    false_start = False
    finished = False
    finish_time = None
    a0, b0 = track.inner[0], track.outer[0]

    wall0 = time.monotonic()
    period = 1.0 / HZ
    try:
        while True:
            tick_wall = time.monotonic()
            if tick_wall - wall0 > hard_cutoff_s:
                print(f"[shadow] '{label}': hard cutoff reached, aborting", flush=True)
                break
            if proc.poll() is not None:
                print(f"[shadow] '{label}': autodrive exited early (rc={proc.returncode})",
                      flush=True)
                break

            state = sget("/state")
            t = state["time"]
            pose = state["vehicle"]
            x, y, yaw = pose["x"], pose["y"], pose["yaw"]
            light_state = next((l.get("state") for l in state.get("lights", [])
                               if l.get("name") == "light1"), None)

            # driveLightSequence(): redT only latches once red is OBSERVED
            # (matches evaluation.js — not merely "loop has started").
            if red_t is None and light_state == "red":
                red_t = t
            if red_t is not None and not green_sent and t - red_t >= green_delay:
                green_sent = True
                requests.post(f"{SIM}/traffic_lights/light1", json={"state": "green"},
                             timeout=5)

            if start_t is None:
                # Arm phase: keep the course intact, no penalties yet — a cone
                # nudged before the light is irrelevant to the score.
                live_objects = state.get("objects", {})
                for name, base in cone_base.items():
                    cur = live_objects.get(name)
                    if cur is None:
                        continue
                    h0 = hit.get(name)
                    if cone_moved(cur, base) and (not h0 or t >= h0["rearmT"]):
                        restore_cone(name, base)
                        hit[name] = {"latched": False, "rearmT": t + REARM_S}

                if car_start is None:
                    car_start = (x, y)
                go_green = green_sent and light_state == "green"
                car_moved = dist((x, y), car_start) > 0.03
                if go_green or car_moved:
                    start_t = t
                    prev = None
                    if car_moved and light_state != "green":
                        penalty += 10
                        false_start = True
                        print(f"[shadow] '{label}': FALSE START at t={t:.2f}", flush=True)
                time.sleep(max(0.0, period - (time.monotonic() - tick_wall)))
                continue

            # racing phase
            f = (x + math.cos(yaw) * FRONT, y + math.sin(yaw) * FRONT)
            if prev is not None and dist(f, prev) > 0.5:
                prev = None
            crossed = crossed_start_line(prev, f, a0, b0)
            prev = f
            if crossed and adv > track.n / 2:
                finished = True
                finish_time = t
                break

            # trackAdvance
            i = track.nearest_idx(x, y) if last_idx is None \
                else track.nearest_idx_near(x, y, last_idx, 10, 15)
            if last_idx is not None:
                d = i - last_idx
                if d > track.n / 2:
                    d -= track.n
                elif d < -track.n / 2:
                    d += track.n
                adv += d
            last_idx = i

            # off-track (always a GLOBAL nearest search, per the judge)
            gi = track.nearest_idx(x, y)
            half = track.half_gap(gi)
            off = dist((x, y), track.wp[gi]) > half + OFFTRACK_MARGIN
            if off:
                if not off_latch:
                    off_latch = True
                    penalty += 5
                    offtrack_events += 1
                    ri = track.nearest_idx(x, y) if last_idx is None \
                        else track.nearest_idx_near(x, y, last_idx, 10, 15)
                    ri = track.fwd_index(ri, RESUME_AHEAD_M)
                    q = track.wp[(ri + 1) % track.n]
                    spost("/pose", {"x": track.wp[ri][0], "y": track.wp[ri][1],
                                    "yaw": math.atan2(q[1] - track.wp[ri][1],
                                                      q[0] - track.wp[ri][0])})
                    prev = None
                    print(f"[shadow] '{label}': +5s off-track @t={t:.2f} (n={offtrack_events})",
                          flush=True)
            else:
                off_latch = False

            # cone hits
            live_objects = state.get("objects", {})
            for name, base in cone_base.items():
                cur = live_objects.get(name)
                if cur is None:
                    continue
                moved = cone_moved(cur, base)
                h = hit.get(name)
                if h and h["latched"]:
                    if t < h["rearmT"]:
                        continue
                    if not moved:
                        h["latched"] = False
                        continue
                    restore_cone(name, base)
                    h["rearmT"] = t + REARM_S
                    continue
                if moved:
                    hit[name] = {"latched": True, "rearmT": t + REARM_S}
                    penalty += 5
                    cones_hit.add(name)
                    restore_cone(name, base)
                    car_idx = last_idx if last_idx is not None else track.nearest_idx(x, y)
                    obs_idx = track.nearest_idx_near(base["x"], base["y"], car_idx, 5, 20)
                    bi = track.fwd_index(obs_idx, RESUME_AHEAD_M)
                    q = track.wp[(bi + 1) % track.n]
                    spost("/pose", {"x": track.wp[bi][0], "y": track.wp[bi][1],
                                    "yaw": math.atan2(q[1] - track.wp[bi][1],
                                                      q[0] - track.wp[bi][0])})
                    prev = None
                    print(f"[shadow] '{label}': +5s cone hit '{name}' @t={t:.2f}", flush=True)

            time.sleep(max(0.0, period - (time.monotonic() - tick_wall)))
    finally:
        kill_process(proc)
        drive_stop()
        log.close()

    lap_time = (finish_time - start_t) if (finished and start_t is not None) else None
    score = (lap_time + penalty) if lap_time is not None else None
    result = {
        "label": label,
        "world": world_info.get("world"),
        "world_display": world_info.get("display"),
        "world_id": world_info.get("world_id"),
        "world_rev": world_info.get("rev"),
        "env": env_overrides,
        "entry": entry,
        "finished": finished,
        "lap_time_s": lap_time,
        "cone_hits": len(cones_hit),
        "cones": sorted(cones_hit),
        "offtrack_events": offtrack_events,
        "false_start": false_start,
        "penalty_s": penalty,
        "score_s": score,
        "log": str(log_path),
    }
    RESULTS.parent.mkdir(parents=True, exist_ok=True)
    with RESULTS.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(result, ensure_ascii=False) + "\n")

    print("──────── SHADOW RESULT ────────")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print("────────────────────────────────")
    return result


def parse_sets(pairs):
    out = {}
    for p in pairs or []:
        if "=" not in p:
            raise SystemExit(f"--set expects NAME=VALUE, got: {p}")
        k, v = p.split("=", 1)
        out[k] = v
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", required=True)
    ap.add_argument("--set", dest="sets", action="append",
                    help="NAME=VALUE env override for autodrive.py; repeatable")
    ap.add_argument("--log", default=None, help="path to write controller stdout/stderr")
    ap.add_argument("--entry", default="race/run.py",
                    help="workspace-relative controller entry point (default: race/run.py)")
    ap.add_argument("--cutoff", type=float, default=HARD_CUTOFF_S,
                    help="hard wall-clock abort in seconds (safety net beyond the 180s rule)")
    args = ap.parse_args()

    env_overrides = parse_sets(args.sets)
    log_path = Path(args.log) if args.log else Path(f"/tmp/shadow_{args.label}.log")
    run_once(args.label, env_overrides, log_path, hard_cutoff_s=args.cutoff,
             entry=args.entry)


if __name__ == "__main__":
    main()
