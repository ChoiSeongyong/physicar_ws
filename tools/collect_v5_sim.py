#!/usr/bin/env python3
"""Collect diverse SIM camera/steering data for the v5 adviser.

This is SIM-only: route/pose APIs are used only to generate a supervised
teacher label and never appear in the v5 runtime controller.  Each profile
physically drives a centred, left, right, or smoothly transitioning line and
stores the camera image plus the teacher steering command.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import time
from pathlib import Path

import cv2
import requests

ROOT = Path(__file__).resolve().parents[1]
BASE = "http://localhost"
SIM = f"{BASE}/sim/api"
WHEELBASE = 0.19
STEER_LIMIT = math.radians(20)


def get(path):
    r = requests.get(SIM + path, timeout=5); r.raise_for_status(); return r.json()


def post(path, data):
    r = requests.post(SIM + path, json=data, timeout=5); r.raise_for_status(); return r.json() if r.content else {}


def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi


def nearest(points, x, y, previous=None):
    candidates = range(len(points)) if previous is None else ((previous + k) % len(points) for k in range(-15, 60))
    return min(candidates, key=lambda i: (points[i][0]-x)**2+(points[i][1]-y)**2)


def advance(points, start, distance):
    total = 0.; i = start
    while total < distance:
        j = (i + 1) % len(points); total += math.dist(points[i], points[j]); i = j
        if i == start: break
    return i


def normal(points, i):
    a, b = points[(i-1) % len(points)], points[(i+1) % len(points)]
    dx, dy = b[0]-a[0], b[1]-a[1]; n = max(math.hypot(dx, dy), 1e-8)
    return -dy/n, dx/n


def profile_offset(name, progress):
    """Metres, bounded inside the lane. progress is 0..1 around one lap."""
    if name == "centre": return 0.0
    if name == "left": return 0.18
    if name == "right": return -0.18
    if name == "left_to_right": return 0.18 * math.cos(2 * math.pi * progress)
    if name == "right_to_left": return -0.18 * math.cos(2 * math.pi * progress)
    if name == "sine": return 0.16 * math.sin(4 * math.pi * progress)
    raise ValueError(name)


def wait_ready():
    for _ in range(150):
        try:
            s = get('/status')
            if s.get('running') and not s.get('switching'): return
        except requests.RequestException: pass
        time.sleep(.2)
    raise RuntimeError('simulator did not become ready')


def capture_frame():
    r = requests.get(BASE + '/camera', params={'width': 480, 'height': 360}, timeout=3)
    if not r.ok or r.content[:2] != b'\xff\xd8': return None
    import numpy as np
    return cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)


def drive(speed, steer):
    requests.post(BASE + '/steering', json={'value': steer}, timeout=2)
    requests.post(BASE + '/speed', json={'value': speed}, timeout=2)


def stop():
    try: drive(0., 0.)
    except requests.RequestException: pass


def collect_profile(points, name, out, hz, max_seconds):
    frames = out / 'frames'; frames.mkdir(parents=True, exist_ok=True)
    csv_path = out / 'labels.csv'
    previous = None; started = time.monotonic(); sample_at = 0.; count = 0
    with csv_path.open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=('frame','elapsed_s','profile','route_index','progress','target_offset_m','steer_deg','speed_mps'))
        w.writeheader()
        while time.monotonic() - started < max_seconds:
            tick = time.monotonic()
            try:
                state = get('/state'); pose = state['vehicle']
                i = nearest(points, pose['x'], pose['y'], previous); previous = i
                progress = i / len(points)
                look = advance(points, i, 0.72)
                offset = profile_offset(name, look / len(points))
                nx, ny = normal(points, look)
                tx, ty = points[look][0] + offset * nx, points[look][1] + offset * ny
                alpha = wrap(math.atan2(ty-pose['y'], tx-pose['x']) - pose['yaw'])
                distance = max(math.hypot(tx-pose['x'], ty-pose['y']), .16)
                steer = max(-STEER_LIMIT, min(STEER_LIMIT, math.atan2(2*WHEELBASE*math.sin(alpha), distance)))
                # Slow enough for reliable capture yet varied around curved track.
                speed = .65 if abs(steer) < math.radians(10) else .48
                drive(speed, steer)
                if tick >= sample_at:
                    img = capture_frame()
                    if img is not None:
                        fn = f'{count:06d}.jpg'; cv2.imwrite(str(frames / fn), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
                        w.writerow({'frame': fn, 'elapsed_s': f'{tick-started:.3f}', 'profile': name, 'route_index': i,
                                    'progress': f'{progress:.6f}', 'target_offset_m': f'{offset:.4f}',
                                    'steer_deg': f'{math.degrees(steer):.4f}', 'speed_mps': speed})
                        count += 1
                    sample_at = tick + 1 / hz
            except requests.RequestException:
                pass
            time.sleep(max(0., 1/15 - (time.monotonic()-tick)))
    stop(); return count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', type=Path, default=ROOT/'dataset'/'v5_sim')
    ap.add_argument('--profiles', nargs='+', default=['centre','left','right','left_to_right','right_to_left','sine'])
    ap.add_argument('--hz', type=float, default=6.)
    ap.add_argument('--seconds', type=float, default=42.)
    ap.add_argument('--clean', action='store_true')
    args = ap.parse_args()
    if args.clean and args.out.exists(): shutil.rmtree(args.out)
    args.out.mkdir(parents=True, exist_ok=True)
    world = get('/world'); route = get('/route')
    points = [tuple(map(float, p)) for p in route['waypoints'][:-1]]
    manifest = {'world': world.get('world'), 'world_display': world.get('display'), 'world_id': world.get('world_id'),
                'world_rev': world.get('rev'), 'profiles': args.profiles, 'hz': args.hz, 'seconds_per_profile': args.seconds,
                'created_at': time.strftime('%FT%TZ', time.gmtime()), 'samples': {}}
    print(f"[v5 collect] {manifest['world_display']} ({manifest['world_id']}) profiles={args.profiles}", flush=True)
    for name in args.profiles:
        print(f'[v5 collect] respawn -> {name}', flush=True)
        post('/respawn', {}); wait_ready(); time.sleep(2)
        # Start facing route 0; no traffic-light procedure: this is labelled data, not an evaluation.
        a, b = points[0], points[1]; heading = math.atan2(b[1]-a[1], b[0]-a[0])
        post('/pose', {'x': a[0], 'y': a[1], 'yaw': heading}); time.sleep(.5)
        session = args.out / f"{manifest['world_id']}_{name}"
        manifest['samples'][name] = collect_profile(points, name, session, args.hz, args.seconds)
        print(f"[v5 collect] {name}: {manifest['samples'][name]} frames", flush=True)
    (args.out/'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    print('[v5 collect] complete', json.dumps(manifest['samples']), flush=True)

if __name__ == '__main__': main()
