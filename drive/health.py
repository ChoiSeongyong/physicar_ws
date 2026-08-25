#!/usr/bin/env python3
"""Is this machine fast enough to drive? Measure, do not assume.

    python3 tools/health.py

Run this on whatever machine is actually driving. The controller was tuned at
about 15 Hz on one particular setup, and every distance in it is a time in
disguise: the reaction range is speed x 1.6 s, the steering reversal guard is
three frames, the speed law reacts to what the camera showed *this* tick. Halve
the loop rate and the car reacts at half the distance, which looks exactly like
"it used to avoid that cone and now it hits it".

So this times each stage separately. A slow loop has a cause — the camera, the
lidar, the detector, or the simulator itself running below real time — and the
cause decides the fix.

Standalone on purpose: it imports nothing from this project except the lane
detector, so it can be pasted onto a machine that only has the driving files.
"""
import os
import statistics
import sys
import time

import numpy as np
import requests

BASE = os.environ.get("PHYSICAR_URL", "http://localhost")

# What the tuning assumed. Not a hard requirement — a report, not a verdict.
REF_HZ = 15.0


def timeit(fn, n=25):
    """(median ms, p90 ms) — median because one slow call is not the story."""
    ts = []
    for _ in range(n):
        t = time.time()
        try:
            fn()
        except Exception:                                  # noqa: BLE001
            return None, None
        ts.append((time.time() - t) * 1000.0)
    ts.sort()
    return statistics.median(ts), ts[int(0.9 * len(ts)) - 1]


def main():
    print(f"대상: {BASE}\n")

    # --- simulator health -------------------------------------------------
    try:
        st = requests.get(f"{BASE}/sim/api/state", timeout=5).json()
        rtf = st.get("rtf", 0.0)
        print(f"시뮬레이터   RTF {rtf:.2f}   paused={st.get('paused')}")
        if rtf < 0.85:
            print("  경고: 실시간보다 느립니다. 이 상태에서는 제어 주기가")
            print("        아무리 빨라도 차가 굼뜨게 반응합니다.")
    except requests.RequestException:
        print("시뮬레이터   /sim/api 응답 없음 (실차이거나 API가 없는 환경)")
    print()

    # --- per-stage timing -------------------------------------------------
    import cv2
    from drive import lane

    def cam():
        r = requests.get(f"{BASE}/camera", timeout=3).content
        return cv2.imdecode(np.frombuffer(r, np.uint8), cv2.IMREAD_COLOR)

    img = cam()
    if img is None:
        print("카메라에서 프레임을 못 받았습니다. 여기서 중단합니다.")
        return 1
    print(f"카메라 해상도 {img.shape[1]}x{img.shape[0]}\n")

    stages = [
        ("카메라 수신", lambda: cam()),
        ("차선 검출", lambda: lane.detect(img)),
        ("라이다 수신", lambda: requests.get(f"{BASE}/lidar", timeout=3).json()),
        ("속도 명령", lambda: requests.post(f"{BASE}/speed", json={"value": 0.0},
                                        timeout=3)),
        ("조향 명령", lambda: requests.post(f"{BASE}/steering",
                                        json={"value": 0.0}, timeout=3)),
    ]
    total = 0.0
    print(f"{'단계':<14}{'중앙값':>10}{'90퍼센타일':>12}")
    for name, fn in stages:
        med, p90 = timeit(fn)
        if med is None:
            print(f"{name:<14}{'실패':>10}")
            continue
        total += med
        print(f"{name:<14}{med:>8.1f} ms{p90:>10.1f} ms")
    print(f"{'합계':<14}{total:>8.1f} ms   -> 최대 {1000.0 / total:>5.1f} Hz")
    print()

    # --- the loop as it actually runs --------------------------------------
    from drive import cones as cone_mod
    from drive import control

    gains = control.Gains()
    ts = []
    last_steer = 0.0
    for _ in range(40):
        t = time.time()
        im = cam()
        if im is None:
            continue
        est = lane.detect(im)
        sp, stg = control.command(est, gains, last_steer)
        try:
            seen = cone_mod.detect(
                requests.get(f"{BASE}/lidar", timeout=3).json())
            cone_mod.bias(seen, est.offset, sp, stg)
        except Exception:                                  # noqa: BLE001
            pass
        last_steer = stg
        ts.append(time.time() - t)
    if ts:
        ts.sort()
        med = statistics.median(ts)
        hz = 1.0 / med
        print(f"실제 루프    중앙값 {med * 1000:.1f} ms  ->  {hz:.1f} Hz"
              f"   (튜닝 기준 {REF_HZ:.0f} Hz)")
        worst = 1.0 / ts[-1]
        print(f"             최악 {ts[-1] * 1000:.1f} ms  ->  {worst:.1f} Hz")
        print()
        if hz < REF_HZ * 0.7:
            print("  이 속도는 회피에 직접 영향을 줍니다. 제어 주기가 절반이면")
            print("  같은 고깔에 대해 반응 거리도 절반이 됩니다 — '잘 피하던")
            print("  고깔에 부딪힌다'의 전형적인 원인입니다.")
            print()
            print("  대응: PC_SPEED_MAX 를 낮춰 속도를 주기에 맞추세요.")
            print(f"        예) {3.0 * hz / REF_HZ:.1f} 정도부터 시험")
        elif hz < REF_HZ:
            print("  기준보다 느리지만 큰 차이는 아닙니다. 속도 상한을 조금")
            print("  낮춰 점수가 좋아지는지 확인해 볼 값어치는 있습니다.")
        else:
            print("  주기는 충분합니다. 버벅임이 보인다면 원인은 다른 곳입니다.")

    # --- frame freshness ---------------------------------------------------
    a = cam()
    time.sleep(0.5)
    b = cam()
    if a is not None and b is not None:
        d = float(np.mean(np.abs(a.astype(np.int16) - b.astype(np.int16))))
        print()
        print(f"정지 상태 연속 프레임 차이 {d:.2f}")
        if d < 0.5:
            print("  프레임이 갱신되고 있습니다 (차가 멈춰 있으므로 정상).")
    return 0


def _find_drive():
    """Make `import drive` work wherever this file was dropped.

    It was written to live in tools/ and reached the package by going one
    directory up, which breaks the moment someone copies it next to the code
    instead — the likeliest thing to happen when the transfer is by hand. So
    the package is searched for rather than assumed: the current directory,
    this file's directory, and one level up from each.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.getcwd(), here,
                 os.path.dirname(os.getcwd()), os.path.dirname(here)):
        if os.path.isdir(os.path.join(cand, "drive")):
            sys.path.insert(0, cand)
            return cand
    return None


if __name__ == "__main__":
    root = _find_drive()
    if root is None:
        print("drive/ 폴더를 찾지 못했습니다.", file=sys.stderr)
        print("drive/ 가 보이는 디렉터리에서 실행하세요 "
              "(예: cd ~/physicar_ws).", file=sys.stderr)
        raise SystemExit(1)
    raise SystemExit(main())
