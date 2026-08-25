#!/usr/bin/env python3
"""대회연습 월드용 경로 기반 레이싱 컨트롤러.

시뮬레이터가 공개하는 route/state/object API를 사용한다. 실차용이 아니며,
평가 규칙이 이 API 사용을 허용하는 현재 SIM 월드에서만 사용한다.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import sys
import time
from dataclasses import dataclass
from typing import Any

import requests

# 이 파일은 SIM 평가의 진입점이면서 실차에서도 같은 이름으로 실행된다.
# race/에서 실행하면 작업공간 최상위가 import 경로에 없으므로, 실차용 센서
# 컨트롤러(autodrive.py, drive/)를 불러올 수 있게 명시적으로 추가한다.
WORKSPACE = Path(__file__).resolve().parents[1]
if str(WORKSPACE) not in sys.path:
    sys.path.insert(0, str(WORKSPACE))

from config import (
    AVOID_CLEAR_M,
    AVOID_OFFSET_M,
    AVOID_PEAK_M,
    AVOID_SPEED_MPS,
    AVOID_START_M,
    CONE_OVERRIDES,
    CONTROL_HZ,
    CURVATURE_PREVIEW_M,
    GREEN_TIMEOUT_S,
    LOOKAHEAD_MAX_M,
    LOOKAHEAD_MIN_M,
    LOOKAHEAD_SPEED_GAIN,
    MAX_LATERAL_ACCEL,
    MAX_RUNTIME_S,
    SPEED_MAX_MPS,
    SPEED_MIN_MPS,
    STEER_LIMIT_RAD,
    WHEELBASE_M,
)

BASE_URL = "http://localhost"
SIM_URL = f"{BASE_URL}/sim/api"
RUNNING = True


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Obstacle:
    name: str
    index: int
    lateral_m: float


class Api:
    def __init__(self) -> None:
        self.http = requests.Session()

    def sim_get(self, path: str) -> dict[str, Any]:
        response = self.http.get(f"{SIM_URL}{path}", timeout=2.0)
        response.raise_for_status()
        return response.json()

    def drive(self, speed: float, steering: float) -> None:
        self.http.post(f"{BASE_URL}/steering", json={"value": steering}, timeout=1.0).raise_for_status()
        self.http.post(f"{BASE_URL}/speed", json={"value": speed}, timeout=1.0).raise_for_status()

    def stop(self) -> None:
        try:
            self.drive(0.0, 0.0)
        except requests.RequestException:
            pass


class RouteController:
    def __init__(self, api: Api) -> None:
        route = api.sim_get("/route")
        raw = route["waypoints"]
        self.points = [(float(x), float(y)) for x, y in raw[:-1]]
        if len(self.points) < 20:
            raise RuntimeError("route has too few points")
        self.n = len(self.points)
        self.last_index: int | None = None
        self.obstacles = self._load_obstacles(api)
        self.active_obstacle: Obstacle | None = None

    def _nearest_index(self, x: float, y: float) -> int:
        if self.last_index is None:
            candidates = range(self.n)
        else:
            # 인접한 반대 방향 차선으로 튀지 않도록 이전 진행 위치 주변만 탐색.
            candidates = ((self.last_index + step) % self.n for step in range(-18, 55))
        return min(candidates, key=lambda i: (self.points[i][0] - x) ** 2 + (self.points[i][1] - y) ** 2)

    def _advance(self, index: int, distance: float) -> int:
        travelled, current = 0.0, index
        while travelled < distance:
            nxt = (current + 1) % self.n
            travelled += math.dist(self.points[current], self.points[nxt])
            current = nxt
            if current == index:
                break
        return current

    def _route_distance(self, start: int, end: int) -> float:
        distance, current = 0.0, start
        while current != end:
            nxt = (current + 1) % self.n
            distance += math.dist(self.points[current], self.points[nxt])
            current = nxt
            if current == start:
                return math.inf
        return distance

    def _normal(self, index: int) -> tuple[float, float]:
        before = self.points[(index - 1) % self.n]
        after = self.points[(index + 1) % self.n]
        dx, dy = after[0] - before[0], after[1] - before[1]
        scale = max(math.hypot(dx, dy), 1e-6)
        return -dy / scale, dx / scale

    def _load_obstacles(self, api: Api) -> list[Obstacle]:
        obstacles: list[Obstacle] = []
        for obj in api.sim_get("/objects").get("objects", []):
            if obj.get("type") != "object" or obj.get("static"):
                continue
            origin = obj.get("origin", {})
            x, y = origin.get("x"), origin.get("y")
            if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
                continue
            index = min(range(self.n), key=lambda i: (self.points[i][0] - x) ** 2 + (self.points[i][1] - y) ** 2)
            nx, ny = self._normal(index)
            lateral = (x - self.points[index][0]) * nx + (y - self.points[index][1]) * ny
            obstacles.append(Obstacle(str(obj.get("name", "object")), index, lateral))
        return obstacles

    def _avoidance(self, current: int) -> tuple[float, float, float]:
        # 회피 조향 시작점과 감속 시작점을 분리한다. 급커브 앞의 콘은 조향을 너무
        # 일찍 시작하면 차선 여유를 잃지만, 감속은 더 이르게 해야 재현성이 높다.
        slow_speed = SPEED_MAX_MPS
        for obstacle in self.obstacles:
            override = CONE_OVERRIDES.get(obstacle.name, {})
            slow_start_m = float(override.get("slow_start_m", override.get("start_m", AVOID_START_M)))
            distance = self._route_distance(current, obstacle.index)
            if distance <= slow_start_m:
                slow_speed = min(slow_speed, float(override.get("speed_mps", AVOID_SPEED_MPS)))

        # 목표점은 차량보다 앞에 있다. 따라서 콘에 도달한 직후 회피를 해제하면
        # 목표점이 중심선으로 복귀하면서 차량 뒤쪽/옆의 콘을 스치게 된다. 활성 콘을
        # 일정 경로거리만큼 유지한 뒤에만 다음 콘 탐색으로 넘긴다.
        if self.active_obstacle is not None:
            obstacle = self.active_obstacle
            override = CONE_OVERRIDES.get(obstacle.name, {})
            passed = self._route_distance(obstacle.index, current)
            # 전역 기본값은 기존 동작(즉시 해제)을 유지하고, 필요한 콘만 clear_m로
            # 회피 목표 유지 거리를 지정한다.
            clear_m = float(override.get("clear_m", AVOID_CLEAR_M))
            if passed <= clear_m:
                offset_m = float(override.get("offset_m", AVOID_OFFSET_M))
                speed_mps = float(override.get("speed_mps", AVOID_SPEED_MPS))
                side = -1.0 if obstacle.lateral_m >= 0.0 else 1.0
                return side * offset_m, 0.0, speed_mps
            self.active_obstacle = None

        nearest: tuple[float, Obstacle, float, float, float] | None = None
        for obstacle in self.obstacles:
            override = CONE_OVERRIDES.get(obstacle.name, {})
            start_m = float(override.get("start_m", AVOID_START_M))
            distance = self._route_distance(current, obstacle.index)
            if distance <= start_m and (nearest is None or distance < nearest[0]):
                nearest = (distance, obstacle, start_m,
                           float(override.get("offset_m", AVOID_OFFSET_M)),
                           float(override.get("speed_mps", AVOID_SPEED_MPS)))
        if nearest is None:
            return 0.0, math.inf, slow_speed
        distance, obstacle, start_m, offset_m, speed_mps = nearest
        self.active_obstacle = obstacle
        side = -1.0 if obstacle.lateral_m >= 0.0 else 1.0
        # `override`는 앞선 탐색 루프의 마지막 콘 값을 가리킬 수 있다. 선택된
        # 콘의 설정을 다시 읽어야, cone8 전용 램프가 cone3 등에 섞이지 않는다.
        selected_override = CONE_OVERRIDES.get(obstacle.name, {})
        # 회피 폭을 콘 도달 전에 완성한다. 콘별로 급커브 진입 여유가 다르므로
        # 필요하면 full_offset_at_m를 개별 설정으로 덮어쓴다.
        full_offset_at_m = float(selected_override.get("full_offset_at_m", max(AVOID_PEAK_M, 0.85)))
        ramp = clamp((start_m - distance) / max(start_m - full_offset_at_m, 0.01), 0.0, 1.0)
        return side * offset_m * ramp, distance, min(speed_mps, slow_speed)

    def command(self, pose: dict[str, Any]) -> tuple[float, float, int]:
        x, y, yaw = float(pose["x"]), float(pose["y"]), float(pose["yaw"])
        current = self._nearest_index(x, y)
        self.last_index = current

        probe = self._advance(current, CURVATURE_PREVIEW_M)
        a, b, c = self.points[current], self.points[probe], self.points[self._advance(probe, CURVATURE_PREVIEW_M)]
        heading_a = math.atan2(b[1] - a[1], b[0] - a[0])
        heading_b = math.atan2(c[1] - b[1], c[0] - b[0])
        curvature = abs(wrap(heading_b - heading_a)) / max(CURVATURE_PREVIEW_M, 0.05)
        curve_speed = math.sqrt(MAX_LATERAL_ACCEL / max(curvature, 0.05))

        offset, cone_distance, cone_speed = self._avoidance(current)
        speed = clamp(min(SPEED_MAX_MPS, curve_speed), SPEED_MIN_MPS, SPEED_MAX_MPS)
        # cone_distance가 무한대여도 _avoidance()는 조기 감속 제한을 돌려줄 수 있다.
        # 따라서 거리 유한성 대신 항상 반환된 제한속도를 적용한다.
        speed = min(speed, cone_speed)

        lookahead = clamp(LOOKAHEAD_MIN_M + LOOKAHEAD_SPEED_GAIN * speed, LOOKAHEAD_MIN_M, LOOKAHEAD_MAX_M)
        target_index = self._advance(current, lookahead)
        tx, ty = self.points[target_index]
        nx, ny = self._normal(target_index)
        tx += offset * nx
        ty += offset * ny

        alpha = wrap(math.atan2(ty - y, tx - x) - yaw)
        target_distance = max(math.hypot(tx - x, ty - y), 0.15)
        steering = math.atan2(2.0 * WHEELBASE_M * math.sin(alpha), target_distance)
        return speed, clamp(steering, -STEER_LIMIT_RAD, STEER_LIMIT_RAD), current


def on_signal(_signum: int, _frame: Any) -> None:
    global RUNNING
    RUNNING = False


def wait_for_green(api: Api) -> bool:
    deadline = time.monotonic() + GREEN_TIMEOUT_S
    while RUNNING and time.monotonic() < deadline:
        try:
            state = api.sim_get("/state")
            api.drive(0.0, 0.0)
            if any(light.get("state") == "green" for light in state.get("lights", [])):
                return True
        except requests.RequestException:
            pass
        time.sleep(0.1)
    return False


def _is_real_mode() -> bool:
    """명시한 PC_TARGET 또는 SIM API 유무로 실행 대상을 결정한다.

    실차에는 /sim/api가 없으므로 기본값(auto)은 센서 기반 autodrive로 안전하게
    전환한다. SIM 대회 평가는 PC_TARGET=sim을 지정해 기존 경로 기반 제어를 쓴다.
    """
    target = os.environ.get("PC_TARGET", "auto").strip().lower()
    if target == "real":
        return True
    if target == "sim":
        return False
    try:
        response = requests.get(f"{SIM_URL}/status", timeout=0.5)
        return not response.ok
    except requests.RequestException:
        return True


def run_real() -> int:
    """실차: 카메라·LiDAR·오도메트리만 쓰는 공용 자율주행기를 실행한다."""
    from autodrive import main as sensor_main

    print("[race] real mode — camera/LiDAR/odometry controller", flush=True)
    sensor_main()
    return 0


def main() -> int:
    if _is_real_mode():
        return run_real()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    api = Api()
    try:
        # 평가기는 직전 실행의 속도 명령을 보존할 수 있다. 경로/콘 데이터를 읽는
        # 동안에도 차가 움직이면 빨간불 출발이므로, 가장 먼저 중립 명령을 보낸다.
        api.stop()
        time.sleep(0.25)
        controller = RouteController(api)
        api.stop()
        print(f"[race] route={controller.n} points, obstacles={len(controller.obstacles)}", flush=True)
        print("[race] waiting for green", flush=True)
        if not wait_for_green(api):
            print("[race] no green signal observed; stopping", flush=True)
            return 1
        print("[race] green observed; racing", flush=True)
        deadline = time.monotonic() + MAX_RUNTIME_S
        period = 1.0 / CONTROL_HZ
        while RUNNING and time.monotonic() < deadline:
            started = time.monotonic()
            try:
                state = api.sim_get("/state")
                if not state.get("running") or state.get("switching"):
                    break
                speed, steering, index = controller.command(state["vehicle"])
                api.drive(speed, steering)
                if index % 80 == 0:
                    print(f"[race] wp={index} v={speed:.2f} steer={math.degrees(steering):+.1f}deg", flush=True)
            except (KeyError, TypeError, ValueError, requests.RequestException) as exc:
                print(f"[race] transient error: {exc}", flush=True)
                api.stop()
                time.sleep(0.15)
            time.sleep(max(0.0, period - (time.monotonic() - started)))
        return 0
    finally:
        api.stop()


if __name__ == "__main__":
    sys.exit(main())
