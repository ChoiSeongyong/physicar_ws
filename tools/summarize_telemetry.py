#!/usr/bin/env python3
"""Summarise an optional autodrive telemetry CSV without external packages."""
from __future__ import annotations

import csv
import statistics
import sys
from pathlib import Path


def numbers(rows, name):
    return [float(row[name]) for row in rows if row.get(name, "") != ""]


def main(path: str) -> int:
    file = Path(path)
    with file.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        print("기록된 제어 행이 없습니다.")
        return 1

    elapsed = numbers(rows, "elapsed_s")
    speed = numbers(rows, "speed_cmd")
    steer = numbers(rows, "steer_cmd_deg")
    view = numbers(rows, "view")
    cone_rows = [row for row in rows if int(row["cone_count"] or 0) > 0]
    blind = sum(row["source"] == "none" for row in rows)
    seek = sum(row["source"] == "seek" for row in rows)
    saturated = sum(abs(value) >= 19.9 for value in steer)

    duration = elapsed[-1] - elapsed[0] if len(elapsed) > 1 else 0.0
    hz = (len(rows) - 1) / duration if duration > 0 else 0.0
    print(f"파일: {file}")
    print(f"제어: {len(rows)} ticks, {duration:.1f}s, 평균 {hz:.1f} Hz")
    print(f"속도 명령: 평균 {statistics.mean(speed):.2f} m/s, 최대 {max(speed):.2f} m/s")
    print(f"조향: 최대 {max(abs(value) for value in steer):.1f}°, 한계(≥19.9°) {saturated}/{len(rows)}")
    print(f"가시거리: 중앙값 {statistics.median(view):.2f}, 최저 {min(view):.2f}")
    print(f"차선 상실: none {blind} ticks, seek {seek} ticks")
    print(f"콘 감지: {len(cone_rows)} ticks")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"사용: {Path(sys.argv[0]).name} <telemetry.csv>")
        raise SystemExit(2)
    raise SystemExit(main(sys.argv[1]))
