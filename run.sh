#!/usr/bin/env bash
# 공용 진입점: SIM에서는 경로 기반 race/run.py, 실차에서는 동일 run.py가
# 센서 기반 autodrive 컨트롤러로 자동 전환한다.
set -euo pipefail
cd /home/physicar/physicar_ws/race
exec python3 -u run.py
