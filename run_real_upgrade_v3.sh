#!/usr/bin/env bash
# 실차 고도화 후보 v3 — v2의 "콘 통과 직후 후미 접촉" 방지를 거리 기준으로 바꾼다.
#
# v2는 마지막 LiDAR 회피조향을 고정 시간(0.18초) 동안 유지했다. 하지만 콘 감속으로
# 실제 속도가 달라지면 같은 시간에 확보되는 후미 이동거리도 달라진다. v3는 카메라,
# LiDAR와 이미 명령한 속도만 사용해 회피 종료 뒤 약 0.11m를 진행할 때까지 조향을
# 선형 감쇠한다. SIM route/object/pose 정보는 사용하지 않는다.
#
# 실행 전까지는 절대 차가 움직이지 않는다. 실차에서 실행할 때만:
#   PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real_upgrade_v3.sh
# 중단: 같은 터미널에서 Ctrl+C

if [ "${PC_REAL_CONFIRM:-}" != "YES" ]; then
  echo "실차 고도화 후보 v3는 시작하지 않았습니다. 다음처럼 명시적으로 실행하세요:"
  echo "  PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real_upgrade_v3.sh"
  return 2 2>/dev/null || exit 2
fi

# v2에서 검증한 보수 속도/콘 감속값을 그대로 사용한다. 호출 시 명시한 값이 우선한다.
: "${PC_SPEED_MAX:=0.60}"
: "${PC_SPEED_MIN:=0.32}"
: "${PC_CONE_SLOW_MAX:=0.55}"
: "${PC_CONE_SLOW_MIN:=0.32}"
# 0.11m는 v2의 0.18초가 최대 0.60m/s에서 만들던 약 10.8cm와 동등하다.
# 거리 값이 있으면 v2의 시간 값보다 우선한다.
: "${PC_CONE_EXIT_HOLD_M:=0.11}"
: "${PC_CONE_EXIT_HOLD_S:=0}"
: "${PC_TELEMETRY_CSV:=/tmp/real-upgrade-v3_$(date +%Y%m%d_%H%M%S).csv}"

export PC_SPEED_MAX PC_SPEED_MIN PC_CONE_SLOW_MAX PC_CONE_SLOW_MIN
export PC_CONE_EXIT_HOLD_M PC_CONE_EXIT_HOLD_S PC_TELEMETRY_CSV

echo "[upgrade-v3] 실차 센서 후보: max=${PC_SPEED_MAX}m/s, min=${PC_SPEED_MIN}m/s"
echo "[upgrade-v3] 콘 통과 후 회피조향 감쇠 유지: ${PC_CONE_EXIT_HOLD_M}m"
echo "[upgrade-v3] 텔레메트리: ${PC_TELEMETRY_CSV}"

_pc_upgrade_src="${BASH_SOURCE[0]:-$0}"
_pc_upgrade_dir="$(cd "$(dirname "$_pc_upgrade_src")" >/dev/null 2>&1 && pwd)"
source "$_pc_upgrade_dir/run_real.sh"
