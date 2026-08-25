#!/usr/bin/env bash
# 실차 고도화 후보 v2 — SIM에서 반복된 "콘을 지난 직후 후미 접촉" 패턴을
# 실차의 카메라·LiDAR 기반 회피 명령에만 반영한다. SIM route/object API는 쓰지 않는다.
#
# v1(speed60)에서 확인된 안전한 저시야 속도와 콘 감속값을 유지한다. 차이는 단 하나:
# 콘의 거리 보정이 끝난 뒤 0.18초 동안 마지막 회피 조향을 선형 감쇠해 유지한다.
# 이 시간은 콘의 정체성이나 맵 좌표가 아니라, 마지막 LiDAR 기반 회피 명령만 사용한다.
#
# 실행 전까지는 절대 차가 움직이지 않는다. 실차에서 실행할 때만:
#   PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real_upgrade_v2.sh
# 중단: 같은 터미널에서 Ctrl+C

if [ "${PC_REAL_CONFIRM:-}" != "YES" ]; then
  echo "실차 고도화 후보 v2는 시작하지 않았습니다. 다음처럼 명시적으로 실행하세요:"
  echo "  PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real_upgrade_v2.sh"
  return 2 2>/dev/null || exit 2
fi

# v1 speed60 기준값. 호출 시 명시한 값이 우선한다.
: "${PC_SPEED_MAX:=0.60}"
: "${PC_SPEED_MIN:=0.32}"
: "${PC_CONE_SLOW_MAX:=0.55}"
: "${PC_CONE_SLOW_MIN:=0.32}"
# 0이면 기존 컨트롤러와 동일하다. 이번 후보는 후미 여유만 0.18초 추가한다.
: "${PC_CONE_EXIT_HOLD_S:=0.18}"
: "${PC_TELEMETRY_CSV:=/tmp/real-upgrade-v2_$(date +%Y%m%d_%H%M%S).csv}"

export PC_SPEED_MAX PC_SPEED_MIN PC_CONE_SLOW_MAX PC_CONE_SLOW_MIN
export PC_CONE_EXIT_HOLD_S PC_TELEMETRY_CSV

echo "[upgrade-v2] 실차 센서 후보: max=${PC_SPEED_MAX}m/s, min=${PC_SPEED_MIN}m/s"
echo "[upgrade-v2] 콘 통과 후 회피조향 감쇠 유지: ${PC_CONE_EXIT_HOLD_S}s"
echo "[upgrade-v2] 텔레메트리: ${PC_TELEMETRY_CSV}"

_pc_upgrade_src="${BASH_SOURCE[0]:-$0}"
_pc_upgrade_dir="$(cd "$(dirname "$_pc_upgrade_src")" >/dev/null 2>&1 && pwd)"
source "$_pc_upgrade_dir/run_real.sh"
