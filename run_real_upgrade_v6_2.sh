#!/usr/bin/env bash
# v6-2: v4 복제본 + 직선 조향 안정화 강화 후보.
#
# 직선에서만 히스테리시스·느린 EMA·soft gain·조향 반전 확인을 적용한다.
# 커브, 복구, 차선 미검출, 콘 회피·콘 통과 hold는 v4 명령을 그대로 사용한다.
#
# SIM smoke test:
#   PC_TARGET=sim PC_MAX_SECONDS=30 source ./run_real_upgrade_v6_2.sh
# Real vehicle (explicit approval required):
#   PC_REAL_CONFIRM=YES source ./run_real_upgrade_v6_2.sh

if [ "${PC_REAL_CONFIRM:-}" != "YES" ] && [ "${PC_TARGET:-}" != "sim" ]; then
  echo "v6-2 실차 주행은 시작하지 않았습니다. 다음처럼 명시적으로 실행하세요:"
  echo "  PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real_upgrade_v6_2.sh"
  return 2 2>/dev/null || exit 2
fi

_pc_v6_src="${BASH_SOURCE[0]:-$0}"
_pc_v6_dir="$(cd "$(dirname "$_pc_v6_src")" >/dev/null 2>&1 && pwd)"

# v4 안전 제어 로직은 그대로 두되, v6-1 실차 후보의 기본 운용 속도는
# 저속 스모크 기준으로 둔다. 호출 시 환경변수를 주면 그 값이 우선한다.
: "${PC_SPEED_MAX:=0.80}"
: "${PC_SPEED_MIN:=0.40}"
: "${PC_CONE_SLOW_MAX:=0.65}"
: "${PC_CONE_SLOW_MIN:=0.38}"
: "${PC_CONE_EXIT_HOLD_M:=0.11}"
: "${PC_CONE_EXIT_HOLD_S:=0}"

# v6-2 직선 전용 안정화값. 필요 시 호출 시점에만 override한다.
: "${PC_STRAIGHT_STABILIZE:=1}"
: "${PC_STRAIGHT_ENTER_FRAMES:=4}"
: "${PC_STRAIGHT_EXIT_FRAMES:=2}"
: "${PC_STRAIGHT_DEADBAND_DEG:=1.0}"
: "${PC_STRAIGHT_EMA_ALPHA:=0.24}"
: "${PC_STRAIGHT_GAIN:=0.68}"
: "${PC_STRAIGHT_SLEW_DEG:=1.1}"
: "${PC_STRAIGHT_REVERSE_FRAMES:=3}"
: "${PC_STRAIGHT_VIEW_ENTER:=0.70}"
: "${PC_STRAIGHT_VIEW_HOLD:=0.62}"
: "${PC_STRAIGHT_VIEW_EXIT_NOW:=0.54}"
: "${PC_STRAIGHT_SLOPE_ENTER:=0.075}"
: "${PC_STRAIGHT_SLOPE_HOLD:=0.12}"
: "${PC_STRAIGHT_SLOPE_EXIT_NOW:=0.16}"
: "${PC_STRAIGHT_STEER_ENTER:=6.5}"
: "${PC_STRAIGHT_STEER_HOLD:=8.5}"
: "${PC_STRAIGHT_STEER_EXIT_NOW:=10.0}"
: "${PC_TELEMETRY_CSV:=/tmp/real-upgrade-v6_2_$(date +%Y%m%d_%H%M%S).csv}"

export PC_SPEED_MAX PC_SPEED_MIN PC_CONE_SLOW_MAX PC_CONE_SLOW_MIN
export PC_CONE_EXIT_HOLD_M PC_CONE_EXIT_HOLD_S PC_TELEMETRY_CSV
export PC_STRAIGHT_STABILIZE PC_STRAIGHT_ENTER_FRAMES PC_STRAIGHT_EXIT_FRAMES
export PC_STRAIGHT_DEADBAND_DEG PC_STRAIGHT_EMA_ALPHA PC_STRAIGHT_GAIN
export PC_STRAIGHT_SLEW_DEG PC_STRAIGHT_REVERSE_FRAMES
export PC_STRAIGHT_VIEW_ENTER PC_STRAIGHT_VIEW_HOLD PC_STRAIGHT_VIEW_EXIT_NOW
export PC_STRAIGHT_SLOPE_ENTER PC_STRAIGHT_SLOPE_HOLD PC_STRAIGHT_SLOPE_EXIT_NOW
export PC_STRAIGHT_STEER_ENTER PC_STRAIGHT_STEER_HOLD PC_STRAIGHT_STEER_EXIT_NOW

echo "[upgrade-v6-2] 고정 v4 복제본 + 강화된 직선 조향 안정화"
echo "[upgrade-v6-2] speed=${PC_SPEED_MAX}/${PC_SPEED_MIN}m/s, cone-exit=${PC_CONE_EXIT_HOLD_M}m"
echo "[upgrade-v6-2] straight: enter/exit=${PC_STRAIGHT_ENTER_FRAMES}/${PC_STRAIGHT_EXIT_FRAMES}f, deadband=${PC_STRAIGHT_DEADBAND_DEG}°, ema=${PC_STRAIGHT_EMA_ALPHA}, gain=${PC_STRAIGHT_GAIN}, slew=${PC_STRAIGHT_SLEW_DEG}°/tick, reverse=${PC_STRAIGHT_REVERSE_FRAMES}f"
echo "[upgrade-v6-2] 커브·복구·콘 회피에는 v4 원 명령을 사용합니다. telemetry: $PC_TELEMETRY_CSV"

PC_ENTRY=autodrive_v6_2.py source "$_pc_v6_dir/t_run.sh"
