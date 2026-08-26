#!/usr/bin/env bash
# v6-1: v4를 복제한 센서 제어기 + 실차 직선 조향 안정화 후보.
#
# v4 파일/실행값은 변경하지 않는다. v6-1만 clear straight에서 영상 노이즈로
# 인한 작은 반대 조향을 deadband + EMA + 변화율 제한으로 억제한다. 커브, 복구,
# 차선 미검출, 콘 회피·콘 통과 hold는 v4 명령을 그대로 사용한다.
#
# SIM smoke test:
#   PC_TARGET=sim PC_MAX_SECONDS=30 source ./run_real_upgrade_v6_1.sh
# Real vehicle (explicit approval required):
#   PC_REAL_CONFIRM=YES source ./run_real_upgrade_v6_1.sh

if [ "${PC_REAL_CONFIRM:-}" != "YES" ] && [ "${PC_TARGET:-}" != "sim" ]; then
  echo "v6-1 실차 주행은 시작하지 않았습니다. 다음처럼 명시적으로 실행하세요:"
  echo "  PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real_upgrade_v6_1.sh"
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

# v6-1 직선 전용 안정화값. 필요 시 호출 시점에만 override한다.
: "${PC_STRAIGHT_STABILIZE:=1}"
: "${PC_STRAIGHT_VIEW_MIN:=0.70}"
: "${PC_STRAIGHT_SLOPE_MAX:=0.07}"
: "${PC_STRAIGHT_STEER_MAX:=7.0}"
: "${PC_STRAIGHT_ENTER_FRAMES:=3}"
: "${PC_STRAIGHT_DEADBAND_DEG:=0.75}"
: "${PC_STRAIGHT_EMA_ALPHA:=0.42}"
: "${PC_STRAIGHT_SLEW_DEG:=1.8}"
: "${PC_TELEMETRY_CSV:=/tmp/real-upgrade-v6_1_$(date +%Y%m%d_%H%M%S).csv}"

export PC_SPEED_MAX PC_SPEED_MIN PC_CONE_SLOW_MAX PC_CONE_SLOW_MIN
export PC_CONE_EXIT_HOLD_M PC_CONE_EXIT_HOLD_S PC_TELEMETRY_CSV
export PC_STRAIGHT_STABILIZE PC_STRAIGHT_VIEW_MIN PC_STRAIGHT_SLOPE_MAX
export PC_STRAIGHT_STEER_MAX PC_STRAIGHT_ENTER_FRAMES PC_STRAIGHT_DEADBAND_DEG
export PC_STRAIGHT_EMA_ALPHA PC_STRAIGHT_SLEW_DEG

echo "[upgrade-v6-1] 고정 v4 복제본 + 직선 조향 안정화"
echo "[upgrade-v6-1] v4 값: speed=${PC_SPEED_MAX}/${PC_SPEED_MIN}m/s, cone-exit=${PC_CONE_EXIT_HOLD_M}m"
echo "[upgrade-v6-1] straight: enter=${PC_STRAIGHT_ENTER_FRAMES}f, deadband=${PC_STRAIGHT_DEADBAND_DEG}°, ema=${PC_STRAIGHT_EMA_ALPHA}, slew=${PC_STRAIGHT_SLEW_DEG}°/tick"
echo "[upgrade-v6-1] 커브·복구·콘 회피에는 v4 원 명령을 사용합니다. telemetry: $PC_TELEMETRY_CSV"

PC_ENTRY=autodrive_v6_1.py source "$_pc_v6_dir/t_run.sh"
