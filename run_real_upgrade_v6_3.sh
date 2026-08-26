#!/usr/bin/env bash
# v6-3: 고정 v4 복제본 + 직선 원명령 통과 + 대각선 turn-in 보조 후보.
#
# 진짜 직선은 v4 원 조향을 그대로 사용한다. 확인된 완만한 대각선에서만
# v4가 이미 고른 방향과 같은 쪽으로 작은 조향을 더한다. 커브, 복구,
# 차선 미검출, 콘 회피·콘 통과 hold는 v4 명령을 그대로 사용한다.
#
# SIM smoke test:
#   PC_TARGET=sim PC_MAX_SECONDS=30 source ./run_real_upgrade_v6_3.sh
# Real vehicle (explicit approval required):
#   PC_REAL_CONFIRM=YES source ./run_real_upgrade_v6_3.sh

if [ "${PC_REAL_CONFIRM:-}" != "YES" ] && [ "${PC_TARGET:-}" != "sim" ]; then
  echo "v6-3 실차 주행은 시작하지 않았습니다. 다음처럼 명시적으로 실행하세요:"
  echo "  PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real_upgrade_v6_3.sh"
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

# v6-3은 직선에 별도 필터를 쓰지 않는다. 대각선 확인 후에만 v4 원명령의
# 같은 방향으로 최대 2.8°를 보태서 turn-in 부족을 줄인다.
: "${PC_DIAGONAL_ASSIST:=1}"
: "${PC_DIAGONAL_ENTER_FRAMES:=3}"
: "${PC_DIAGONAL_EXIT_FRAMES:=2}"
: "${PC_DIAGONAL_SLOPE_ENTER:=0.055}"
: "${PC_DIAGONAL_SLOPE_HOLD:=0.035}"
: "${PC_DIAGONAL_SLOPE_EXIT_NOW:=0.18}"
: "${PC_DIAGONAL_VIEW_MIN:=0.64}"
: "${PC_DIAGONAL_STEER_MIN:=0.50}"
: "${PC_DIAGONAL_STEER_MAX:=9.0}"
: "${PC_DIAGONAL_SLOPE_ALPHA:=0.42}"
: "${PC_DIAGONAL_EXTRA_GAIN:=13.0}"
: "${PC_DIAGONAL_MAX_ASSIST_DEG:=2.8}"
: "${PC_DIAGONAL_SLEW_DEG:=0.9}"
: "${PC_TELEMETRY_CSV:=/tmp/real-upgrade-v6_3_$(date +%Y%m%d_%H%M%S).csv}"

export PC_SPEED_MAX PC_SPEED_MIN PC_CONE_SLOW_MAX PC_CONE_SLOW_MIN
export PC_CONE_EXIT_HOLD_M PC_CONE_EXIT_HOLD_S PC_TELEMETRY_CSV
export PC_DIAGONAL_ASSIST PC_DIAGONAL_ENTER_FRAMES PC_DIAGONAL_EXIT_FRAMES
export PC_DIAGONAL_SLOPE_ENTER PC_DIAGONAL_SLOPE_HOLD PC_DIAGONAL_SLOPE_EXIT_NOW
export PC_DIAGONAL_VIEW_MIN PC_DIAGONAL_STEER_MIN PC_DIAGONAL_STEER_MAX
export PC_DIAGONAL_SLOPE_ALPHA PC_DIAGONAL_EXTRA_GAIN PC_DIAGONAL_MAX_ASSIST_DEG
export PC_DIAGONAL_SLEW_DEG

echo "[upgrade-v6-3] 고정 v4 복제본 + 직선 원명령 통과 + 대각선 turn-in 보조"
echo "[upgrade-v6-3] speed=${PC_SPEED_MAX}/${PC_SPEED_MIN}m/s, cone-exit=${PC_CONE_EXIT_HOLD_M}m"
echo "[upgrade-v6-3] diagonal: enter/exit=${PC_DIAGONAL_ENTER_FRAMES}/${PC_DIAGONAL_EXIT_FRAMES}f, slope=${PC_DIAGONAL_SLOPE_ENTER}/${PC_DIAGONAL_SLOPE_HOLD}, gain=${PC_DIAGONAL_EXTRA_GAIN}, cap=${PC_DIAGONAL_MAX_ASSIST_DEG}°, slew=${PC_DIAGONAL_SLEW_DEG}°/tick"
echo "[upgrade-v6-3] 직선·커브·복구·콘 회피는 v4 원 명령을 사용합니다. telemetry: $PC_TELEMETRY_CSV"

PC_ENTRY=autodrive_v6_3.py source "$_pc_v6_dir/t_run.sh"
