#!/usr/bin/env bash
# 실차 고도화 후보 v4 — v3(콘 후미 접촉 방지, 거리기반 조향 감쇠)는 그대로 두고
# 속도 상한만 SIM 섀도우 평가로 재탐색해 올린다.
#
# 배경: tools/shadow_eval.py로 PC_SPEED_MAX를 0.60 → 3.00(하드웨어 절대 상한)까지
# 스윕한 결과, SIM에서는 어느 값에서도 콘 접촉/오프트랙/false start가 전혀
# 발생하지 않았다(각 값 26~90초 컷오프, 총 13회 실행). 커브에서는 조향각 기반
# 감속 로직이 이미 자동으로 속도를 낮추므로 PC_SPEED_MAX는 "직선에서 낼 수 있는
# 상한"만 결정한다.
#
#   PC_SPEED_MAX  lap_time_s(SIM)   cone_hits  offtrack  false_start
#   0.60 (v3)     61.4~62.2         0          0         false
#   0.75          52.9~53.2         0          0         false
#   0.90          46.6~47.1         0          0         false
#   1.10          41.1              0          0         false
#   1.30 (v4 채택) 37.1             0          0         false
#   1.60          32.3              0          0         false
#   2.00          28.7              0          0         false
#   3.00 (하드웨어 상한) 23.8       0          0         false
#
# 그러나 SIM 카메라는 모션 블러/노출 지연이 없고 SIM 노면은 타이어 슬립이 없어
# 고속일수록 실차와의 괴리가 커진다. 따라서 여기서는 "확실히 빠르면서도 과감하지
# 않은" 1.30 m/s를 기본값으로 채택한다(v3 대비 랩타임 -40%). 더 밀어붙이고 싶다면
# 위 표를 참고해 PC_SPEED_MAX 등을 호출 시점에 override할 것 — 단, 그 값들은 SIM
# 에서만 검증되었고 실차 재검증 전에는 신뢰할 수 없다.
#
# 실행 전까지는 절대 차가 움직이지 않는다. 실차에서 실행할 때만:
#   PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real_upgrade_v4.sh
# 중단: 같은 터미널에서 Ctrl+C
#
# 처음 시도하는 경우 반드시 더 낮은 값(예: PC_SPEED_MAX=0.90)으로 먼저 실행해
# 차가 트랙/콘/신호등 로직에 정상 반응하는지 확인한 뒤 올릴 것을 권장한다:
#   PC_REAL_CONFIRM=YES PC_SPEED_MAX=0.90 PC_SPEED_MIN=0.38 \
#     PC_CONE_SLOW_MAX=0.65 PC_CONE_SLOW_MIN=0.38 \
#     source /home/physicar/physicar_ws/run_real_upgrade_v4.sh

if [ "${PC_REAL_CONFIRM:-}" != "YES" ]; then
  echo "실차 고도화 후보 v4는 시작하지 않았습니다. 다음처럼 명시적으로 실행하세요:"
  echo "  PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real_upgrade_v4.sh"
  echo "처음이라면 먼저 낮은 속도로 검증하세요, 예:"
  echo "  PC_REAL_CONFIRM=YES PC_SPEED_MAX=0.90 PC_SPEED_MIN=0.38 \\"
  echo "    PC_CONE_SLOW_MAX=0.65 PC_CONE_SLOW_MIN=0.38 \\"
  echo "    source /home/physicar/physicar_ws/run_real_upgrade_v4.sh"
  return 2 2>/dev/null || exit 2
fi

# SIM 섀도우 스윕(tools/shadow_eval.py)에서 클린 완주가 확인된 v4 기본값.
# 호출 시 명시한 값이 우선한다.
: "${PC_SPEED_MAX:=1.30}"
: "${PC_SPEED_MIN:=0.45}"
: "${PC_CONE_SLOW_MAX:=0.80}"
: "${PC_CONE_SLOW_MIN:=0.45}"
# v3에서 검증한 거리 기반 회피조향 감쇠(속도 무관 0.11m)를 그대로 계승.
: "${PC_CONE_EXIT_HOLD_M:=0.11}"
: "${PC_CONE_EXIT_HOLD_S:=0}"
: "${PC_TELEMETRY_CSV:=/tmp/real-upgrade-v4_$(date +%Y%m%d_%H%M%S).csv}"

export PC_SPEED_MAX PC_SPEED_MIN PC_CONE_SLOW_MAX PC_CONE_SLOW_MIN
export PC_CONE_EXIT_HOLD_M PC_CONE_EXIT_HOLD_S PC_TELEMETRY_CSV

echo "[upgrade-v4] 실차 속도 후보: max=${PC_SPEED_MAX}m/s, min=${PC_SPEED_MIN}m/s"
echo "[upgrade-v4] 콘 감속: max=${PC_CONE_SLOW_MAX}m/s, min=${PC_CONE_SLOW_MIN}m/s"
echo "[upgrade-v4] 콘 통과 후 회피조향 감쇠 유지: ${PC_CONE_EXIT_HOLD_M}m (v3 계승)"
echo "[upgrade-v4] 텔레메트리: ${PC_TELEMETRY_CSV}"
echo "[upgrade-v4] 주의: 이 값은 SIM 섀도우 평가에서만 검증됨(도구: tools/shadow_eval.py)."
echo "[upgrade-v4]       실차 카메라 블러/타이어 슬립은 재현되지 않으므로 첫 주행은 유심히 지켜볼 것."

_pc_upgrade_src="${BASH_SOURCE[0]:-$0}"
_pc_upgrade_dir="$(cd "$(dirname "$_pc_upgrade_src")" >/dev/null 2>&1 && pwd)"
source "$_pc_upgrade_dir/run_real.sh"
