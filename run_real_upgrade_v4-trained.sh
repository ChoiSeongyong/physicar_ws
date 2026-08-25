#!/usr/bin/env bash
# v4-trained: fixed run_real_upgrade_v4.sh를 수정하지 않는 별도 후보.
# dataset/diag로 학습한 보정기는 v4의 차선 조향에만 최대 25%/3°로 섞이며,
# 콘 회피·속도·신호·복구·안전정지는 모두 고정 v4를 그대로 사용한다.
# 모델 또는 신뢰도 게이트가 실패하면 autodrive_trained.py는 즉시 고정 v4로 폴백한다.
#
# 먼저 학습/검증:
#   python3 tools/train_v4_residual.py
# 실차 실행(명시적 승인 필요):
#   PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real_upgrade_v4-trained.sh

if [ "${PC_REAL_CONFIRM:-}" != "YES" ]; then
  echo "학습형 v4는 시작하지 않았습니다. 다음처럼 명시적으로 실행하세요:"
  echo "  PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real_upgrade_v4-trained.sh"
  return 2 2>/dev/null || exit 2
fi

_pc_trained_src="${BASH_SOURCE[0]:-$0}"
_pc_trained_dir="$(cd "$(dirname "$_pc_trained_src")" >/dev/null 2>&1 && pwd)"
: "${PC_FIELD_ADVISER_MODEL:=$_pc_trained_dir/models/v4_field_adviser.onnx}"
: "${PC_FIELD_ADVISER_BLEND:=0.25}"
: "${PC_FIELD_ADVISER_DELTA_DEG:=3.0}"
# Fixed v4's validated defaults (kept here rather than sourcing v4, because v4
# deliberately selects race/run.py as its entry point).
: "${PC_SPEED_MAX:=1.30}"
: "${PC_SPEED_MIN:=0.45}"
: "${PC_CONE_SLOW_MAX:=0.80}"
: "${PC_CONE_SLOW_MIN:=0.45}"
: "${PC_CONE_EXIT_HOLD_M:=0.11}"
: "${PC_CONE_EXIT_HOLD_S:=0}"
: "${PC_TELEMETRY_CSV:=/tmp/real-upgrade-v4-trained_$(date +%Y%m%d_%H%M%S).csv}"
export PC_FIELD_ADVISER_MODEL PC_FIELD_ADVISER_BLEND PC_FIELD_ADVISER_DELTA_DEG
export PC_SPEED_MAX PC_SPEED_MIN PC_CONE_SLOW_MAX PC_CONE_SLOW_MIN
export PC_CONE_EXIT_HOLD_M PC_CONE_EXIT_HOLD_S PC_TELEMETRY_CSV

echo "[upgrade-v4-trained] 고정 v4 + dataset/diag 학습 보정기"
echo "[upgrade-v4-trained] 모델: $PC_FIELD_ADVISER_MODEL"
echo "[upgrade-v4-trained] 보정 한계: blend=$PC_FIELD_ADVISER_BLEND, delta=${PC_FIELD_ADVISER_DELTA_DEG}°"
echo "[upgrade-v4-trained] v4 속도/콘 설정: $PC_SPEED_MAX/$PC_SPEED_MIN, $PC_CONE_SLOW_MAX/$PC_CONE_SLOW_MIN"
echo "[upgrade-v4-trained] 모델 미존재·오류·OOD 이미지에서는 고정 v4로 폴백합니다."

PC_TARGET=real PC_ENTRY=autodrive_trained.py source "$_pc_trained_dir/t_run.sh"
