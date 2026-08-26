#!/usr/bin/env bash
# v5: fixed v4 safety controller + SIM-diversity fine-tuned steering adviser.
# The adviser may change only valid lane steering, within the limits below.
# Cone avoidance, speed, traffic lights, recovery and fail-safe behavior remain
# the fixed v4 implementation.  Missing/broken/OOD adviser input falls back to v4.
#
# SIM smoke test:
#   PC_MAX_SECONDS=30 source /home/physicar/physicar_ws/run_real_upgrade_v5.sh
# Real vehicle (explicit approval required):
#   PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real_upgrade_v5.sh

if [ "${PC_REAL_CONFIRM:-}" != "YES" ] && [ "${PC_TARGET:-}" != "sim" ]; then
  echo "v5 실차 주행은 시작하지 않았습니다. 다음처럼 명시적으로 실행하세요:"
  echo "  PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real_upgrade_v5.sh"
  return 2 2>/dev/null || exit 2
fi

_pc_v5_src="${BASH_SOURCE[0]:-$0}"
_pc_v5_dir="$(cd "$(dirname "$_pc_v5_src")" >/dev/null 2>&1 && pwd)"
: "${PC_FIELD_ADVISER_MODEL:=$_pc_v5_dir/models/v5_sim_adviser.onnx}"
: "${PC_FIELD_ADVISER_BLEND:=0.20}"
: "${PC_FIELD_ADVISER_DELTA_DEG:=2.5}"
# Fixed v4 validated defaults.
: "${PC_SPEED_MAX:=1.30}"
: "${PC_SPEED_MIN:=0.45}"
: "${PC_CONE_SLOW_MAX:=0.80}"
: "${PC_CONE_SLOW_MIN:=0.45}"
: "${PC_CONE_EXIT_HOLD_M:=0.11}"
: "${PC_CONE_EXIT_HOLD_S:=0}"
: "${PC_TELEMETRY_CSV:=/tmp/real-upgrade-v5_$(date +%Y%m%d_%H%M%S).csv}"
export PC_FIELD_ADVISER_MODEL PC_FIELD_ADVISER_BLEND PC_FIELD_ADVISER_DELTA_DEG
export PC_SPEED_MAX PC_SPEED_MIN PC_CONE_SLOW_MAX PC_CONE_SLOW_MIN
export PC_CONE_EXIT_HOLD_M PC_CONE_EXIT_HOLD_S PC_TELEMETRY_CSV

echo "[upgrade-v5] 고정 v4 + SIM 다양성 파인튜닝 보정기"
echo "[upgrade-v5] 모델: $PC_FIELD_ADVISER_MODEL"
echo "[upgrade-v5] 보정 한계: blend=$PC_FIELD_ADVISER_BLEND, delta=${PC_FIELD_ADVISER_DELTA_DEG}°"
echo "[upgrade-v5] 모델·메타데이터·OOD 게이트 실패 시 고정 v4로 폴백합니다."

PC_ENTRY=autodrive_v5.py source "$_pc_v5_dir/t_run.sh"
