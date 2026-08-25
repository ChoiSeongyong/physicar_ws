#!/usr/bin/env bash
# 실차용 보수적 첫 주행 진입점.
#
# race/run.py가 실차를 감지하면 카메라·LiDAR·오도메트리 기반 컨트롤러로
# 자동 전환한다. 이 파일은 첫 실차 주행용 보수 속도 설정만 제공한다.
#
# 사용 전: 차를 바닥에 놓고, 바퀴가 뜬 상태에서 먼저 카메라/조향을 확인한다.
# 첫 주행(60초 제한):
#   PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real.sh
# 중단: 이 터미널에서 Ctrl+C (t_run.sh의 trap이 속도·조향을 모두 0으로 보낸다).

# 물리적인 차를 실제로 움직이는 명령이므로, 오타나 자동 실행으로 인한 출발을 막는다.
if [ "${PC_REAL_CONFIRM:-}" != "YES" ]; then
  echo "실차 주행은 시작하지 않았습니다. 다음처럼 명시적으로 실행하세요:"
  echo "  PC_REAL_CONFIRM=YES source /home/physicar/physicar_ws/run_real.sh"
  return 2 2>/dev/null || exit 2
fi

# 첫 실차 검증값: 현재 simulator 속도(최대 1.45 m/s)보다 낮게 제한한다.
# 호출 시 환경변수로만 덮어쓸 수 있어, 코드 수정 없이 단계적으로 올릴 수 있다.
: "${PC_SPEED_MAX:=0.65}"
: "${PC_SPEED_MIN:=0.35}"
: "${PC_CONE_SLOW_MAX:=0.55}"
: "${PC_CONE_SLOW_MIN:=0.35}"
: "${PC_MAX_SECONDS:=60}"
: "${PC_WAIT_GREEN:=1}"
: "${PC_COURSE_AUTO:=0}"
: "${PC_CONE_MODEL:=}"
# 빈 값이면 기존과 동일하게 기록하지 않는다. 값을 주면 다음 실차 주행의 센서·명령
# 기록을 CSV로 남겨 속도 상향 같은 고도화를 실제 데이터로 판단할 수 있다.
: "${PC_TELEMETRY_CSV:=}"

export PC_SPEED_MAX PC_SPEED_MIN PC_CONE_SLOW_MAX PC_CONE_SLOW_MIN
export PC_MAX_SECONDS PC_WAIT_GREEN PC_COURSE_AUTO PC_CONE_MODEL PC_TELEMETRY_CSV

_pc_real_src="${BASH_SOURCE[0]:-$0}"
_pc_real_dir="$(cd "$(dirname "$_pc_real_src")" >/dev/null 2>&1 && pwd)"

echo "[real] 실차 보수 시험: max=${PC_SPEED_MAX}m/s, cone=${PC_CONE_SLOW_MAX}m/s, timeout=${PC_MAX_SECONDS}s"
[ -z "$PC_TELEMETRY_CSV" ] || echo "[real] 텔레메트리: $PC_TELEMETRY_CSV"
echo "[real] 신호등·카메라 확인 후 출발합니다. 즉시 중단은 Ctrl+C입니다."
PC_TARGET=real PC_ENTRY=race/run.py source "$_pc_real_dir/t_run.sh"
