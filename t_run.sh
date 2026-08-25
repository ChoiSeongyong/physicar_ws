# PhysiCar 대회 진입점 — 관계자가 출발 직전 1회 실행합니다.
#
#     source /home/physicar/physicar_ws/run.sh
#
# 규정상 이 스크립트는 `source` 됩니다. 즉 관계자의 셸 안에서 그대로 실행되므로,
# `exit`나 `set -e`를 그냥 쓰면 관계자의 셸이 죽습니다. 그래서 본문 전체를
# 서브셸 `( ... )` 안에 가둡니다 — 디렉터리 이동·trap·변수·종료가 바깥으로
# 새지 않습니다.
#
# 하는 일:
#   1. 워크스페이스 경로를 스스로 찾아 PYTHONPATH 설정
#   2. autodrive.py 실행 (신호등 대기 → 주행)
#   3. 죽으면 다시 띄움 — 도전 기회가 1회뿐이고 재부팅은 기록 무효이므로,
#      프로세스가 영구히 죽는 것이 최악입니다
#   4. 어떤 경로로 끝나든 차를 반드시 정지시킴
#   5. 전 과정을 로그 파일로 남김 (주행 후 원인 분석용)
#
# 테스트용 환경변수 (실주행에서는 설정하지 않습니다):
#   PC_MAX_SECONDS=60   지정 시간 후 자동 종료
#   PC_LOG=/path/log    로그 파일 위치

(
  set -u

  # `source` 되었을 때와 직접 실행되었을 때 모두에서 스크립트 위치를 찾습니다.
  _pc_src="${BASH_SOURCE[0]:-$0}"
  PC_WS="$(cd "$(dirname "$_pc_src")" >/dev/null 2>&1 && pwd)"
  PC_LOG="${PC_LOG:-/tmp/physicar_run_$(date +%Y%m%d_%H%M%S).log}"
  PC_PIDFILE="${PC_PIDFILE:-/tmp/physicar_autodrive.pid}"
  PC_PY="${PC_PY:-python3}"
  # 기본은 기존 센서 컨트롤러지만, run_real.sh는 race/run.py를 명시한다.
  # 상대 경로는 워크스페이스 루트(PC_WS) 기준이다.
  PC_ENTRY="${PC_ENTRY:-autodrive.py}"

  # 인터넷이 없는 환경이 전제입니다. 파이썬이 런타임에 무언가 받으러 나가지
  # 않도록 못 박고, 출력이 버퍼에 갇혀 로그가 비는 일이 없게 합니다.
  export PYTHONPATH="$PC_WS${PYTHONPATH:+:$PYTHONPATH}"
  export PYTHONUNBUFFERED=1
  export PYTHONDONTWRITEBYTECODE=1
  export YOLO_OFFLINE=1

  # 주행 프로세스를 확실히 종료시킵니다.
  #
  # 이걸 빠뜨리면 run.sh 를 세워도 차가 계속 굴러갑니다. 실제로 그랬습니다:
  # 이전 판은 `python3 ... | tee` 파이프였고, `$!` 가 파이썬이 아니라 파이프라인
  # 서브셸을 가리켜서 부모 bash 를 죽여도 autodrive 가 고아로 살아남았습니다.
  # (평가 러너의 stop 을 눌러도 마찬가지였습니다.) 그래서 파이프를 걷어내고
  # 파이썬 PID 를 직접 잡습니다.
  _pc_kill_child() {
    [ -n "${_pc_pid:-}" ] || return 0
    kill -0 "$_pc_pid" 2>/dev/null || return 0
    kill -TERM "$_pc_pid" 2>/dev/null || true
    _pc_i=0
    while kill -0 "$_pc_pid" 2>/dev/null && [ "$_pc_i" -lt 25 ]; do
      sleep 0.2
      _pc_i=$((_pc_i + 1))
    done
    kill -KILL "$_pc_pid" 2>/dev/null || true
  }

  # 차를 세우는 일에는 단일 실패점을 두지 않습니다. curl 우선(파이썬이 이미
  # 죽어 있어도 동작), 없으면 파이썬 표준 라이브러리로 대체합니다.
  _pc_stop() {
    _pc_kill_child
    if command -v curl >/dev/null 2>&1; then
      for _pc_ep in speed steering; do
        curl -s -m 2 -X POST -H 'Content-Type: application/json' \
          -d '{"value":0}' "http://localhost/$_pc_ep" >/dev/null 2>&1 || true
      done
    else
      "$PC_PY" - <<'PYSTOP' >/dev/null 2>&1 || true
import json, urllib.request
for ep in ("speed", "steering"):
    try:
        urllib.request.urlopen(urllib.request.Request(
            f"http://localhost/{ep}", data=json.dumps({"value": 0}).encode(),
            headers={"Content-Type": "application/json"}, method="POST"),
            timeout=2)
    except Exception:
        pass
PYSTOP
    fi
  }
  trap '_pc_stop' INT TERM EXIT

  {
    echo "=========================================================="
    echo " PhysiCar run.sh"
    echo " 시각      : $(date '+%Y-%m-%d %H:%M:%S')"
    echo " 워크스페이스: $PC_WS"
    echo " 로그      : $PC_LOG"
    echo "=========================================================="
  } | tee -a "$PC_LOG"

  if [ ! -f "$PC_WS/$PC_ENTRY" ]; then
    echo "오류: $PC_WS/$PC_ENTRY 가 없습니다. 주행을 시작하지 않습니다." \
      | tee -a "$PC_LOG"
  else
    # 시작 전에 남아 있는 주행 프로세스를 정리합니다.
    #
    # 평가 러너는 자기가 띄운 프로세스 그룹만 죽입니다(killpg). 앞선 실행이
    # 남긴 autodrive 는 다른 그룹이라 살아남고, 그러면 컨트롤러 둘이 동시에
    # 같은 차에 명령을 쏩니다 — 실제로 stop 을 눌러도 차가 0.41 m/s 로 계속
    # 굴러가는 것을 확인했습니다. 도전 기회가 1회뿐이므로 여기서 못 박습니다.
    #
    # PID 파일로 특정합니다. `pgrep -f autodrive.py` 는 절대 쓰지 않습니다 —
    # 명령줄에 그 문자열이 들어간 아무 프로세스나 잡아서, 실제로 그 이름을
    # 출력하던 테스트 셸을 죽였습니다. 죽일 대상은 정확히 알고 있어야 합니다.
    if [ -f "$PC_PIDFILE" ]; then
      _pc_old="$(cat "$PC_PIDFILE" 2>/dev/null || true)"
      case "$_pc_old" in
        ''|*[!0-9]*) _pc_old="" ;;
      esac
      # 그 PID 가 정말 우리 프로그램인지 확인한 뒤에만 신호를 보냅니다.
      if [ -n "$_pc_old" ] && [ -r "/proc/$_pc_old/cmdline" ] &&
         tr '\0' ' ' < "/proc/$_pc_old/cmdline" | grep -q 'autodrive\.py'; then
        echo "경고: 이전 주행 프로세스($_pc_old) 정리" | tee -a "$PC_LOG"
        kill -TERM "$_pc_old" 2>/dev/null || true
        sleep 1
        kill -KILL "$_pc_old" 2>/dev/null || true
      fi
      rm -f "$PC_PIDFILE"
    fi

    # 최대 재시작 횟수. 무한 재시작은 즉시 죽는 버그를 붙잡고 도는 낭비가 되고,
    # 0회는 한 번의 예외로 도전 기회를 날립니다.
    _pc_max_restarts="${PC_MAX_RESTARTS:-5}"
    _pc_try=0
    cd "$PC_WS" || true
    while : ; do
      _pc_try=$((_pc_try + 1))
      echo "--- $PC_ENTRY 실행 (시도 $_pc_try) ---" | tee -a "$PC_LOG"
      # 파이프가 아니라 프로세스 치환으로 로그를 흘립니다. 이래야 `$!` 가
      # 파이썬 자신의 PID 가 되어 trap 이 실제로 주행을 멈출 수 있습니다.
      "$PC_PY" -u "$PC_ENTRY" > >(tee -a "$PC_LOG") 2>&1 &
      _pc_pid=$!
      echo "$_pc_pid" > "$PC_PIDFILE" 2>/dev/null || true
      wait "$_pc_pid"
      _pc_rc=$?
      _pc_pid=""
      rm -f "$PC_PIDFILE" 2>/dev/null || true

      # 130 = Ctrl-C, 143 = SIGTERM. 사람이 세운 것이므로 다시 띄우지 않습니다.
      if [ "$_pc_rc" = "0" ] || [ "$_pc_rc" = "130" ] || [ "$_pc_rc" = "143" ]; then
        echo "autodrive 정상 종료 (rc=$_pc_rc)" | tee -a "$PC_LOG"
        break
      fi
      if [ "$_pc_try" -ge "$_pc_max_restarts" ]; then
        echo "autodrive 재시작 한도($_pc_max_restarts) 도달, 중단 (rc=$_pc_rc)" \
          | tee -a "$PC_LOG"
        break
      fi
      echo "autodrive 비정상 종료 (rc=$_pc_rc). 1초 후 재시작." | tee -a "$PC_LOG"
      _pc_stop
      sleep 1
    done
  fi

  _pc_stop
  rm -f "$PC_PIDFILE" 2>/dev/null || true
  trap - INT TERM EXIT
  echo "run.sh 종료. 로그: $PC_LOG" | tee -a "$PC_LOG"
)
