"""레이싱 컨트롤러의 조정값 모음."""

CONTROL_HZ = 20.0
WHEELBASE_M = 0.18
STEER_LIMIT_RAD = 0.349

# 경로 추종
LOOKAHEAD_MIN_M = 0.38
LOOKAHEAD_SPEED_GAIN = 0.20
LOOKAHEAD_MAX_M = 0.72

# 속도 계획
SPEED_MAX_MPS = 1.45
SPEED_MIN_MPS = 0.42
MAX_LATERAL_ACCEL = 1.15
CURVATURE_PREVIEW_M = 0.65

# 콘 회피: 기준 주행에서 cone3·cone8 충돌이 발생해, 더 일찍·넓게 회피한다.
AVOID_START_M = 1.35
AVOID_PEAK_M = 0.58
AVOID_OFFSET_M = 0.34
AVOID_SPEED_MPS = 0.58
# 차체가 콘을 완전히 지난 뒤에만 경로 중심으로 복귀한다.
# 회피 오프셋은 콘 통과 직후 다음 제어 주기에서 해제한다.
AVOID_CLEAR_M = 0.0

# cone3는 중심선에 거의 걸쳐 있고 직전 굴곡 때문에 조향 한계에 닿았다.
# 이 콘에서만 더 먼 거리부터, 더 낮은 속도로 회피한다.
CONE_OVERRIDES = {
    "cone3": {"start_m": 1.80, "offset_m": 0.38, "speed_mps": 0.42},
    # 3차에서 cone3는 통과했지만 cone8이 다시 충돌했다. cone8도 조기 진입을
    # 적용하되, 불필요한 랩타임 손실을 막기 위해 cone3보다 덜 보수적으로 둔다.
    # cone8은 진입 직전 제동 지연 때문에 3회 중 2회 접촉했다. 회피 방향은
    # 유지하되, 조향은 1.35 m부터 시작하고 감속만 2.10 m부터 선행한다.
    # cone8은 S자 진입 직후다. 과도한 조기 회피는 안쪽 차선까지 침범하므로,
    # 검증된 완만한 진입을 유지한다. 콘의 경로 인덱스를 지난 뒤에도 회피 목표를
    # 잠시 유지해, 조기 중심선 복귀로 차체 뒤쪽이 콘을 스치는 것을 막는다.
    "cone8": {"start_m": 2.20, "slow_start_m": 2.20, "full_offset_at_m": 0.95,
              "offset_m": 0.42, "speed_mps": 0.45, "clear_m": 0.70},
}

# 시작 신호와 종료
GREEN_TIMEOUT_S = 20.0
MAX_RUNTIME_S = 175.0
