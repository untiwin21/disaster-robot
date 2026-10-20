"""
PID Controller

Proportional-Integral-Derivative 제어기. 산업계의 80%를 차지하는
가장 널리 쓰이는 제어 알고리즘.

[직관]
목표값(setpoint)과 현재값(measurement) 사이의 오차를 줄이는 출력을 생성.

    error = setpoint - measurement
    output = Kp·error + Ki·∫error dt + Kd·d(error)/dt

[각 항의 역할]
- P (비례): "오차 크면 크게 반응" → 빠른 응답, 그러나 정상상태 오차 남음
- I (적분): "오차 누적되면 더 세게" → 정상상태 오차 제거, 그러나 오버슈트
- D (미분): "오차 변화 빠르면 미리 제동" → 안정성 향상, 그러나 노이즈에 민감

[튜닝 직관]
- Kp만 너무 크면: 진동 (oscillation), 발산
- Ki 너무 크면: 큰 오버슈트, 적분 windup (값 폭발)
- Kd 너무 크면: 측정 노이즈가 미분 항을 통해 증폭

[실전 디테일]
1. Anti-windup: 적분항이 무한정 커지지 않게 클램프
2. Output saturation: 액추에이터 한계 반영
3. Derivative on measurement: 미분을 error 대신 measurement에 적용
   (setpoint 갑자기 바꿀 때 derivative kick 방지)

[적용 예]
- 모터 속도 제어: setpoint=목표 RPM, measurement=현재 RPM, output=PWM
- 조향 제어: setpoint=목표 각도, measurement=현재 각도, output=서보 명령
- 로봇 제자리 회전: setpoint=목표 yaw, measurement=현재 yaw
"""

import time
from dataclasses import dataclass
from typing import Optional

from shared.config import PIDConfig


@dataclass
class PIDState:
    """디버깅/모니터링용 내부 상태"""
    error: float = 0.0
    integral: float = 0.0
    derivative: float = 0.0
    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0
    output: float = 0.0


class PIDController:
    """
    표준 PID with anti-windup + derivative on measurement.
    
    사용:
        pid = PIDController(config.control.velocity_pid)
        output = pid.update(setpoint=1.0, measurement=0.7)
    """
    
    def __init__(self, config: PIDConfig):
        self.kp = config.kp
        self.ki = config.ki
        self.kd = config.kd
        self.integral_limit = config.integral_limit
        self.output_limit = config.output_limit
        
        self._integral = 0.0
        self._prev_measurement: Optional[float] = None
        self._prev_time: Optional[float] = None
        self.state = PIDState()  # 외부 모니터링용
    
    def update(self, setpoint: float, measurement: float,
               dt: Optional[float] = None) -> float:
        """
        한 스텝 업데이트.
        
        Args:
            setpoint: 목표값
            measurement: 현재 측정값
            dt: 이전 호출과의 시간 간격 [s]. None이면 자동 계산.
        
        Returns:
            제어 출력 (output_limit으로 클램프됨)
        """
        # ─── dt 결정 ───
        now = time.monotonic()
        if dt is None:
            if self._prev_time is None:
                dt = 0.0
            else:
                dt = now - self._prev_time
        self._prev_time = now
        
        # 첫 호출 처리 (dt=0이면 적분/미분 0)
        first_call = self._prev_measurement is None
        
        # ─── 오차 계산 ───
        error = setpoint - measurement
        
        # ─── P항 ───
        p_term = self.kp * error
        
        # ─── I항 (anti-windup) ───
        if dt > 0 and not first_call:
            self._integral += error * dt
            # 클램프
            self._integral = max(-self.integral_limit,
                                 min(self.integral_limit, self._integral))
        i_term = self.ki * self._integral
        
        # ─── D항 (derivative on measurement) ───
        # 일반 PID: d(error)/dt = -d(measurement)/dt (setpoint 일정 시)
        # setpoint 갑자기 변하면 error 미분이 임펄스 → 출력 튐 (derivative kick)
        # 해결: measurement만 미분 (부호 반대)
        if first_call or dt <= 0:
            derivative = 0.0
        else:
            derivative = -(measurement - self._prev_measurement) / dt
        d_term = self.kd * derivative
        self._prev_measurement = measurement
        
        # ─── 출력 ───
        output = p_term + i_term + d_term
        # Saturation
        output = max(-self.output_limit, min(self.output_limit, output))
        
        # ─── 상태 기록 ───
        self.state.error = error
        self.state.integral = self._integral
        self.state.derivative = derivative
        self.state.p_term = p_term
        self.state.i_term = i_term
        self.state.d_term = d_term
        self.state.output = output
        
        return output
    
    def reset(self) -> None:
        """적분/이전값 초기화"""
        self._integral = 0.0
        self._prev_measurement = None
        self._prev_time = None
        self.state = PIDState()
