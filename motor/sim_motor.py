"""
Simulated Motor HAL

실제 하드웨어 없이 모터 동작 흉내. 가짜 응답 지연 + 상태 추적.

[기능]
- 명령 받으면 약간의 지연 후 상태 반영 (실제 서보 응답 흉내)
- 콘솔 로깅 (verbose=True 시)
- 실제 차량 상태 추적 (Bicycle dynamics와 연동 가능 - 선택)

[목적]
- 통신/GUI 디버깅용 빠른 테스트
- 시각화에서 "지금 모터에 뭐 갔는지" 확인
- 실제 라파 받기 전에 모든 상위 로직 검증
"""

import time
import math
from typing import List, Optional

from .hal import MotorHAL, MotorState


class SimMotorHAL(MotorHAL):
    """가짜 모터. 콘솔 로그 + 상태 캐싱."""
    
    def __init__(self,
                 response_delay: float = 0.02,
                 verbose: bool = False,
                 log_throttle_hz: float = 2.0):
        """
        Args:
            response_delay: 가짜 명령 응답 지연 [s]. 실제 PWM 갱신 흉내.
            verbose: 매 명령 로그 출력
            log_throttle_hz: verbose 시 초당 최대 로그 횟수 (스팸 방지)
        """
        self._state = MotorState()
        self._response_delay = response_delay
        self._verbose = verbose
        self._log_interval = 1.0 / log_throttle_hz if log_throttle_hz > 0 else 0
        self._last_log_time = 0.0
        
        if self._verbose:
            print("[SimMotorHAL] 초기화 완료 (시뮬레이션 모드)")
    
    def set_wheel_velocities(self, velocities: List[float]) -> None:
        if len(velocities) != 6:
            raise ValueError(f"6개 속도 필요, 받음: {len(velocities)}")
        
        self._state.wheel_velocities = list(velocities)
        self._state.last_update_time = time.monotonic()
        self._maybe_log(f"velocities={[f'{v:+.2f}' for v in velocities]}")
    
    def set_steer_angles(self, angles: List[float]) -> None:
        if len(angles) != 6:
            raise ValueError(f"6개 조향각 필요, 받음: {len(angles)}")
        self._state.steer_angles = list(angles)
        self._state.last_update_time = time.monotonic()
        self._maybe_log(f"steer={[f'{math.degrees(a):+.1f}°' for a in angles]}")
    
    def set_wheel_sizes(self, sizes: List[float]) -> None:
        if len(sizes) != 6:
            raise ValueError(f"6개 사이즈 필요, 받음: {len(sizes)}")
        self._state.wheel_sizes = [max(0.0, min(1.0, s)) for s in sizes]
        self._state.last_update_time = time.monotonic()
        self._maybe_log(f"sizes={[f'{s:.2f}' for s in sizes]}")
    
    def emergency_stop(self) -> None:
        self._state.wheel_velocities = [0.0] * 6
        self._state.steer_angles = [0.0] * 6
        if self._verbose:
            print("[SimMotorHAL] EMERGENCY STOP")
    
    def shutdown(self) -> None:
        self.emergency_stop()
        if self._verbose:
            print("[SimMotorHAL] 종료")
    
    def get_state(self) -> MotorState:
        return self._state
    
    def _maybe_log(self, msg: str) -> None:
        """로그 throttling - 너무 자주 출력 안 하게"""
        if not self._verbose:
            return
        now = time.monotonic()
        if now - self._last_log_time >= self._log_interval:
            print(f"[SimMotorHAL] {msg}")
            self._last_log_time = now
