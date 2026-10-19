"""
Motor Hardware Abstraction Layer (HAL)

같은 인터페이스로 시뮬 모터와 실제 모터를 다룬다.

[Pattern]
    MotorHAL (abstract)
    ├── SimMotorHAL    ← 시뮬용 (콘솔 로그 + 상태 추적)
    └── GpioMotorHAL   ← 실제 라파용 (PCA9685 서보 12개 + BTS7960 DC 좌/우)

[하드웨어 구성 (회로도 기준)]
- DC 모터 6개: BTS7960 듀얼 드라이버, 좌3 병렬 + 우3 병렬 (2채널)
- 조향 서보 6개: PCA9685 채널 6~11 (각 바퀴 개별 조향)
- 변형 서보 6개: PCA9685 채널 0~5 (각 바퀴 사이즈 조절)
- I2C: GPIO 2 (SDA), GPIO 3 (SCL)
- BTS7960: GPIO 18 (PWM_L), 23 (DIR_L), 19 (PWM_R), 24 (DIR_R)

main.py 사용 예시:

    motor = create_motor_hal(config)
    motor.set_wheel_velocities([0.5]*6)         # 6개 [m/s]
    motor.set_steer_angles([0.1]*6)             # 6개 [rad]
    motor.set_wheel_sizes([0.7]*6)              # 6개 [0,1]

[자동 선택]
- config.mode == "simulation" → SimMotorHAL
- config.mode == "real" → GpioMotorHAL (라이브러리 import 시도)
- 라파 라이브러리 import 실패 → 자동 SimMotorHAL fallback
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List
import time


@dataclass
class MotorState:
    """현재 모터 상태 (디버깅/텔레메트리용)"""
    wheel_velocities: List[float] = field(default_factory=lambda: [0.0]*6)  # [m/s] (FL, FR, ML, MR, RL, RR)
    steer_angles: List[float] = field(default_factory=lambda: [0.0]*6)      # [rad] 6개
    wheel_sizes: List[float] = field(default_factory=lambda: [0.0]*6)       # [0,1] 6개
    last_update_time: float = 0.0
    
    @property
    def wheel_size_front(self) -> float:
        """앞 평균 (호환성)"""
        return (self.wheel_sizes[0] + self.wheel_sizes[1]) / 2
    
    @property
    def wheel_size_rear(self) -> float:
        """뒤 평균 (호환성)"""
        return (self.wheel_sizes[4] + self.wheel_sizes[5]) / 2
    
    @property
    def wheel_size_middle(self) -> float:
        """중간 평균"""
        return (self.wheel_sizes[2] + self.wheel_sizes[3]) / 2


class MotorHAL(ABC):
    """모터 제어 추상 인터페이스"""
    
    @abstractmethod
    def set_wheel_velocities(self, velocities: List[float]) -> None:
        """6개 바퀴 각각의 목표 선속도 [m/s].
        
        실제 하드웨어에선 좌3/우3 그룹이라 평균내서 적용.
        시뮬에선 6개 그대로 추적.
        """
        pass
    
    @abstractmethod
    def set_steer_angles(self, angles: List[float]) -> None:
        """6개 바퀴 조향각 [rad]. 0=직진, 양수=좌회전 방향."""
        pass
    
    @abstractmethod
    def set_wheel_sizes(self, sizes: List[float]) -> None:
        """6개 바퀴 사이즈 정규화 [0, 1]."""
        pass
    
    @abstractmethod
    def emergency_stop(self) -> None:
        """모든 모터 즉시 정지"""
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        """리소스 정리"""
        pass
    
    @abstractmethod
    def get_state(self) -> MotorState:
        """현재 상태 (텔레메트리용)"""
        pass
