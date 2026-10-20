"""
Kinematics Base

조향 알고리즘들의 공통 인터페이스.

[입력]   고수준 명령 (throttle, steer ∈ [-1, 1])
[출력]   각 바퀴의 (속도, 조향각)

좌표계 (ISO 8855 자동차 좌표계):
    x: 전방 (+)
    y: 좌측 (+)
    yaw(ψ): 반시계 방향 (+)
    
바퀴 인덱스:
    FL(0) ─────── FR(1)        F(Front)/M(Middle)/R(Rear)
      │              │           L(Left)/R(Right)
    ML(2) ─────── MR(3)
      │              │
    RL(4) ─────── RR(5)
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List
import math

from shared.config import RobotConfig


# 바퀴 인덱스 상수
FL, FR = 0, 1   # Front Left, Front Right
ML, MR = 2, 3   # Middle Left, Middle Right
RL, RR = 4, 5   # Rear Left, Rear Right
NUM_WHEELS = 6

WHEEL_NAMES = ["FL", "FR", "ML", "MR", "RL", "RR"]


@dataclass
class WheelCommand:
    """단일 바퀴에 대한 명령"""
    velocity: float          # [m/s] 선속도 (바퀴 접지점 기준)
    steer_angle: float       # [rad] 조향각 (0이면 직진, 양수면 좌회전 방향)
    
    def __repr__(self) -> str:
        return f"v={self.velocity:+.3f}m/s, δ={math.degrees(self.steer_angle):+6.1f}°"


@dataclass
class KinematicsCommand:
    """6개 바퀴 전체 명령 묶음"""
    wheels: List[WheelCommand]
    
    def __post_init__(self):
        if len(self.wheels) != NUM_WHEELS:
            raise ValueError(f"바퀴 개수는 {NUM_WHEELS}개여야 합니다. 받은: {len(self.wheels)}")
    
    def __getitem__(self, idx: int) -> WheelCommand:
        return self.wheels[idx]
    
    def pretty(self) -> str:
        lines = []
        for i, w in enumerate(self.wheels):
            lines.append(f"  {WHEEL_NAMES[i]}: {w}")
        return "\n".join(lines)


class KinematicsBase(ABC):
    """
    조향 알고리즘의 추상 베이스.
    
    하위 클래스가 구현할 것:
        - compute(throttle, steer) -> KinematicsCommand
    
    공통 기능:
        - robot config 보관
        - 입력 정규화/클리핑
        - 회전 반경 계산 헬퍼
    """
    
    name: str = "base"
    
    def __init__(self, robot: RobotConfig):
        self.robot = robot
        self.L = robot.wheelbase           # 휠베이스 [m]
        self.W = robot.track               # 트랙 [m]
        self.max_steer = robot.steering.max_angle      # 최대 조향각 [rad]
        self.max_velocity = robot.motion.max_velocity  # 최대 속도 [m/s]
    
    @abstractmethod
    def compute(self, throttle: float, steer: float) -> KinematicsCommand:
        """
        Args:
            throttle: 정규화 가속/감속 입력 [-1, 1]
            steer:    정규화 조향 입력      [-1, 1]
                     (좌회전 양수 vs 우회전 양수는 좌표계 따라감 - 여기선 좌회전 양수)
        Returns:
            KinematicsCommand: 6개 바퀴 각각의 (속도, 조향각)
        """
        pass
    
    @staticmethod
    def _clamp(value: float, lo: float = -1.0, hi: float = 1.0) -> float:
        return max(lo, min(hi, value))
    
    def _normalize_inputs(self, throttle: float, steer: float) -> tuple[float, float]:
        """입력값을 [-1, 1]로 클리핑"""
        return self._clamp(throttle), self._clamp(steer)
