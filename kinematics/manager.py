"""
Kinematics Manager

여러 조향 모드를 보관하고 런타임에 전환.
사용자가 키 'M'을 누르면 다음 모드로 순환.

조향 모드 목록:
    1. Ackermann      - 자동차식, 안정적, 일반 주행
    2. SkidSteer      - 탱크식, 제자리 회전, 험지
    3. Crab           - 게걸음, 측면 이동
    4. DoubleAckermann - 4륜 조향, 좁은 회전반경
"""

from typing import List

from shared.config import RobotConfig
from .base import KinematicsBase, KinematicsCommand
from .ackermann import AckermannKinematics
from .skid_steer import SkidSteerKinematics
from .crab import CrabSteerKinematics
from .double_ackermann import DoubleAckermannKinematics


class KinematicsManager:
    """조향 모드 관리자."""
    
    def __init__(self, robot: RobotConfig, initial_mode: str = "Ackermann"):
        self.modes: List[KinematicsBase] = [
            AckermannKinematics(robot),
            SkidSteerKinematics(robot),
            CrabSteerKinematics(robot),
            DoubleAckermannKinematics(robot),
        ]
        self.mode_names = [m.name for m in self.modes]
        
        # 초기 모드 설정
        try:
            self.current_idx = self.mode_names.index(initial_mode)
        except ValueError:
            print(f"[Kinematics] 알 수 없는 모드 '{initial_mode}', Ackermann으로 대체")
            self.current_idx = 0
    
    @property
    def current(self) -> KinematicsBase:
        return self.modes[self.current_idx]
    
    @property
    def current_name(self) -> str:
        return self.current.name
    
    def cycle_next(self) -> str:
        """다음 모드로 순환. 새 모드 이름 반환."""
        self.current_idx = (self.current_idx + 1) % len(self.modes)
        return self.current_name
    
    def set_mode(self, name: str) -> bool:
        """이름으로 모드 지정. 성공 시 True."""
        if name in self.mode_names:
            self.current_idx = self.mode_names.index(name)
            return True
        return False
    
    def compute(self, throttle: float, steer: float) -> KinematicsCommand:
        """현재 모드로 변환 수행."""
        return self.current.compute(throttle, steer)
