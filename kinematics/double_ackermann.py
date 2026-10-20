"""
Double Ackermann (Counter-Steer / Four-Wheel Steering) Kinematics

앞뒤 바퀴를 반대 방향으로 조향. 회전 반경이 절반으로 줄어듦.
좁은 공간에서 빠른 방향 전환에 유용.

[원리]
- 앞바퀴: 조향각 +δ
- 뒷바퀴: 조향각 -δ (반대)
- 회전 중심이 차체 중앙(휠베이스의 중점)에 위치
- → ICR이 차체에 더 가까워져 회전 반경이 절반

[기하 비교]
일반 Ackermann:           Double Ackermann:
                                
ICR ←─ R ────              ICR ←─ R/2 ─    ← 회전반경 절반!
                                
       FL ↗                       FL ↗
       │                          │
       │  L                       │  L
       │                          │
       RL │ (고정)                  RL ↘     ← 뒷바퀴도 반대로 조향

[수식 유도]
일반 Ackermann: R = L / tan(δ)  (뒷차축 중심 기준)

Double Ackermann은 회전중심이 차체 중앙(L/2 지점)에 위치.
→ 앞축에서 회전중심까지 거리: L/2
→ R_center = (L/2) / tan(δ)  =  L / (2 tan(δ))
→ 회전반경이 정확히 절반

각 바퀴 조향각 (회전중심이 차체 중앙):
    앞바퀴 (x = +L/2):
        tan(δ_FL) = (L/2) / (R_c - W/2)
        tan(δ_FR) = (L/2) / (R_c + W/2)
    뒷바퀴 (x = -L/2): 회전중심 반대편이므로 부호 반대
        tan(δ_RL) = -(L/2) / (R_c - W/2) = -tan(δ_FL_at_same_y)
        tan(δ_RR) = -(L/2) / (R_c + W/2) = -tan(δ_FR_at_same_y)

[중간 바퀴]
회전중심과 같은 x 위치(차체 중앙)면 R_wheel = |R_c ± W/2|
조향각은 0 (회전중심이 ML/MR을 지나는 직선 위에 있어 조향 불필요)
"""

import math
from typing import List

from .base import (
    KinematicsBase, KinematicsCommand, WheelCommand,
    FL, FR, ML, MR, RL, RR, NUM_WHEELS
)


class DoubleAckermannKinematics(KinematicsBase):
    name = "DoubleAckermann"
    
    def compute(self, throttle: float, steer: float) -> KinematicsCommand:
        throttle, steer = self._normalize_inputs(throttle, steer)
        
        v = throttle * self.max_velocity
        delta = steer * self.max_steer
        L = self.L
        W = self.W
        offset = self.robot.middle_axle_offset
        
        if abs(delta) < 1e-6:
            return self._straight(v)
        
        # 회전 중심이 차체 중앙 (앞축에서 -L/2 떨어진 곳)
        # R_c = 차체 중앙에서 회전중심까지 y방향 거리
        R_c = (L / 2) / math.tan(delta)
        
        # 앞바퀴 조향각 (앞축 x=+L/2, 회전중심까지의 종방향 거리=L/2)
        delta_FL = math.atan2(L/2, R_c - W/2)
        delta_FR = math.atan2(L/2, R_c + W/2)
        delta_FL = self._wrap_steering(delta_FL)
        delta_FR = self._wrap_steering(delta_FR)
        
        # 뒷바퀴는 앞바퀴 부호 반대 (대칭)
        delta_RL = -delta_FL
        delta_RR = -delta_FR
        
        # 각속도 (차체 중심 기준)
        omega = v / R_c
        
        # 회전중심 기준 거리 (각 바퀴 위치는 차체 중심을 원점으로)
        # FL: (+L/2, +W/2), FR: (+L/2, -W/2)
        # ML: (offset, +W/2), MR: (offset, -W/2)
        # RL: (-L/2, +W/2), RR: (-L/2, -W/2)
        # 회전중심: (0, R_c)
        r_FL = math.hypot(L/2, R_c - W/2)
        r_FR = math.hypot(L/2, R_c + W/2)
        r_ML = math.hypot(offset, R_c - W/2)
        r_MR = math.hypot(offset, R_c + W/2)
        r_RL = math.hypot(L/2, R_c - W/2)   # FL과 대칭이므로 같은 거리
        r_RR = math.hypot(L/2, R_c + W/2)
        
        sign = 1 if v >= 0 else -1
        
        wheels: List[WheelCommand] = [None] * NUM_WHEELS
        wheels[FL] = WheelCommand(abs(omega)*r_FL*sign, delta_FL)
        wheels[FR] = WheelCommand(abs(omega)*r_FR*sign, delta_FR)
        wheels[ML] = WheelCommand(abs(omega)*r_ML*sign, 0.0)
        wheels[MR] = WheelCommand(abs(omega)*r_MR*sign, 0.0)
        wheels[RL] = WheelCommand(abs(omega)*r_RL*sign, delta_RL)
        wheels[RR] = WheelCommand(abs(omega)*r_RR*sign, delta_RR)
        
        return KinematicsCommand(wheels=wheels)
    
    def _straight(self, v: float) -> KinematicsCommand:
        return KinematicsCommand(wheels=[
            WheelCommand(velocity=v, steer_angle=0.0) for _ in range(NUM_WHEELS)
        ])
    
    @staticmethod
    def _wrap_steering(angle: float) -> float:
        if angle > math.pi / 2:
            return angle - math.pi
        return angle
