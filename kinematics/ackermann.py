"""
Ackermann Steering Kinematics

자동차 표준 조향 모델. 앞바퀴만 꺾이고 뒷바퀴는 고정.

[원리]
- 모든 바퀴가 미끄러짐 없이 굴러가려면, 모든 바퀴 회전축의 연장선이
  한 점(ICR, Instantaneous Center of Rotation)에서 만나야 함.
- 회전 시 안쪽 바퀴는 작은 원, 바깥쪽 바퀴는 큰 원을 그림.
- → 안쪽 바퀴를 더 많이 꺾고, 바깥쪽 바퀴를 덜 꺾어야 함.

[기하]
                      ICR
                       •
                      /│\\
                     / │ \\        R_outer = ICR ~ FR
                    /  │  \\       R_inner = ICR ~ FL
                   /   │   \\      R_center = ICR ~ Rear axle center
              δ_FL│    │    │δ_FR
                FL├────┼────┤FR    ← Front axle
                  │    │    │
                  │    │  L │      L = wheelbase
                  │    │    │
                RL├────┼────┤RR    ← Rear axle (회전 중심 라인)
                       │
                       │ W (track)
                       │←──→
                       
[수식]
입력: 조향각 δ (steer × max_steer_angle), 차체 속도 v
회전 반경 (뒷차축 중심 기준):
    R = L / tan(δ)
    
앞바퀴 조향각 (Ackermann 공식):
    tan(δ_inner) = L / (R - W/2)        ← 회전 안쪽
    tan(δ_outer) = L / (R + W/2)        ← 회전 바깥쪽
    
각 바퀴 속도 (회전중심 기준 거리에 비례):
    R_wheel = sqrt(L² + (R ± W/2)²)     ← 앞바퀴
    R_wheel = R ± W/2                    ← 뒷바퀴
    v_wheel = v · R_wheel / R

[중간 바퀴 처리]
6륜의 중간 축은 차체 중앙에 위치. config의 middle_axle_offset 사용.
조향 안 함, 속도만 회전 반경 비례로 조정.
"""

import math
from typing import List

from .base import (
    KinematicsBase, KinematicsCommand, WheelCommand,
    FL, FR, ML, MR, RL, RR, NUM_WHEELS
)


class AckermannKinematics(KinematicsBase):
    name = "Ackermann"
    
    def compute(self, throttle: float, steer: float) -> KinematicsCommand:
        throttle, steer = self._normalize_inputs(throttle, steer)
        
        v = throttle * self.max_velocity   # 차체 중심 속도 [m/s]
        delta = steer * self.max_steer     # 가상 조향각 [rad]
        L = self.L
        W = self.W
        offset = self.robot.middle_axle_offset
        
        # ─── 직진 케이스 ───
        # tan(0)=0이라 R 발산, 분기 처리
        if abs(delta) < 1e-6:
            return self._straight(v)
        
        # ─── 회전 케이스 ───
        # 회전 반경 (뒷차축 중심 기준)
        # delta > 0 (좌회전) → R > 0 → ICR이 좌측
        # delta < 0 (우회전) → R < 0 → ICR이 우측
        R = L / math.tan(delta)
        
        # 좌/우 어느 쪽이 안쪽인지: R 부호로 판단
        # R > 0 (좌회전): 좌측이 안쪽 → R_left = |R| - W/2 (작음)
        # R < 0 (우회전): 우측이 안쪽 → R_right = |R| - W/2
        # 부호 그대로 두고 거리만 abs로 다루면 자연스럽게 처리됨
        
        # 앞바퀴 조향각 (Ackermann 공식)
        # 좌측 바퀴: 회전중심까지 (y=+W/2) 거리 = R - W/2
        # 우측 바퀴: 회전중심까지 (y=-W/2) 거리 = R + W/2
        delta_FL = math.atan2(L, R - W/2)
        delta_FR = math.atan2(L, R + W/2)
        
        # atan2(L, x)는 [0, π] 범위. 우리는 조향각이니 [-π/2, π/2]로 맞춰야 함.
        # x > 0이면 0~π/2, x < 0이면 π/2~π → π 빼서 -π/2~0으로
        delta_FL = self._wrap_steering(delta_FL)
        delta_FR = self._wrap_steering(delta_FR)
        
        # ─── 각 바퀴 속도 계산 ───
        # 회전중심에서 각 바퀴까지의 거리에 비례한 선속도
        # v_wheel = ω · R_wheel,  ω = v / R (차체 중심 기준)
        omega = v / R
        
        # 앞바퀴: 회전중심 기준 거리 = sqrt(L² + (R ± W/2)²)
        r_FL = math.hypot(L, R - W/2)
        r_FR = math.hypot(L, R + W/2)
        
        # 중간바퀴: x=offset, y=±W/2 → 회전중심까지 sqrt(offset² + (R±W/2)²)
        r_ML = math.hypot(offset, R - W/2)
        r_MR = math.hypot(offset, R + W/2)
        
        # 뒷바퀴: y=±W/2만, x=0 → |R ± W/2|
        r_RL = abs(R - W/2)
        r_RR = abs(R + W/2)
        
        # 부호: omega > 0 (좌회전)일 때 모든 바퀴 전진 방향
        sign = 1 if v >= 0 else -1
        speed_sign = sign if omega >= 0 else sign  # 정상적으론 모두 같은 방향
        
        wheels: List[WheelCommand] = [None] * NUM_WHEELS
        wheels[FL] = WheelCommand(velocity=abs(omega) * r_FL * sign, steer_angle=delta_FL)
        wheels[FR] = WheelCommand(velocity=abs(omega) * r_FR * sign, steer_angle=delta_FR)
        wheels[ML] = WheelCommand(velocity=abs(omega) * r_ML * sign, steer_angle=0.0)
        wheels[MR] = WheelCommand(velocity=abs(omega) * r_MR * sign, steer_angle=0.0)
        wheels[RL] = WheelCommand(velocity=abs(omega) * r_RL * sign, steer_angle=0.0)
        wheels[RR] = WheelCommand(velocity=abs(omega) * r_RR * sign, steer_angle=0.0)
        
        return KinematicsCommand(wheels=wheels)
    
    def _straight(self, v: float) -> KinematicsCommand:
        """조향 0인 경우, 모든 바퀴 같은 속도로 직진."""
        return KinematicsCommand(wheels=[
            WheelCommand(velocity=v, steer_angle=0.0) for _ in range(NUM_WHEELS)
        ])
    
    @staticmethod
    def _wrap_steering(angle: float) -> float:
        """atan2 결과 [0, π]를 조향각 [-π/2, π/2]로 변환."""
        if angle > math.pi / 2:
            return angle - math.pi
        return angle
