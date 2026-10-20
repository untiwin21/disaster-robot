"""
Crab Steering Kinematics

앞뒤 바퀴를 같은 방향으로 동일 각도 조향.
차체 방향(yaw)은 그대로 유지하면서 사선/측면으로 이동.

[원리]
- 모든 조향 가능 바퀴가 평행하게 같은 각도로 꺾임
- 차체는 회전 안 하고 직선 평행 이동
- "게걸음(crab walk)" 또는 "lateral translation"
- 좁은 통로에서 측면 정렬 시 유용

[기하]
        조향각 δ                    조향각 δ
         ↗                          ↗
       FL                          FR
        │                            │
        │                            │
       ML                          MR     ← 중간은 조향 안 함
        │                            │
        │                            │
       RL                          RR
         ↗                          ↗
        조향각 δ                    조향각 δ
         
        → 차체 전체가 (cos δ, sin δ) 방향으로 평행 이동

[수식]
모든 (조향 가능) 바퀴: steer_angle = δ = steer · max_steer
모든 바퀴 속도 = v = throttle · v_max (같음)

[적용 조건]
이 모드는 앞뒤 바퀴가 모두 조향 가능해야 의미가 있음.
config.robot.steering.front_only=true이면 사실상 Ackermann과 동일.
이 클래스는 6륜 모두 조향 가능하다고 가정.
"""

from typing import List

from .base import (
    KinematicsBase, KinematicsCommand, WheelCommand,
    FL, FR, ML, MR, RL, RR, NUM_WHEELS
)


class CrabSteerKinematics(KinematicsBase):
    name = "Crab"
    
    def compute(self, throttle: float, steer: float) -> KinematicsCommand:
        throttle, steer = self._normalize_inputs(throttle, steer)
        
        v = throttle * self.max_velocity
        delta = steer * self.max_steer
        
        # 모든 바퀴 동일: 같은 속도, 같은 조향각
        # (중간 바퀴는 조향 메커니즘 없으므로 0 유지 - 설계상 게걸음 모드에선
        #  앞뒤만 꺾임. 단, 미끄러짐 발생 → 험지에서만 권장)
        wheels: List[WheelCommand] = [None] * NUM_WHEELS
        for i in range(NUM_WHEELS):
            if i in (ML, MR):
                # 중간 바퀴는 조향 불가 (기계 설계 가정)
                wheels[i] = WheelCommand(velocity=v, steer_angle=0.0)
            else:
                wheels[i] = WheelCommand(velocity=v, steer_angle=delta)
        
        return KinematicsCommand(wheels=wheels)
