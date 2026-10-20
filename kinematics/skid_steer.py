"""
Skid Steer Kinematics

탱크/불도저식 조향. 좌우 바퀴의 속도 차이로만 회전.
스티어링 메커니즘 자체가 없음 (또는 사용 안 함).

[원리]
- 좌측 바퀴 그룹과 우측 바퀴 그룹이 다른 속도로 회전
- 한 쪽이 빠르면 그 반대 방향으로 차체가 회전
- 회전 시 바퀴들이 옆으로 미끄러짐 (그래서 "skid")
- 험지/실내에서 강력. 제자리 회전 가능 (한쪽 정방향, 한쪽 역방향)

[수식]
입력: 차체 선속도 v, 차체 각속도 ω
좌측 바퀴 속도:  v_left  = v - ω · W/2
우측 바퀴 속도:  v_right = v + ω · W/2

여기서 W는 좌우 바퀴 거리(track).

정규화 입력으로 표현:
    v = throttle · v_max
    ω = steer · ω_max
    (ω_max는 max_yaw_rate)

[6륜 처리]
좌측 3개(FL/ML/RL) 같은 속도, 우측 3개(FR/MR/RR) 같은 속도.
조향각은 모두 0.
"""

from typing import List

from .base import (
    KinematicsBase, KinematicsCommand, WheelCommand,
    FL, FR, ML, MR, RL, RR, NUM_WHEELS
)


class SkidSteerKinematics(KinematicsBase):
    name = "SkidSteer"
    
    def __init__(self, robot):
        super().__init__(robot)
        self.max_yaw_rate = robot.motion.max_yaw_rate  # [rad/s]
    
    def compute(self, throttle: float, steer: float) -> KinematicsCommand:
        throttle, steer = self._normalize_inputs(throttle, steer)
        
        # 차체 선속도와 각속도
        v = throttle * self.max_velocity         # [m/s]
        omega = steer * self.max_yaw_rate * 2.0  # [rad/s]
        
        # 좌우 그룹 속도
        # 좌회전(steer>0)일 때 우측이 빨라야 함 → ω·W/2를 우측에 더함
        v_left  = v - omega * self.W / 2
        v_right = v + omega * self.W / 2
        
        # 최대속도 초과 방지: 비율 유지하며 스케일 다운
        max_abs = max(abs(v_left), abs(v_right))
        if max_abs > self.max_velocity:
            scale = self.max_velocity / max_abs
            v_left *= scale
            v_right *= scale
        
        wheels: List[WheelCommand] = [None] * NUM_WHEELS
        wheels[FL] = WheelCommand(velocity=v_left,  steer_angle=0.0)
        wheels[ML] = WheelCommand(velocity=v_left,  steer_angle=0.0)
        wheels[RL] = WheelCommand(velocity=v_left,  steer_angle=0.0)
        wheels[FR] = WheelCommand(velocity=v_right, steer_angle=0.0)
        wheels[MR] = WheelCommand(velocity=v_right, steer_angle=0.0)
        wheels[RR] = WheelCommand(velocity=v_right, steer_angle=0.0)
        
        return KinematicsCommand(wheels=wheels)
