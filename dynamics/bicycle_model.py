"""
Bicycle Dynamics Model

차량 동역학의 표준 단순화 모델. 좌우 바퀴를 하나로 합쳐서
"앞바퀴 1개 + 뒷바퀴 1개"의 자전거처럼 다룸. 좌우 대칭 가정.

Kinematic bicycle은 미끄러짐 없는 가정,
Dynamic bicycle은 슬립과 타이어 힘을 고려.

[좌표계]
차체 좌표계 (Body frame):
    x: 전방, y: 좌측, ψ(yaw): 반시계 방향
    
상태 변수:
    v_x:  종방향 속도 [m/s]      (구르는 방향)
    v_y:  횡방향 속도 [m/s]      (옆 미끄러짐)
    r(yaw rate): 차체 각속도 [rad/s]

[슬립각 (Slip angle)]
바퀴가 향한 방향(δ)과 실제 속도 벡터 방향 차이.

    앞바퀴: α_f = δ - arctan((v_y + L_f · r) / v_x)
    뒷바퀴: α_r =     -arctan((v_y - L_r · r) / v_x)

    L_f: 차체 중심 ~ 앞축 거리
    L_r: 차체 중심 ~ 뒷축 거리

[운동 방정식 (Newton-Euler in body frame)]
    m·(dv_x/dt - v_y·r) = F_xf·cos(δ) - F_yf·sin(δ) + F_xr
    m·(dv_y/dt + v_x·r) = F_xf·sin(δ) + F_yf·cos(δ) + F_yr
    I_z·(dr/dt)         = L_f·(F_xf·sin(δ) + F_yf·cos(δ)) - L_r·F_yr

    m: 질량, I_z: yaw 관성 모멘트
    F_xf, F_yf: 앞바퀴 종/횡력
    F_xr, F_yr: 뒷바퀴 종/횡력

[월드 좌표계로 변환]
    dx/dt = v_x·cos(ψ) - v_y·sin(ψ)
    dy/dt = v_x·sin(ψ) + v_y·cos(ψ)
    dψ/dt = r
"""

import math
from dataclasses import dataclass

from shared.config import RobotConfig
from .tire_model import PacejkaTireModel


@dataclass
class VehicleState:
    """차량 상태 (월드 좌표계 위치 + 차체 좌표계 속도)"""
    x: float = 0.0      # 월드 x 위치 [m]
    y: float = 0.0      # 월드 y 위치 [m]
    psi: float = 0.0    # 차체 yaw [rad]
    
    v_x: float = 0.0    # 차체 종속도 [m/s]
    v_y: float = 0.0    # 차체 횡속도 [m/s]
    r: float = 0.0      # yaw rate [rad/s]


@dataclass
class VehicleInputs:
    """차량 제어 입력"""
    steer_angle: float    # [rad] 앞바퀴 조향각
    F_drive: float        # [N] 종방향 구동력 (가속력)
                          # 양수: 가속, 음수: 감속/후진


class BicycleDynamics:
    """
    Dynamic bicycle model.
    
    Kinematics와 다른 점:
    - 미끄러짐(slip) 명시적으로 계산
    - 타이어 힘으로 운동 방정식 푸는 정통 방식
    - 고속/험지에서 더 정확
    
    한계:
    - 좌우 대칭 가정 (좌우 바퀴 차이 무시)
    - 6륜 → bicycle로 단순화 (앞축 + 뒷축)
    """
    
    def __init__(self, robot: RobotConfig, tire_model: PacejkaTireModel):
        self.m = robot.mass                       # [kg]
        self.I_z = robot.moment_of_inertia        # [kg·m²]
        self.L = robot.wheelbase                  # [m]
        # 무게 중심이 휠베이스 중앙이라고 가정
        self.L_f = self.L / 2                     # 중심 ~ 앞축 [m]
        self.L_r = self.L / 2                     # 중심 ~ 뒷축 [m]
        self.tire = tire_model
        
        # 수치적 안정화: 매우 저속에서 slip angle 계산 폭발 방지
        self.MIN_SPEED = 0.1  # [m/s]
    
    def step(self, state: VehicleState, inputs: VehicleInputs, dt: float) -> VehicleState:
        """
        한 스텝 진행. 4차 Runge-Kutta 적분 사용.
        
        오일러 적분(가장 단순)도 가능하지만 dt가 크면 발산할 수 있음.
        RK4는 4번 평가하지만 정확도가 훨씬 좋음.
        """
        def deriv(s: VehicleState) -> VehicleState:
            return self._compute_derivatives(s, inputs)
        
        k1 = deriv(state)
        k2 = deriv(self._add_state(state, k1, dt / 2))
        k3 = deriv(self._add_state(state, k2, dt / 2))
        k4 = deriv(self._add_state(state, k3, dt))
        
        # 가중 평균
        new_state = VehicleState(
            x   = state.x   + dt / 6 * (k1.x   + 2*k2.x   + 2*k3.x   + k4.x),
            y   = state.y   + dt / 6 * (k1.y   + 2*k2.y   + 2*k3.y   + k4.y),
            psi = state.psi + dt / 6 * (k1.psi + 2*k2.psi + 2*k3.psi + k4.psi),
            v_x = state.v_x + dt / 6 * (k1.v_x + 2*k2.v_x + 2*k3.v_x + k4.v_x),
            v_y = state.v_y + dt / 6 * (k1.v_y + 2*k2.v_y + 2*k3.v_y + k4.v_y),
            r   = state.r   + dt / 6 * (k1.r   + 2*k2.r   + 2*k3.r   + k4.r),
        )
        return new_state
    
    def _compute_derivatives(self, state: VehicleState, inputs: VehicleInputs) -> VehicleState:
        """상태의 시간 미분(dx/dt, dy/dt, ...) 계산."""
        v_x, v_y, r = state.v_x, state.v_y, state.r
        psi = state.psi
        delta = inputs.steer_angle
        F_drive = inputs.F_drive
        
        # ─── Slip angle 계산 ───
        # 저속에서는 v_x가 0에 가까우면 발산 → 단순화
        v_x_safe = v_x if abs(v_x) > self.MIN_SPEED else (
            self.MIN_SPEED if v_x >= 0 else -self.MIN_SPEED
        )
        
        # 앞바퀴: 조향각 - 진행방향 각도
        alpha_f = delta - math.atan2(v_y + self.L_f * r, v_x_safe)
        # 뒷바퀴: 조향 0 가정
        alpha_r =       - math.atan2(v_y - self.L_r * r, v_x_safe)
        
        # ─── 타이어 힘 ───
        F_yf = self.tire.lateral_force(alpha_f)
        F_yr = self.tire.lateral_force(alpha_r)
        # 단순화: 구동력은 뒷바퀴(rear-wheel drive)로 가정
        F_xf = 0.0
        F_xr = F_drive
        
        # ─── 차체 좌표계 가속도 (Newton-Euler) ───
        a_x = (F_xf * math.cos(delta) - F_yf * math.sin(delta) + F_xr) / self.m + v_y * r
        a_y = (F_xf * math.sin(delta) + F_yf * math.cos(delta) + F_yr) / self.m - v_x * r
        a_r = (self.L_f * (F_xf * math.sin(delta) + F_yf * math.cos(delta))
               - self.L_r * F_yr) / self.I_z
        
        # ─── 월드 좌표계 위치 변화 ───
        dx_dt = v_x * math.cos(psi) - v_y * math.sin(psi)
        dy_dt = v_x * math.sin(psi) + v_y * math.cos(psi)
        dpsi_dt = r
        
        # 미분값을 VehicleState 형태로 (위치/각도는 시간 미분, 속도는 가속도)
        return VehicleState(
            x=dx_dt, y=dy_dt, psi=dpsi_dt,
            v_x=a_x, v_y=a_y, r=a_r
        )
    
    @staticmethod
    def _add_state(s: VehicleState, ds: VehicleState, dt: float) -> VehicleState:
        """RK4 중간 평가용: state + ds·dt"""
        return VehicleState(
            x=s.x + ds.x * dt, y=s.y + ds.y * dt, psi=s.psi + ds.psi * dt,
            v_x=s.v_x + ds.v_x * dt, v_y=s.v_y + ds.v_y * dt, r=s.r + ds.r * dt,
        )
