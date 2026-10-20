"""
Pacejka Magic Formula Tire Model

타이어가 발생시키는 마찰력을 모델링하는 산업 표준 공식.
F1, 자동차 시뮬레이터, 차량 제어 연구에서 모두 사용.

[배경]
타이어는 노면과 미끄러짐(slip)이 있을 때 마찰력을 발생시킴.
- slip 0     → 마찰력 0     (그립 안 함)
- slip 작음   → 마찰력 선형 증가 (선형 영역)
- slip 적정  → 마찰력 최대  (peak grip)
- slip 큼    → 마찰력 감소 (saturation, 미끄러짐)

이 비선형 곡선을 표현하는 게 Pacejka 공식.

[Magic Formula]
    F(α) = D · sin(C · arctan(B·α - E·(B·α - arctan(B·α))))

    α: slip angle [rad] 또는 slip ratio [-]
    B: 강성 계수 (stiffness factor) - 곡선 시작 기울기
    C: 형상 계수 (shape factor)     - 곡선 모양
    D: 피크 계수 (peak value)       - 최대 마찰력 (μ · 수직하중)
    E: 곡률 계수 (curvature factor) - peak 이후 감쇠

[Slip Angle 정의]
바퀴가 향한 방향과 실제 진행 방향 사이의 각도.
    α = arctan(v_y / v_x)
    v_x: 바퀴 진행 방향 속도 (구르는 방향)
    v_y: 바퀴 측면 방향 속도 (옆으로 미끄러짐)

[횡력(lateral force)과 종력(longitudinal force)]
- 횡력 F_y: slip angle α에 의해 발생 (커브 돌 때 원심력 버팀)
- 종력 F_x: slip ratio κ에 의해 발생 (가속/제동)

이 모듈에선 일단 lateral force만 다루고, longitudinal은 단순화.
실제 결합 시엔 friction circle 제약 적용 (F_x² + F_y² ≤ (μN)²).

[참고 - 시각적 직관]
slip이 작으면 타이어가 잘 잡지만, 너무 크게 미끄러지면 오히려 그립 잃음.
사람이 빙판에서 너무 세게 밀면 미끄러지는 거랑 같은 원리.
"""

import math
from dataclasses import dataclass

from shared.config import TireConfig


@dataclass
class TireForces:
    """타이어가 발생시키는 힘"""
    F_lateral: float      # [N] 횡력 (옆방향)
    F_longitudinal: float # [N] 종력 (구르는 방향)
    
    def magnitude(self) -> float:
        return math.hypot(self.F_lateral, self.F_longitudinal)


class PacejkaTireModel:
    """
    Pacejka Magic Formula 기반 타이어 모델.
    
    단순화 가정:
    - lateral과 longitudinal을 분리해서 계산 (실제로 결합 효과 있음)
    - 캠버, 노면 변화 등은 무시
    - 수직하중은 일정 가정 (실제론 가속/제동 시 변함)
    """
    
    def __init__(self, tire: TireConfig, normal_load: float = 50.0):
        """
        Args:
            tire: Pacejka 계수 (B, C, D, E)
            normal_load: [N] 바퀴당 수직하중 (로봇 무게 / 바퀴 수 × g)
                         예: 5kg 6륜이면 약 5×9.81/6 ≈ 8N. 시뮬용 임시값.
        """
        self.B = tire.B
        self.C = tire.C
        self.D = tire.D * normal_load   # peak force = μ · N
        self.E = tire.E
        self.normal_load = normal_load
    
    def lateral_force(self, slip_angle: float) -> float:
        """
        Slip angle [rad] → 횡력 [N]
        
        F_y = D · sin(C · arctan(B·α - E·(B·α - arctan(B·α))))
        """
        Ba = self.B * slip_angle
        inner = Ba - self.E * (Ba - math.atan(Ba))
        return self.D * math.sin(self.C * math.atan(inner))
    
    def longitudinal_force(self, slip_ratio: float) -> float:
        """
        Slip ratio [-] → 종력 [N]
        
        slip_ratio = (ω·r - v) / max(|ω·r|, |v|)
            ω: 바퀴 각속도, r: 바퀴 반경, v: 차체 속도
            +값: 가속 (구동), -값: 제동
        """
        # 동일한 Magic Formula, 입력만 다름
        Bk = self.B * slip_ratio
        inner = Bk - self.E * (Bk - math.atan(Bk))
        return self.D * math.sin(self.C * math.atan(inner))
    
    def combined_force(self, slip_angle: float, slip_ratio: float) -> TireForces:
        """
        Lateral + Longitudinal을 friction circle 제약으로 결합.
        
        F_x² + F_y² ≤ μ_max²
        타이어 그립의 한계가 원형 → 한쪽이 크면 다른 쪽이 줄어듬.
        가속하면서 동시에 코너링하면 둘 다 감소하는 현상.
        """
        F_y_uncoupled = self.lateral_force(slip_angle)
        F_x_uncoupled = self.longitudinal_force(slip_ratio)
        
        # Friction circle 제약
        magnitude = math.hypot(F_x_uncoupled, F_y_uncoupled)
        max_force = self.D
        
        if magnitude > max_force and magnitude > 0:
            # 비율 유지하며 스케일 다운
            scale = max_force / magnitude
            return TireForces(
                F_lateral=F_y_uncoupled * scale,
                F_longitudinal=F_x_uncoupled * scale,
            )
        return TireForces(F_lateral=F_y_uncoupled, F_longitudinal=F_x_uncoupled)
