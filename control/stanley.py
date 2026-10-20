"""
Stanley Path Tracker

스탠포드 대학교가 2005년 DARPA Grand Challenge에서 우승한
무인차 'Stanley'에 사용된 path tracking 알고리즘.

[Pure Pursuit과 차이]
- Pure Pursuit: 미래의 한 점을 추격. 코너에서 컷 인 (안쪽으로 들어감).
- Stanley: 차량의 앞축이 가장 가까운 경로점에 닿도록 보정. 더 정확.

Stanley는 두 가지 오차를 동시에 보상:
1. Heading error (ψ_e): 차량 방향과 경로 방향의 각도 차
2. Cross-track error (e): 차량 앞축과 경로 사이의 횡방향 거리

[수식]
    δ = ψ_e + arctan(k · e / v)
    
    ψ_e: heading error (차체 yaw - 경로 접선 yaw)
    e:   cross-track error (음수: 경로 우측에 있음)
    v:   현재 속도
    k:   gain (튜닝 파라미터, 보통 0.5~5)

[직관]
- ψ_e만 있으면: 경로 방향으로 정렬 (heading 보정)
- e만 있으면: 경로 쪽으로 다가감 (위치 보정)
- v로 나누는 이유: 빠르면 작게 보정 (안정성), 느리면 크게 (정확성)

[저속 처리]
v=0 근처에선 1/v 발산. softening 사용:
    δ_e = arctan(k · e / (v_softening + v))

[참조점]
Pure Pursuit: 차량 위치(보통 뒷축) 기준
Stanley:      차량 앞축 기준 ← 이게 차이의 핵심
"""

import math
from dataclasses import dataclass
from typing import List, Tuple, Optional

from shared.config import RobotConfig


@dataclass
class StanleyResult:
    steer: float                # [rad]
    heading_error: float        # [rad] ψ_e
    cross_track_error: float    # [m] e (부호 있음)
    target_index: int           # 가장 가까운 경로점 인덱스
    front_axle: Tuple[float, float]  # 앞축 위치 (월드)


class StanleyController:
    """
    Stanley path tracker.
    
    경로의 yaw(접선 방향)도 필요하므로 경로 형식이 다름:
    path: [(x, y, yaw), ...]
    
    yaw가 없으면 인접 점 차이로 자동 계산하는 헬퍼도 제공.
    """
    
    def __init__(self, robot: RobotConfig,
                 k: float = 1.5,
                 softening_speed: float = 0.3,
                 max_steer: Optional[float] = None):
        """
        Args:
            k: cross-track gain
            softening_speed: 1/v 발산 방지용
            max_steer: 조향각 제한. None이면 robot.steering.max_angle 사용.
        """
        self.k = k
        self.softening = softening_speed
        self.L = robot.wheelbase
        self.L_f = self.L / 2  # 앞축은 차체 중심에서 +L/2
        self.max_steer = max_steer if max_steer is not None else robot.steering.max_angle
    
    @staticmethod
    def compute_path_yaws(path_xy: List[Tuple[float, float]]) -> List[float]:
        """경로점들의 인접 차이로 yaw 계산"""
        yaws = []
        for i in range(len(path_xy)):
            if i == 0:
                dx = path_xy[1][0] - path_xy[0][0]
                dy = path_xy[1][1] - path_xy[0][1]
            elif i == len(path_xy) - 1:
                dx = path_xy[-1][0] - path_xy[-2][0]
                dy = path_xy[-1][1] - path_xy[-2][1]
            else:
                # 중간점은 양쪽 평균 (smoother)
                dx = path_xy[i+1][0] - path_xy[i-1][0]
                dy = path_xy[i+1][1] - path_xy[i-1][1]
            yaws.append(math.atan2(dy, dx))
        return yaws
    
    def compute(self,
                path: List[Tuple[float, float, float]],   # (x, y, yaw)
                pose: Tuple[float, float, float],         # 차량 (x, y, psi)
                velocity: float) -> Optional[StanleyResult]:
        if not path:
            return None
        
        car_x, car_y, psi = pose
        
        # ─── 앞축 위치 ───
        # 차체 중심이 (car_x, car_y), 앞축은 +x 방향으로 L_f
        front_x = car_x + self.L_f * math.cos(psi)
        front_y = car_y + self.L_f * math.sin(psi)
        
        # ─── 가장 가까운 경로점 찾기 ───
        # 단순 무차별 검색. 경로 길면 KD-tree나 단조 검색 권장.
        min_dist_sq = float('inf')
        min_idx = 0
        for i, (px, py, _) in enumerate(path):
            d2 = (front_x - px)**2 + (front_y - py)**2
            if d2 < min_dist_sq:
                min_dist_sq = d2
                min_idx = i
        
        target_x, target_y, target_yaw = path[min_idx]
        
        # ─── Cross-track error (부호 있음) ───
        # 경로 접선 방향이 target_yaw, 그 법선 방향이 (-sin, cos)
        # 차량이 경로의 어느 쪽에 있는지: 법선 방향과 (front - target)의 내적
        dx = front_x - target_x
        dy = front_y - target_y
        # 법선 단위벡터 (좌측 양수)
        nx = -math.sin(target_yaw)
        ny =  math.cos(target_yaw)
        cross_track = dx * nx + dy * ny  # 좌측 양수
        
        # ─── Heading error ───
        heading_error = self._normalize_angle(target_yaw - psi)
        
        # ─── Stanley 공식 ───
        # δ = ψ_e + atan(k·e / (softening + v))
        # 단, 부호 주의: cross_track > 0 (좌측에 있음)이면 우측으로 꺾어야 함
        # → atan(-k·e / ...) 가 일반적 부호 컨벤션
        # 실제로 경로마다 다르므로 시각화로 검증하는 게 가장 확실함
        v_safe = max(self.softening, abs(velocity))
        cte_term = math.atan2(-self.k * cross_track, v_safe)
        steer = heading_error + cte_term
        
        # ─── Saturation ───
        steer = max(-self.max_steer, min(self.max_steer, steer))
        
        return StanleyResult(
            steer=steer,
            heading_error=heading_error,
            cross_track_error=cross_track,
            target_index=min_idx,
            front_axle=(front_x, front_y),
        )
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """각도를 [-π, π]로 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
