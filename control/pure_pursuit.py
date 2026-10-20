"""
Pure Pursuit Path Tracker

가장 직관적이고 견고한 path tracking 알고리즘. 1990년대 카네기멜런에서
시작, 지금도 자율주행 차량에서 널리 쓰임.

[직관]
운전할 때 사람은 멀리 있는 한 점을 보고 거기로 향한다.
Pure Pursuit도 똑같음:
1. 경로상에서 차량 앞쪽 일정 거리(lookahead)에 있는 점을 찾는다
2. 그 점으로 향하는 원호를 그리는 조향각을 계산한다
3. 차가 진행하면서 lookahead 점도 같이 움직인다 (말 그대로 "추격")

[수식 유도]
차량 위치를 원점, 진행방향을 x축으로 한 좌표계에서
목표점 (x_t, y_t)에 도달하려면, 원호의 반지름은:

    L_d² = x_t² + y_t²        ← lookahead distance
    R = L_d² / (2·y_t)         ← 원의 반지름 (sin θ = y_t / L_d 이용)

조향각 (Ackermann bicycle 가정):
    δ = arctan(L · 2·y_t / L_d²)
    
    L: 휠베이스, y_t: 목표점의 횡방향 거리

[Lookahead distance 선택]
- 짧으면: 정확하지만 진동(oscillation), 코너에서 컷 인
- 길면: 부드럽지만 추종 정확도 떨어짐, 코너에서 큰 원호

흔한 휴리스틱: L_d = k·v + L_d_min   (속도 비례)

[좌표계]
이 모듈은 path를 월드 좌표계에서 받는다.
차량 상태(x, y, ψ)를 사용해 차량 좌표계로 변환 후 계산.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple, Optional

from shared.config import PurePursuitConfig


@dataclass
class PurePursuitResult:
    """디버깅/시각화용 출력"""
    steer: float                    # [rad] 계산된 조향각
    lookahead_distance: float       # 사용된 L_d
    target_index: int               # 경로에서 선택된 인덱스
    target_world: Tuple[float, float]  # 월드 좌표계 목표점
    target_body: Tuple[float, float]   # 차체 좌표계 목표점


class PurePursuitController:
    """
    경로 추종 제어기.
    
    경로(path): [(x, y), ...] 점들의 시퀀스.
    차량 상태: (x, y, psi) 월드 좌표.
    """
    
    def __init__(self, config: PurePursuitConfig, wheelbase: float):
        self.L_d_base = config.lookahead_distance
        self.L_d_min = config.min_lookahead
        self.L_d_max = config.max_lookahead
        self.L = wheelbase
        
        # 속도-적응 게인. 속도가 0이면 L_d_base 사용.
        # L_d = k_v · v + L_d_base 로 만드려면 k_v 정해야 하나
        # 단순화: max_velocity일 때 L_d_max에 도달하는 비율
        # → 외부에서 max_v 받으면 k_v = (L_d_max - L_d_base) / max_v
        # 여기선 일단 L_d_base 고정으로 두고, compute_lookahead에서 v 받으면 조절
    
    def compute_lookahead(self, velocity: float, max_velocity: float = 1.0) -> float:
        """속도 적응형 lookahead. 빠를수록 멀리 본다."""
        # 0 ~ max_velocity → L_d_base ~ L_d_max
        ratio = max(0.0, min(1.0, abs(velocity) / max_velocity))
        L_d = self.L_d_base + ratio * (self.L_d_max - self.L_d_base)
        return max(self.L_d_min, min(self.L_d_max, L_d))
    
    def find_target(self,
                    path: List[Tuple[float, float]],
                    pose: Tuple[float, float, float],
                    L_d: float,
                    last_target_idx: int = 0) -> Optional[int]:
        """
        경로상에서 차량으로부터 L_d 떨어진 점 찾기.
        
        효율을 위해 last_target_idx부터 검색 (단조 증가 가정).
        
        Returns:
            찾은 점의 인덱스. 끝에 도달했으면 마지막 인덱스.
            경로 자체가 비어있으면 None.
        """
        if not path:
            return None
        
        car_x, car_y, _ = pose
        
        # last_target_idx부터 진행하며 처음으로 L_d 이상 멀어진 점 찾기
        for i in range(last_target_idx, len(path)):
            dx = path[i][0] - car_x
            dy = path[i][1] - car_y
            dist = math.hypot(dx, dy)
            if dist >= L_d:
                return i
        
        # 경로 끝까지 갔는데 L_d 이상 떨어진 점 없음 → 마지막 점 사용
        return len(path) - 1
    
    def compute(self,
                path: List[Tuple[float, float]],
                pose: Tuple[float, float, float],
                velocity: float,
                max_velocity: float = 1.0,
                last_target_idx: int = 0) -> Optional[PurePursuitResult]:
        """
        경로 추종 조향각 계산.
        
        Args:
            path: 월드 좌표계 경로 [(x, y), ...]
            pose: 차량 상태 (x, y, psi)
            velocity: 현재 속도 [m/s] (lookahead 계산용)
            max_velocity: 정규화용
            last_target_idx: 이전 목표 인덱스 (탐색 시작점)
        
        Returns:
            PurePursuitResult, 또는 path가 비어있으면 None
        """
        if not path:
            return None
        
        L_d = self.compute_lookahead(velocity, max_velocity)
        target_idx = self.find_target(path, pose, L_d, last_target_idx)
        if target_idx is None:
            return None
        
        target_world = path[target_idx]
        car_x, car_y, psi = pose
        
        # ─── 월드 → 차체 좌표계 변환 ───
        # 차체 x축이 월드 (cos ψ, sin ψ) 방향.
        # 차체 좌표계에서 목표점은:
        #   x_body =  (target_x - car_x)·cos(ψ) + (target_y - car_y)·sin(ψ)
        #   y_body = -(target_x - car_x)·sin(ψ) + (target_y - car_y)·cos(ψ)
        dx = target_world[0] - car_x
        dy = target_world[1] - car_y
        x_body =  dx * math.cos(psi) + dy * math.sin(psi)
        y_body = -dx * math.sin(psi) + dy * math.cos(psi)
        
        # ─── Pure Pursuit 수식 ───
        # δ = arctan(2·L·y_body / L_d²)
        # L_d는 실제 차량~목표점 거리로 다시 계산 (정확도)
        L_d_actual = math.hypot(x_body, y_body)
        if L_d_actual < 1e-6:
            return PurePursuitResult(
                steer=0.0, lookahead_distance=L_d, target_index=target_idx,
                target_world=target_world, target_body=(x_body, y_body),
            )
        
        # 목표점이 차량 뒤쪽이면 (x_body < 0) → 회전 방향 모호
        # 단순화: 일단 그래도 공식 적용. 실전에선 후진 모드 등 별도 처리.
        steer = math.atan2(2 * self.L * y_body, L_d_actual ** 2)
        
        return PurePursuitResult(
            steer=steer,
            lookahead_distance=L_d_actual,
            target_index=target_idx,
            target_world=target_world,
            target_body=(x_body, y_body),
        )
