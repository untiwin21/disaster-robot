"""
Odometry (오도메트리)

바퀴 회전수만으로 로봇의 위치/방향 추정.
"Dead reckoning" 이라고도 부름 - 외부 참조 없이 누적만으로 위치 추적.

[원리]
바퀴가 굴러간 거리 = 바퀴 회전각 × 바퀴 반지름
좌우 바퀴의 이동 거리 차이로 회전각도 계산.

차량 중심 이동:
    d = (d_left + d_right) / 2          ← 평균 거리 (선형 이동)
    Δψ = (d_right - d_left) / W          ← yaw 변화 (좌우 차이)

월드 좌표 갱신 (Runge 적분):
    x  ← x + d · cos(ψ + Δψ/2)         ← 중간 yaw 사용 (정확도)
    y  ← y + d · sin(ψ + Δψ/2)
    ψ  ← ψ + Δψ

[Differential drive odometry]
좌우 두 바퀴(또는 그룹)만 알아도 되는 가장 단순한 형태.
6륜 로봇에서는 좌측 3개, 우측 3개를 각각 평균해서 사용.

[한계]
- 누적 오차: 작은 오차도 시간 지나면 큼 ("drift")
- 미끄러짐 못 잡음 (skid steer에선 특히 심각)
- 평지 가정 (험지에선 부정확)
- → IMU, GPS, 비전 등과 fusion 필요 (그래서 Kalman 필터)

[입력 데이터]
실제 로봇: 엔코더 카운트 → 회전각 → 거리
시뮬: 모터에 보낸 명령으로부터 추정 (아니면 실제 차량 상태에서 노이즈 추가)
"""

import math
from dataclasses import dataclass


@dataclass
class OdometryState:
    """추정된 차량 상태"""
    x: float = 0.0        # 월드 [m]
    y: float = 0.0        # 월드 [m]
    psi: float = 0.0      # yaw [rad]
    
    # 디버깅용 누적 거리
    distance_traveled: float = 0.0


@dataclass
class WheelEncoderData:
    """바퀴 엔코더 한 번 읽음"""
    # 좌우 그룹 평균 거리 (이전 읽음 이후 이동한 거리)
    d_left: float    # [m]
    d_right: float   # [m]


class DifferentialOdometry:
    """
    Differential drive 오도메트리.
    
    매 스텝마다 좌우 그룹 거리 변화를 받아 차량 위치 갱신.
    """
    
    def __init__(self, track_width: float, initial_state: OdometryState = None):
        """
        Args:
            track_width: 좌우 바퀴 사이 거리 [m]
            initial_state: 시작 위치
        """
        self.W = track_width
        self.state = initial_state if initial_state else OdometryState()
    
    def update(self, encoder: WheelEncoderData) -> OdometryState:
        """
        한 스텝 업데이트.
        
        Args:
            encoder: 이 스텝 동안 좌우 그룹이 이동한 거리
        
        Returns:
            갱신된 위치/자세
        """
        d_left = encoder.d_left
        d_right = encoder.d_right
        
        # 차량 중심 이동
        d = (d_left + d_right) / 2.0
        d_psi = (d_right - d_left) / self.W
        
        # 중간 yaw로 적분 (Runge integration, 일반 Euler보다 정확)
        psi_mid = self.state.psi + d_psi / 2.0
        self.state.x += d * math.cos(psi_mid)
        self.state.y += d * math.sin(psi_mid)
        self.state.psi += d_psi
        
        # yaw [-π, π] 정규화
        self.state.psi = self._normalize_angle(self.state.psi)
        
        self.state.distance_traveled += abs(d)
        return self.state
    
    def reset(self, state: OdometryState = None) -> None:
        self.state = state if state else OdometryState()
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def compute_wheel_distances_from_velocities(v_left: float, v_right: float,
                                            dt: float) -> WheelEncoderData:
    """
    속도 명령에서 거리 변환 헬퍼.
    
    실제 엔코더 대신 시뮬에서 명령 속도를 이상적으로 적분할 때 사용.
    """
    return WheelEncoderData(
        d_left=v_left * dt,
        d_right=v_right * dt,
    )
