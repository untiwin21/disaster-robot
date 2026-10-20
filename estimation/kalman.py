"""
Kalman Filter

선형 가우시안 시스템에서 최적 상태 추정기. 1960년 Rudolf Kalman이 발표.
아폴로 11호 달 착륙 항법, GPS, INS, 거의 모든 자율시스템에서 사용.

[직관]
"예측"과 "측정"을 가중평균으로 합친다.

    예측: 모델로 다음 상태 추정 (불확실성 ↑)
    측정: 센서로 직접 관측 (노이즈 있음)
    
    → 둘을 조합. 신뢰도 높은 쪽에 가중치 더.
    "최소 분산 추정" 이라는 의미에서 최적.

[수학적 표현]
시스템 모델 (선형):
    x_{k+1} = F·x_k + B·u_k + w_k        ← 상태 전이 (process noise w)
    z_k     = H·x_k + v_k                 ← 측정 (measurement noise v)

w ~ N(0, Q): process noise covariance
v ~ N(0, R): measurement noise covariance

[알고리즘 (2단계 반복)]

1) 예측 (Predict):
    x̂_{k|k-1} = F·x̂_{k-1|k-1} + B·u_k       ← 상태 예측
    P_{k|k-1} = F·P_{k-1|k-1}·Fᵀ + Q          ← 공분산 예측

2) 갱신 (Update):
    y_k = z_k - H·x̂_{k|k-1}                   ← innovation (측정 - 예측)
    S_k = H·P_{k|k-1}·Hᵀ + R                  ← innovation covariance
    K_k = P_{k|k-1}·Hᵀ·S_k⁻¹                  ← Kalman gain
    x̂_{k|k} = x̂_{k|k-1} + K_k·y_k             ← 상태 갱신
    P_{k|k} = (I - K_k·H)·P_{k|k-1}           ← 공분산 갱신

[Kalman gain의 의미]
- 측정 신뢰도 높음 (R 작음): K 큼 → 측정 쪽으로 끌림
- 측정 신뢰도 낮음 (R 큼): K 작음 → 예측 유지
- 자동으로 최적 조합 찾음

[너 프로젝트 적용]
- Process: 바퀴 명령으로 다음 위치 예측 (Odometry 기반)
- Measurement: 만약 IMU/GPS/비전 있으면 측정값 fusion
- 현재 우린 센서가 초음파만 있어서, 사실 KF가 큰 의미 없음.
  학습용으로만 구현. 실전에선 IMU 추가하면 진가 발휘.
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional


class KalmanFilter:
    """
    Linear Kalman Filter.
    
    상태 차원, 측정 차원 자유롭게 설정 가능.
    """
    
    def __init__(self,
                 F: np.ndarray, H: np.ndarray,
                 Q: np.ndarray, R: np.ndarray,
                 B: Optional[np.ndarray] = None,
                 x0: Optional[np.ndarray] = None,
                 P0: Optional[np.ndarray] = None):
        """
        Args:
            F: (n, n) 상태 전이 행렬
            H: (m, n) 관측 행렬
            Q: (n, n) process noise 공분산
            R: (m, m) measurement noise 공분산
            B: (n, p) 제어 입력 행렬 (없으면 입력 무시)
            x0: (n,) 초기 상태. 기본 0.
            P0: (n, n) 초기 공분산. 기본 단위행렬.
        """
        self.F = F
        self.H = H
        self.Q = Q
        self.R = R
        self.B = B
        
        n = F.shape[0]
        self.n = n
        self.x = x0 if x0 is not None else np.zeros(n)
        self.P = P0 if P0 is not None else np.eye(n)
    
    def predict(self, u: Optional[np.ndarray] = None) -> np.ndarray:
        """
        예측 단계.
        
        Args:
            u: 제어 입력 (B가 정의된 경우)
        Returns:
            예측된 상태
        """
        # x̂ = F·x + B·u
        self.x = self.F @ self.x
        if self.B is not None and u is not None:
            self.x = self.x + self.B @ u
        
        # P = F·P·Fᵀ + Q
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x.copy()
    
    def update(self, z: np.ndarray) -> np.ndarray:
        """
        갱신 단계.
        
        Args:
            z: 측정값 (m,)
        Returns:
            갱신된 상태
        """
        # Innovation
        y = z - self.H @ self.x
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        # 상태/공분산 갱신
        self.x = self.x + K @ y
        I = np.eye(self.n)
        self.P = (I - K @ self.H) @ self.P
        
        return self.x.copy()
    
    def step(self, z: np.ndarray, u: Optional[np.ndarray] = None) -> np.ndarray:
        """예측 + 갱신 한 번에"""
        self.predict(u)
        return self.update(z)
