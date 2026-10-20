"""
Estimation Manager

엔코더 + 오도메트리 + 광학흐름 + EKF 통합 관리.

[데이터 흐름]
    EncoderReader (좌/우)
        ↓ v_left, v_right
    DifferentialOdometry      ← 기존 dead-reckoning (참고용)
        ↓ pose_odom
    DifferentialRobotEKF      ← 5-state fusion
        ↑ v_optical
    OpticalFlowEstimator (영상 송신 스레드에서 별도 호출)

[책임 분리]
- EncoderReader: GPIO 카운팅 (callback)
- DifferentialOdometry: 누적 위치 (dead-reckoning, drift 있음)
- OpticalFlow: 카메라 기반 속도 (별도 호출, 비동기 fusion)
- EKF: 위 세 측정을 sensor fusion

[비동기 fusion 패턴]
EKF는 호출 순서 무관하게 동작:
    1. step(dt) → predict + encoder update
    2. (다른 스레드) on_optical_flow(v) → update_optical_flow
공분산 P가 시간 진행에 맞춰 진행되므로 측정값 늦게 와도 적절히 가중평균
"""

import time
import math
from dataclasses import dataclass
from typing import Optional

from .odometry import DifferentialOdometry, OdometryState, WheelEncoderData
from .ekf import DifferentialRobotEKF


@dataclass
class EstimationState:
    """추정 결과 종합."""
    # Odometry (dead-reckoning, 비교용)
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_psi: float = 0.0
    
    # EKF (fusion 결과)
    ekf_x: float = 0.0
    ekf_y: float = 0.0
    ekf_psi: float = 0.0
    ekf_v: float = 0.0
    ekf_omega: float = 0.0
    
    # 측정값 (디버깅)
    v_left: float = 0.0       # 엔코더 좌측 속도 [m/s]
    v_right: float = 0.0      # 엔코더 우측 속도 [m/s]
    v_encoder: float = 0.0    # 엔코더 평균 (선속도)
    omega_encoder: float = 0.0
    v_optical: float = 0.0    # 광학흐름 속도 [m/s]
    optical_valid: bool = False
    
    timestamp: float = 0.0


class EstimationManager:
    """엔코더/오도메트리/EKF 묶음.
    
    EncoderReader 인스턴스를 외부에서 주입 (HAL에서 생성된 거 그대로 사용).
    """
    
    def __init__(self,
                 encoder_left,
                 encoder_right,
                 track_width: float,
                 dt: float = 0.02,
                 ekf_process_noise=(0.01, 0.01, 0.01, 0.1, 0.1),
                 ekf_encoder_noise=(0.05, 0.05),
                 ekf_optical_noise=0.5):
        """
        Args:
            encoder_left: EncoderReader 인스턴스 (좌측, 없으면 None 가능 — sim 모드)
            encoder_right: EncoderReader 인스턴스 (우측)
            track_width: 좌우 휠 거리 [m]
            dt: 기본 추정 주기 [s]
            ekf_*: EKF 노이즈 파라미터
        """
        self._enc_left = encoder_left
        self._enc_right = encoder_right
        self._track = track_width
        self._dt_default = dt
        
        self._odometry = DifferentialOdometry(track_width=track_width)
        self._ekf = DifferentialRobotEKF(
            dt=dt,
            process_noise_std=ekf_process_noise,
            encoder_noise_std=ekf_encoder_noise,
            optical_flow_noise_std=ekf_optical_noise,
        )
        
        self._last_step_time: Optional[float] = None
        self._state = EstimationState()
    
    @property
    def has_real_encoders(self) -> bool:
        return (self._enc_left is not None and self._enc_left.is_available and
                self._enc_right is not None and self._enc_right.is_available)
    
    def step(self,
             dt: Optional[float] = None,
             v_left_sim: Optional[float] = None,
             v_right_sim: Optional[float] = None) -> EstimationState:
        """한 스텝 추정 (예측 + 엔코더 측정 갱신).
        
        실기 모드: 엔코더에서 자동으로 v_left/v_right 읽음
        시뮬 모드: v_left_sim, v_right_sim 명시적 전달
        
        Args:
            dt: 이번 스텝 timestep. None이면 자동 측정
            v_left_sim, v_right_sim: 시뮬용 명령 속도 [m/s]
        """
        now = time.monotonic()
        if dt is None:
            if self._last_step_time is None:
                dt = self._dt_default
            else:
                dt = now - self._last_step_time
        self._last_step_time = now
        
        # 1. 엔코더 측정
        if self.has_real_encoders:
            v_l, dist_l = self._enc_left.compute_velocity(dt)
            v_r, dist_r = self._enc_right.compute_velocity(dt)
        else:
            v_l = v_left_sim if v_left_sim is not None else 0.0
            v_r = v_right_sim if v_right_sim is not None else 0.0
            dist_l = v_l * dt
            dist_r = v_r * dt
        
        # 2. Odometry 갱신 (dead-reckoning, 참고용)
        encoder_data = WheelEncoderData(d_left=dist_l, d_right=dist_r)
        odom_state = self._odometry.update(encoder_data)
        
        # 3. EKF predict
        self._ekf.predict(dt)
        
        # 4. EKF encoder update
        v_avg = (v_l + v_r) / 2
        omega_meas = (v_r - v_l) / self._track if self._track > 0 else 0.0
        self._ekf.update_encoder(v_avg, omega_meas)
        
        # 상태 종합
        ekf_s = self._ekf.state_dict()
        self._state = EstimationState(
            odom_x=odom_state.x,
            odom_y=odom_state.y,
            odom_psi=odom_state.psi,
            ekf_x=ekf_s['x'],
            ekf_y=ekf_s['y'],
            ekf_psi=ekf_s['psi'],
            ekf_v=ekf_s['v'],
            ekf_omega=ekf_s['omega'],
            v_left=v_l,
            v_right=v_r,
            v_encoder=v_avg,
            omega_encoder=omega_meas,
            v_optical=self._state.v_optical,         # 이전 값 유지
            optical_valid=self._state.optical_valid,
            timestamp=now,
        )
        return self._state
    
    def on_optical_flow(self, v_flow: float, valid: bool = True) -> None:
        """비동기 광학흐름 측정 입력 (영상 송신 스레드에서 호출).
        
        Args:
            v_flow: 광학흐름 속도 [m/s]
            valid: 측정 유효성. False면 EKF 갱신 안 함
        """
        self._state.v_optical = v_flow
        self._state.optical_valid = valid
        if valid:
            self._ekf.update_optical_flow(v_flow)
    
    @property
    def state(self) -> EstimationState:
        return self._state
    
    def reset(self) -> None:
        self._odometry = DifferentialOdometry(track_width=self._track)
        self._ekf.reset()
        self._last_step_time = None
        if self._enc_left:
            self._enc_left.reset()
        if self._enc_right:
            self._enc_right.reset()
        self._state = EstimationState()
