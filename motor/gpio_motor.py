"""
GPIO Motor HAL (실제 라파)

회로도 기준 실제 모터 제어:
- PCA9685 16채널 PWM 보드 (I2C, 0x40)
  - 채널 0~5: 변형 서보 6개 (바퀴 사이즈, FL/FR/ML/MR/RL/RR)
  - 채널 6~9: 조향 서보 4개 (FL/FR/RL/RR, 가운데 ML/MR은 조향 없음)
- BTS7960 듀얼 DC 모터 드라이버 2개 (RPWM+LPWM+EN 방식)
  - BTS#1 좌 그룹 (3개 모터 병렬): GPIO 18 (RPWM), 12 (LPWM), 23 (EN)
  - BTS#2 우 그룹 (3개 모터 병렬): GPIO 19 (RPWM), 13 (LPWM), 24 (EN)

[속도 변환 흐름]
1. Kinematics → 6개 wheel_velocities [m/s]
2. 좌3 평균, 우3 평균 (회로상 병렬 묶음이라 같은 명령)
3. 정규화 ([-1, 1])
4. 캘리브레이션 변환 → (duty, direction)
5. BTS7960 출력

[조향 변환 흐름]
1. Kinematics → 6개 steer_angles [rad] (가운데 ML/MR은 0 고정)
2. 4개 조향 가능 바퀴 (FL/FR/RL/RR)만 추출
3. 캘리브레이션 변환 → 펄스폭 [us]
4. PCA9685 채널 6~9 출력

[사이즈 변환 흐름]
1. 6개 wheel_sizes [0,1]
2. 캘리브레이션 변환 → 펄스폭 [us]
3. PCA9685 채널 0~5 출력

[안전]
- 라이브러리 import 실패 → SimMotorHAL fallback (factory에서 처리)
- 초기 상태: BTS EN=LOW (모터 비활성), 서보 중립
- emergency_stop: 모든 출력 0 + EN=LOW
"""

import time
import math
from typing import List, Optional
from pathlib import Path

from .hal import MotorHAL, MotorState
from .calibration import MotorCalibration, STEERABLE_WHEELS


# 회로도 기준 핀/채널 매핑 (WIRING.md 기준)
DEFAULT_TRANSFORM_CHANNELS = [0, 1, 2, 3, 4, 5]    # FL, FR, ML, MR, RL, RR
DEFAULT_STEER_CHANNELS = [6, 7, 8, 9]              # FL, FR, RL, RR (가운데 제외)

# BTS#1 (좌측)
DEFAULT_DC_LEFT_RPWM = 18    # 전진
DEFAULT_DC_LEFT_LPWM = 12    # 후진
DEFAULT_DC_LEFT_R_EN = 6
DEFAULT_DC_LEFT_L_EN = 16

# BTS#2 (우측)
DEFAULT_DC_RIGHT_RPWM = 19   # 전진
DEFAULT_DC_RIGHT_LPWM = 13   # 후진
DEFAULT_DC_RIGHT_R_EN = 23
DEFAULT_DC_RIGHT_L_EN = 24

# 바퀴 인덱스
FL, FR, ML, MR, RL, RR = 0, 1, 2, 3, 4, 5


class GpioMotorHAL(MotorHAL):
    """라즈베리파이 실제 모터 제어"""
    
    def __init__(self,
                 calibration: Optional[MotorCalibration] = None,
                 calibration_path: Optional[str] = None,
                 max_velocity: float = 1.0,
                 transform_channels: List[int] = None,
                 steer_channels: List[int] = None,
                 dc_left_rpwm_pin: int = DEFAULT_DC_LEFT_RPWM,
                 dc_left_lpwm_pin: int = DEFAULT_DC_LEFT_LPWM,
                 dc_left_r_en_pin: int = DEFAULT_DC_LEFT_R_EN,
                 dc_left_l_en_pin: int = DEFAULT_DC_LEFT_L_EN,
                 dc_right_rpwm_pin: int = DEFAULT_DC_RIGHT_RPWM,
                 dc_right_lpwm_pin: int = DEFAULT_DC_RIGHT_LPWM,
                 dc_right_r_en_pin: int = DEFAULT_DC_RIGHT_R_EN,
                 dc_right_l_en_pin: int = DEFAULT_DC_RIGHT_L_EN,
                 verbose: bool = True):
        """
        Args:
            calibration: 캘리브레이션 객체. None이면 calibration_path에서 로드.
            calibration_path: 캘리브레이션 yaml 경로. None이면 기본값.
            max_velocity: 최대 선속도 [m/s]. 정규화 기준.
            transform_channels: 변형 서보 채널 (기본 [0~5])
            steer_channels: 조향 서보 채널 4개 (기본 [6, 7, 8, 9])
            dc_left_*: 좌측 BTS7960 핀
            dc_right_*: 우측 BTS7960 핀
            verbose: 로그 출력
        """
        # 캘리브레이션 로드
        if calibration is not None:
            self._cal = calibration
        elif calibration_path is not None:
            self._cal = MotorCalibration.load(calibration_path)
        else:
            self._cal = MotorCalibration.default()
            if verbose:
                print("[GpioMotorHAL] 캘리브레이션 기본값 사용 (튜닝 안 됨)")
        
        self._max_velocity = max_velocity
        self._verbose = verbose
        
        self._transform_channels = transform_channels or DEFAULT_TRANSFORM_CHANNELS
        self._steer_channels = steer_channels or DEFAULT_STEER_CHANNELS
        
        if len(self._steer_channels) != 4:
            raise ValueError(f"조향 채널은 4개 필요 (FL/FR/RL/RR), 받음: {len(self._steer_channels)}")
        
        self._state = MotorState()
        
        # 드라이버 초기화 (지연 import - 라파에서만 동작)
        self._pca = None
        self._dc_left = None
        self._dc_right = None
        self._initialized = False
        
        try:
            from .pca9685_driver import PCA9685Driver
            from .bts7960_driver import BTS7960Driver
            
            # PCA9685 (서보 10개)
            self._pca = PCA9685Driver(frequency_hz=50)
            
            # BTS7960 좌/우 (RPWM+LPWM+R_EN+L_EN 방식, 분기 없음, lgpio chip handle 자동 공유)
            self._dc_left = BTS7960Driver(
                rpwm_pin=dc_left_rpwm_pin,
                lpwm_pin=dc_left_lpwm_pin,
                r_en_pin=dc_left_r_en_pin,
                l_en_pin=dc_left_l_en_pin,
            )
            self._dc_right = BTS7960Driver(
                rpwm_pin=dc_right_rpwm_pin,
                lpwm_pin=dc_right_lpwm_pin,
                r_en_pin=dc_right_r_en_pin,
                l_en_pin=dc_right_l_en_pin,
            )
            
            # 모두 정상 동작 확인
            if (self._pca.is_available and
                self._dc_left.is_available and
                self._dc_right.is_available):
                
                # 초기 상태: 서보 중립
                self._set_all_servos_center()
                
                # BTS 활성화 (EN HIGH) - 시연 중엔 항상 활성, emergency_stop 시만 disable
                self._dc_left.enable()
                self._dc_right.enable()
                
                self._initialized = True
                if verbose:
                    print("[GpioMotorHAL] 초기화 완료")
            else:
                print("[GpioMotorHAL] 일부 드라이버 사용 불가:")
                print(f"  PCA9685: {self._pca.is_available}")
                print(f"  DC Left: {self._dc_left.is_available}")
                print(f"  DC Right: {self._dc_right.is_available}")
                raise RuntimeError("드라이버 초기화 실패")
                
        except Exception as e:
            print(f"[GpioMotorHAL] 초기화 실패: {e}")
            print("[GpioMotorHAL] SimMotorHAL로 fallback 추천")
            raise
    
    def _set_all_servos_center(self) -> None:
        """모든 서보를 중립 위치로"""
        if self._pca is None or not self._pca.is_available:
            return
        # 변형 서보 6개: 사이즈 0.5
        for i, ch in enumerate(self._transform_channels):
            cal = self._find_transform_cal(ch)
            if cal:
                pulse = cal.value_to_pulse(0.0)
                self._pca.set_pulse_us(ch, pulse)
        # 조향 서보 4개: 각도 0
        for i, ch in enumerate(self._steer_channels):
            cal = self._find_steer_cal(ch)
            if cal:
                pulse = cal.value_to_pulse(0.0)
                self._pca.set_pulse_us(ch, pulse)
    
    def _find_transform_cal(self, channel: int):
        for s in self._cal.transform_servos:
            if s.channel == channel:
                return s
        return None
    
    def _find_steer_cal(self, channel: int):
        for s in self._cal.steer_servos:
            if s.channel == channel:
                return s
        return None
    
    # ════════════════════════════════════════════════════════════════
    # MotorHAL 인터페이스 구현
    # ════════════════════════════════════════════════════════════════
    
    def set_wheel_velocities(self, velocities: List[float]) -> None:
        """6개 [m/s] → 좌3/우3 평균 → BTS7960 출력"""
        if len(velocities) != 6:
            raise ValueError(f"6개 속도 필요, 받음: {len(velocities)}")
        
        self._state.wheel_velocities = list(velocities)
        self._state.last_update_time = time.monotonic()
        
        if not self._initialized:
            return
        
        # 좌/우 그룹 평균 (회로상 병렬이라 같은 명령)
        v_left_avg = (velocities[FL] + velocities[ML] + velocities[RL]) / 3
        v_right_avg = (velocities[FR] + velocities[MR] + velocities[RR]) / 3
        
        # 0이 아닌 명령이 들어오면 EN 자동 복구 (emergency_stop 이후 재기동 대비)
        if abs(v_left_avg) > 1e-6 or abs(v_right_avg) > 1e-6:
            self._dc_left.enable()
            self._dc_right.enable()
        
        # 정규화 ([-1, 1])
        v_left_norm = max(-1.0, min(1.0, v_left_avg / self._max_velocity))
        v_right_norm = max(-1.0, min(1.0, v_right_avg / self._max_velocity))
        
        # 캘리브레이션 변환
        duty_l, dir_l = self._cal.dc_left.velocity_to_duty(v_left_norm)
        duty_r, dir_r = self._cal.dc_right.velocity_to_duty(v_right_norm)
        
        # 출력
        self._dc_left.set(duty_l, dir_l)
        self._dc_right.set(duty_r, dir_r)
    
    def set_steer_angles(self, angles: List[float]) -> None:
        """6개 조향각 [rad] → PCA9685 채널 6~9 (조향 가능한 4개만)
        
        Args:
            angles: 6개 바퀴의 조향각. 가운데 ML/MR (index 2, 3)은 무시됨.
        """
        if len(angles) != 6:
            raise ValueError(f"6개 조향각 필요, 받음: {len(angles)}")
        
        self._state.steer_angles = list(angles)
        self._state.last_update_time = time.monotonic()
        
        if not self._initialized:
            return
        
        # 조향 가능한 4개 바퀴만 추출 (FL, FR, RL, RR)
        for ch_idx, wheel_idx in enumerate(STEERABLE_WHEELS):
            ch = self._steer_channels[ch_idx]
            cal = self._find_steer_cal(ch)
            if cal is None:
                continue
            pulse = cal.value_to_pulse(angles[wheel_idx])
            self._pca.set_pulse_us(ch, pulse)
    
    def set_wheel_sizes(self, sizes: List[float]) -> None:
        """6개 사이즈 [0,1] → PCA9685 채널 0~5"""
        if len(sizes) != 6:
            raise ValueError(f"6개 사이즈 필요, 받음: {len(sizes)}")
        
        self._state.wheel_sizes = [max(0.0, min(1.0, s)) for s in sizes]
        self._state.last_update_time = time.monotonic()
        
        if not self._initialized:
            return
        
        for i, size in enumerate(sizes):
            ch = self._transform_channels[i]
            cal = self._find_transform_cal(ch)
            if cal is None:
                continue
            pulse = cal.value_to_pulse(size)
            self._pca.set_pulse_us(ch, pulse)
    
    def emergency_stop(self) -> None:
        """모든 모터 즉시 정지 + EN LOW (드라이버 비활성)."""
        self._state.wheel_velocities = [0.0] * 6
        if self._initialized:
            try:
                # PWM 먼저 0, 그 다음 EN LOW
                self._dc_left.stop()
                self._dc_right.stop()
                self._dc_left.disable()
                self._dc_right.disable()
            except Exception as e:
                print(f"[GpioMotorHAL] emergency_stop 에러: {e}")
        if self._verbose:
            print("[GpioMotorHAL] EMERGENCY STOP")
    
    def resume(self) -> None:
        """emergency_stop 후 다시 활성화"""
        if self._initialized:
            try:
                self._dc_left.enable()
                self._dc_right.enable()
                if self._verbose:
                    print("[GpioMotorHAL] resumed")
            except Exception as e:
                print(f"[GpioMotorHAL] resume 에러: {e}")
    
    def shutdown(self) -> None:
        """리소스 정리. 모든 PWM 끔."""
        if self._verbose:
            print("[GpioMotorHAL] 종료 중...")
        
        try:
            # DC 정지 + 비활성화 + 핀 free
            if self._dc_left is not None:
                self._dc_left.shutdown()
            if self._dc_right is not None:
                self._dc_right.shutdown()
            
            # PCA9685 끔 (서보 힘 빠짐)
            if self._pca is not None:
                self._pca.disable_all()
                self._pca.shutdown()
        except Exception as e:
            print(f"[GpioMotorHAL] 종료 중 에러: {e}")
        finally:
            self._initialized = False
    
    def get_state(self) -> MotorState:
        return self._state
