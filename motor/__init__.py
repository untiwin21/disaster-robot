"""Motor package - HAL pattern으로 시뮬/실제 추상화"""

from .hal import MotorHAL, MotorState
from .sim_motor import SimMotorHAL
from .calibration import (
    MotorCalibration, ServoCalibration, DcMotorCalibration,
    WHEEL_NAMES, STEERABLE_WHEELS, STEERABLE_NAMES,
    FL, FR, ML, MR, RL, RR,
)


def create_motor_hal(config) -> MotorHAL:
    """
    Config에 따라 적절한 모터 HAL 생성.
    
    - simulation 모드: SimMotorHAL
    - real 모드: GpioMotorHAL 시도 → 실패 시 SimMotorHAL fallback
    """
    if config.is_simulation:
        verbose = False
        return SimMotorHAL(
            response_delay=config.simulation.motor_response_delay if config.simulation else 0.02,
            verbose=verbose,
        )
    
    # real 모드
    try:
        from .gpio_motor import GpioMotorHAL
        
        # config에서 캘리브레이션 경로 + 핀 매핑 가져오기
        cal_path = getattr(config, "motor_calibration_path", None) or "config/calibration.yaml"
        max_v = config.robot.motion.max_velocity
        
        # config.motor_pins 같은 게 있으면 활용 (없으면 기본값)
        pins = getattr(config, "motor_pins", None)
        kwargs = {}
        if pins is not None:
            kwargs.update({
                "transform_channels": pins.get("transform_channels"),
                "steer_channels": pins.get("steer_channels"),
                "dc_left_rpwm_pin": pins.get("dc_left_rpwm_pin", 18),
                "dc_left_lpwm_pin": pins.get("dc_left_lpwm_pin", 12),
                "dc_left_r_en_pin": pins.get("dc_left_r_en_pin", 6),
                "dc_left_l_en_pin": pins.get("dc_left_l_en_pin", 16),
                "dc_right_rpwm_pin": pins.get("dc_right_rpwm_pin", 19),
                "dc_right_lpwm_pin": pins.get("dc_right_lpwm_pin", 13),
                "dc_right_r_en_pin": pins.get("dc_right_r_en_pin", 23),
                "dc_right_l_en_pin": pins.get("dc_right_l_en_pin", 24),
            })
        # None 값 제거
        kwargs = {k: v for k, v in kwargs.items() if v is not None}
        
        return GpioMotorHAL(
            calibration_path=cal_path,
            max_velocity=max_v,
            verbose=True,
            **kwargs,
        )
    except (ImportError, RuntimeError, NotImplementedError) as e:
        print(f"[Motor] GpioMotorHAL 사용 불가 ({e}), SimMotorHAL로 fallback")
        return SimMotorHAL(verbose=True)


__all__ = [
    'MotorHAL', 'MotorState', 'SimMotorHAL', 'create_motor_hal',
    'create_encoders',
    'MotorCalibration', 'ServoCalibration', 'DcMotorCalibration',
    'WHEEL_NAMES', 'STEERABLE_WHEELS', 'STEERABLE_NAMES',
    'FL', 'FR', 'ML', 'MR', 'RL', 'RR',
]


def create_encoders(config):
    """엔코더 리더 생성 (좌/우).
    
    real 모드에서만 동작. sim 모드 또는 lgpio 없으면 (None, None) 반환.
    
    config.encoder_pins 또는 config.motor_pins에서 핀 읽음:
        encoder_ml_c1_pin, encoder_ml_c2_pin (좌측 미들 ML)
        encoder_mr_c1_pin, encoder_mr_c2_pin (우측 미들 MR)
        encoder_counts_per_rev (기본 3960 = 11×4×90)
        encoder_wheel_circumference (기본 0.2042m)
    
    Returns:
        (encoder_left, encoder_right) — 각각 EncoderReader 또는 None
    """
    if config.is_simulation:
        return None, None
    
    try:
        from .encoder_reader import EncoderReader
    except ImportError as e:
        print(f"[Encoder] EncoderReader import 실패: {e}")
        return None, None
    
    pins = getattr(config, "motor_pins", None) or {}
    enc_pins = getattr(config, "encoder_pins", None) or {}
    cfg = {**pins, **enc_pins}  # encoder_pins가 우선
    
    ml_c1 = cfg.get("encoder_ml_c1_pin", 17)
    ml_c2 = cfg.get("encoder_ml_c2_pin", 27)
    mr_c1 = cfg.get("encoder_mr_c1_pin", 22)
    mr_c2 = cfg.get("encoder_mr_c2_pin", 5)
    counts_per_rev = cfg.get("encoder_counts_per_rev", 3960)
    circumference = cfg.get("encoder_wheel_circumference", 0.2042)
    invert_left = cfg.get("encoder_invert_left", False)
    invert_right = cfg.get("encoder_invert_right", False)
    
    try:
        left = EncoderReader(
            c1_pin=ml_c1, c2_pin=ml_c2,
            counts_per_rev=counts_per_rev,
            wheel_circumference_m=circumference,
            invert=invert_left,
        )
        right = EncoderReader(
            c1_pin=mr_c1, c2_pin=mr_c2,
            counts_per_rev=counts_per_rev,
            wheel_circumference_m=circumference,
            invert=invert_right,
        )
        if not left.is_available or not right.is_available:
            print("[Encoder] 일부 엔코더 초기화 실패")
            if left.is_available:
                left.shutdown()
            if right.is_available:
                right.shutdown()
            return None, None
        return left, right
    except Exception as e:
        print(f"[Encoder] 생성 실패: {e}")
        return None, None
