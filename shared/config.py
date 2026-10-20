"""
Config Loader

YAML 파일을 읽어서 dataclass로 변환.
실행 모드(simulation/real)에 따라 다른 검증 규칙 적용.

설계 의도:
- 모든 물리 파라미터를 코드 밖으로 분리
- real 모드에서 0.0 같은 미설정값 발견 시 명확한 에러
- dataclass로 type hint 제공, IDE 자동완성 지원
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional
import yaml


# ════════════════════════════════════════════════════════════════
# Dataclass 정의
# ════════════════════════════════════════════════════════════════

@dataclass
class CameraConfig:
    device_index: int
    width: int
    height: int
    fps: int
    jpeg_quality: int
    send_width: int
    send_height: int


@dataclass
class WheelConfig:
    radius_min: float
    radius_max: float
    radius_default: float
    size_change_rate: float


@dataclass
class SteeringConfig:
    max_angle: float       # [rad]
    rate_limit: float      # [rad/s]
    front_only: bool


@dataclass
class MotionConfig:
    max_velocity: float       # [m/s]
    max_acceleration: float   # [m/s²]
    max_yaw_rate: float       # [rad/s]


@dataclass
class RobotConfig:
    wheelbase: float          # [m]
    middle_axle_offset: float # [m]
    track: float              # [m]
    num_wheels: int
    mass: float               # [kg]
    moment_of_inertia: float  # [kg·m²]
    wheel: WheelConfig
    steering: SteeringConfig
    motion: MotionConfig


@dataclass
class TireConfig:
    """Pacejka Magic Formula 계수"""
    B: float
    C: float
    D: float
    E: float


@dataclass
class PIDConfig:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float


@dataclass
class PurePursuitConfig:
    lookahead_distance: float
    min_lookahead: float
    max_lookahead: float


@dataclass
class ControlConfig:
    loop_rate: int            # [Hz]
    velocity_pid: PIDConfig
    steering_pid: PIDConfig
    pure_pursuit: PurePursuitConfig


@dataclass
class UltrasonicSensor:
    name: str
    x: float       # 로봇 중심 기준 [m]
    y: float       # 로봇 중심 기준 [m]
    yaw: float     # 센서 향한 방향 [rad]


@dataclass
class UltrasonicConfig:
    enabled: bool
    update_rate: int          # [Hz]
    max_range: float          # [m]
    min_range: float          # [m]
    sensors: List[UltrasonicSensor]
    danger_threshold: float   # [m]
    warning_threshold: float  # [m]


@dataclass
class SensorsConfig:
    ultrasonic: UltrasonicConfig


@dataclass
class SimulationConfig:
    fake_obstacle_distance: float
    fake_obstacle_noise: float
    motor_response_delay: float


@dataclass
class Config:
    """루트 설정 객체"""
    mode: str                 # "simulation" or "real"
    camera: CameraConfig
    robot: RobotConfig
    tire: TireConfig
    control: ControlConfig
    sensors: SensorsConfig
    simulation: Optional[SimulationConfig] = None  # real 모드에선 None
    motor_pins: Optional[dict] = None              # 실기 GPIO 핀 매핑 (real 모드)
    encoder_pins: Optional[dict] = None            # 엔코더 핀/분해능 (real 모드)
    optical_flow_scale: float = 0.001              # px/sec → m/s 변환계수 (캘리브레이션 전 placeholder)

    @property
    def is_simulation(self) -> bool:
        return self.mode == "simulation"


# ════════════════════════════════════════════════════════════════
# 로딩 함수
# ════════════════════════════════════════════════════════════════

def load_config(path: str | Path) -> Config:
    """YAML 파일을 읽어 Config 객체로 변환하고 검증한다."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Config 파일이 없습니다: {path}")
    
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    
    config = _build_config(data)
    _validate_config(config)
    return config


def _build_config(data: dict) -> Config:
    """딕셔너리 → Config 객체"""
    robot_data = data['robot']
    sensors_data = data['sensors']
    control_data = data['control']
    
    config = Config(
        mode=data['mode'],
        camera=CameraConfig(**data['camera']),
        robot=RobotConfig(
            wheelbase=robot_data['wheelbase'],
            middle_axle_offset=robot_data['middle_axle_offset'],
            track=robot_data['track'],
            num_wheels=robot_data['num_wheels'],
            mass=robot_data['mass'],
            moment_of_inertia=robot_data['moment_of_inertia'],
            wheel=WheelConfig(**robot_data['wheel']),
            steering=SteeringConfig(**robot_data['steering']),
            motion=MotionConfig(**robot_data['motion']),
        ),
        tire=TireConfig(**data['tire']),
        control=ControlConfig(
            loop_rate=control_data['loop_rate'],
            velocity_pid=PIDConfig(**control_data['velocity_pid']),
            steering_pid=PIDConfig(**control_data['steering_pid']),
            pure_pursuit=PurePursuitConfig(**control_data['pure_pursuit']),
        ),
        sensors=SensorsConfig(
            ultrasonic=UltrasonicConfig(
                enabled=sensors_data['ultrasonic']['enabled'],
                update_rate=sensors_data['ultrasonic']['update_rate'],
                max_range=sensors_data['ultrasonic']['max_range'],
                min_range=sensors_data['ultrasonic']['min_range'],
                sensors=[UltrasonicSensor(**s) for s in sensors_data['ultrasonic']['sensors']],
                danger_threshold=sensors_data['ultrasonic']['danger_threshold'],
                warning_threshold=sensors_data['ultrasonic']['warning_threshold'],
            )
        ),
    )
    
    if 'simulation' in data and data['simulation']:
        config.simulation = SimulationConfig(**data['simulation'])
    
    if 'motor_pins' in data and data['motor_pins']:
        config.motor_pins = data['motor_pins']
    
    if 'encoder_pins' in data and data['encoder_pins']:
        config.encoder_pins = data['encoder_pins']
    
    if 'optical_flow_scale' in data:
        config.optical_flow_scale = float(data['optical_flow_scale'])
    
    return config


# ════════════════════════════════════════════════════════════════
# 검증
# ════════════════════════════════════════════════════════════════

class ConfigValidationError(ValueError):
    """Config 검증 실패"""
    pass


def _validate_config(config: Config) -> None:
    """모드별 검증 규칙 적용."""
    if config.mode not in ("simulation", "real"):
        raise ConfigValidationError(
            f"mode는 'simulation' 또는 'real'이어야 합니다. 현재: {config.mode}"
        )
    
    # 공통 검증
    _validate_camera(config.camera)
    _validate_sensors(config.sensors)
    
    # real 모드는 더 엄격하게
    if config.mode == "real":
        _validate_real_robot(config.robot)


def _validate_camera(cam: CameraConfig) -> None:
    if cam.width <= 0 or cam.height <= 0:
        raise ConfigValidationError("camera width/height가 0 이하")
    if not (0 < cam.jpeg_quality <= 100):
        raise ConfigValidationError(f"jpeg_quality는 1~100: {cam.jpeg_quality}")


def _validate_sensors(sensors: SensorsConfig) -> None:
    us = sensors.ultrasonic
    if us.min_range >= us.max_range:
        raise ConfigValidationError("ultrasonic min_range >= max_range")
    if us.danger_threshold >= us.warning_threshold:
        raise ConfigValidationError(
            "ultrasonic danger_threshold >= warning_threshold"
        )


def _validate_real_robot(robot: RobotConfig) -> None:
    """real 모드에서는 0.0 같은 placeholder 값 발견 시 에러."""
    msg_template = (
        "robot.{field}가 설정되지 않았습니다 (현재: {value}). "
        "기계팀 설계 확정 후 config/real.yaml을 업데이트하세요."
    )
    
    fields_to_check = [
        ("wheelbase", robot.wheelbase),
        ("track", robot.track),
        ("mass", robot.mass),
        ("moment_of_inertia", robot.moment_of_inertia),
        ("wheel.radius_min", robot.wheel.radius_min),
        ("wheel.radius_max", robot.wheel.radius_max),
        ("wheel.radius_default", robot.wheel.radius_default),
        ("steering.max_angle", robot.steering.max_angle),
        ("steering.rate_limit", robot.steering.rate_limit),
        ("motion.max_velocity", robot.motion.max_velocity),
        ("motion.max_acceleration", robot.motion.max_acceleration),
        ("motion.max_yaw_rate", robot.motion.max_yaw_rate),
    ]
    
    for field_name, value in fields_to_check:
        if value <= 0:
            raise ConfigValidationError(
                msg_template.format(field=field_name, value=value)
            )
    
    # 논리적 일관성
    if robot.wheel.radius_min >= robot.wheel.radius_max:
        raise ConfigValidationError("wheel.radius_min >= radius_max")
    if not (robot.wheel.radius_min <= robot.wheel.radius_default <= robot.wheel.radius_max):
        raise ConfigValidationError("wheel.radius_default가 [min, max] 범위 밖")


# ════════════════════════════════════════════════════════════════
# CLI 진입점 (테스트용)
# ════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Config 로드 테스트")
    parser.add_argument("path", help="config YAML 경로")
    args = parser.parse_args()
    
    config = load_config(args.path)
    print(f"[OK] Config 로드 성공 (mode={config.mode})")
    print(f"  - 로봇 wheelbase: {config.robot.wheelbase} m")
    print(f"  - 로봇 track: {config.robot.track} m")
    print(f"  - 초음파 센서 개수: {len(config.sensors.ultrasonic.sensors)}")
