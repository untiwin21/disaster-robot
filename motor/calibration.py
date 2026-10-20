"""
Motor Calibration

서보 10개 (변형 6 + 조향 4) + DC 모터 2개 (좌/우 그룹) 캘리브레이션 값 관리.

[조향 구조]
6륜이지만 조향은 앞/뒤 4개만 (Ackermann + 4WS).
가운데 바퀴 (ML, MR)는 조향 고정.

PCA9685 채널 매핑:
- 채널 0~5: 변형 서보 (FL, FR, ML, MR, RL, RR 바퀴 사이즈)
- 채널 6: 조향 FL
- 채널 7: 조향 FR
- 채널 8: 조향 RL
- 채널 9: 조향 RR

[캘리브레이션이 왜 필요한가]
1. 서보마다 미세하게 다름 (생산 편차)
2. 메커니즘별 가용 범위 다름
3. DC 모터 시작 임계값 (PWM 너무 낮으면 발열만)

[저장 형식]
calibration.yaml 파일. 사람이 읽고 수동 편집 가능.
"""

from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, List, Optional
import yaml
import math


# ════════════════════════════════════════════════════════════════
# 절대 안전 한계 (이 값 절대 못 넘김 - 부품 보호)
# ════════════════════════════════════════════════════════════════
ABSOLUTE_MIN_PULSE_US = 500     # 어떤 서보든 이 아래는 위험
ABSOLUTE_MAX_PULSE_US = 2500    # 어떤 서보든 이 위는 위험
DEFAULT_CENTER_PULSE_US = 1500  # 표준 중간값
SAFE_MIN_PULSE_US = 1000        # 일반 서보 안전 범위
SAFE_MAX_PULSE_US = 2000

# 바퀴 인덱스
FL, FR, ML, MR, RL, RR = 0, 1, 2, 3, 4, 5
WHEEL_NAMES = ["FL", "FR", "ML", "MR", "RL", "RR"]

# 조향 가능한 바퀴 (앞/뒤만, 가운데 제외)
STEERABLE_WHEELS = [FL, FR, RL, RR]
STEERABLE_NAMES = ["FL", "FR", "RL", "RR"]


@dataclass
class ServoCalibration:
    """단일 서보의 캘리브레이션 값"""
    channel: int                          # PCA9685 채널 (0~15)
    name: str = ""                        # 사람이 읽을 이름
    
    # 펄스 폭 한계 [us]
    min_pulse_us: int = SAFE_MIN_PULSE_US
    center_pulse_us: int = DEFAULT_CENTER_PULSE_US
    max_pulse_us: int = SAFE_MAX_PULSE_US
    
    # 펄스 → 값 매핑
    # 조향: 각도 [rad]. 변형: 사이즈 [0,1]
    min_value: float = -math.pi/4
    max_value: float = math.pi/4
    
    # 방향 반전 (좌/우 대칭 보정)
    inverted: bool = False
    
    def value_to_pulse(self, value: float) -> int:
        """값(각도 또는 사이즈) → 펄스폭 [us]
        
        매핑 방식:
        - bipolar (min<0<max): center가 0, min/max가 양 끝
        - unipolar: min~max 단순 선형 보간
        """
        # 클리핑
        v = max(self.min_value, min(self.max_value, value))
        if self.inverted:
            v = self.max_value + self.min_value - v
        
        bipolar = (self.min_value < 0) and (self.max_value > 0)
        
        if bipolar:
            if v >= 0:
                ratio = v / self.max_value if self.max_value != 0 else 0.0
                pulse = self.center_pulse_us + (self.max_pulse_us - self.center_pulse_us) * ratio
            else:
                ratio = v / self.min_value if self.min_value != 0 else 0.0
                pulse = self.center_pulse_us + (self.min_pulse_us - self.center_pulse_us) * ratio
        else:
            span = self.max_value - self.min_value
            if span == 0:
                pulse = self.center_pulse_us
            else:
                ratio = (v - self.min_value) / span
                pulse = self.min_pulse_us + (self.max_pulse_us - self.min_pulse_us) * ratio
        
        pulse_int = int(pulse)
        return max(ABSOLUTE_MIN_PULSE_US, min(ABSOLUTE_MAX_PULSE_US, pulse_int))


@dataclass
class DcMotorCalibration:
    """단일 DC 모터 그룹 (좌 또는 우)"""
    name: str = ""
    
    # PWM duty 범위 [0,1]
    min_duty: float = 0.10        # 시작 임계 (이 아래는 발열만)
    max_duty: float = 0.95        # 안전 최대
    
    # 방향 반전 (좌/우 결선 대칭이라 한 쪽 반전 필요할 수도)
    inverted: bool = False
    
    def velocity_to_duty(self, normalized_velocity: float) -> tuple[float, int]:
        """정규화 속도 [-1, 1] → (duty, direction)
        
        Returns:
            (duty, direction): duty는 [0, max_duty], direction은 1 또는 -1
        """
        v = max(-1.0, min(1.0, normalized_velocity))
        if self.inverted:
            v = -v
        
        direction = 1 if v >= 0 else -1
        abs_v = abs(v)
        
        if abs_v < 1e-3:
            return (0.0, direction)
        
        duty = self.min_duty + (self.max_duty - self.min_duty) * abs_v
        return (duty, direction)


@dataclass
class MotorCalibration:
    """전체 모터 시스템 캘리브레이션"""
    
    # PCA9685 채널 0~5: 변형 서보 6개 (각 바퀴 사이즈 조절)
    # PCA9685 채널 6~9: 조향 서보 4개 (FL, FR, RL, RR만)
    transform_servos: List[ServoCalibration] = field(default_factory=list)
    steer_servos: List[ServoCalibration] = field(default_factory=list)
    
    # DC 모터 좌/우 그룹
    dc_left: DcMotorCalibration = field(default_factory=DcMotorCalibration)
    dc_right: DcMotorCalibration = field(default_factory=DcMotorCalibration)
    
    @classmethod
    def default(cls) -> "MotorCalibration":
        """기본값으로 생성 (캘리브레이션 전 사용)"""
        cal = cls()
        
        # 변형 서보 6개: 채널 0~5, 사이즈 [0, 1]
        for i, name in enumerate(WHEEL_NAMES):
            cal.transform_servos.append(ServoCalibration(
                channel=i,
                name=f"transform_{name}",
                min_pulse_us=SAFE_MIN_PULSE_US,
                center_pulse_us=DEFAULT_CENTER_PULSE_US,
                max_pulse_us=SAFE_MAX_PULSE_US,
                min_value=0.0,
                max_value=1.0,
            ))
        
        # 조향 서보 4개: 채널 6~9, 각도 [-30도, +30도]
        for i, name in enumerate(STEERABLE_NAMES):
            cal.steer_servos.append(ServoCalibration(
                channel=6 + i,
                name=f"steer_{name}",
                min_pulse_us=SAFE_MIN_PULSE_US,
                center_pulse_us=DEFAULT_CENTER_PULSE_US,
                max_pulse_us=SAFE_MAX_PULSE_US,
                min_value=-math.pi/6,   # -30도
                max_value=math.pi/6,    # +30도
            ))
        
        cal.dc_left = DcMotorCalibration(name="dc_left", min_duty=0.10, max_duty=0.95)
        cal.dc_right = DcMotorCalibration(name="dc_right", min_duty=0.10, max_duty=0.95)
        
        return cal
    
    def save(self, path) -> None:
        """YAML 파일로 저장"""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        
        d = {
            "transform_servos": [asdict(s) for s in self.transform_servos],
            "steer_servos": [asdict(s) for s in self.steer_servos],
            "dc_left": asdict(self.dc_left),
            "dc_right": asdict(self.dc_right),
        }
        with open(path, "w", encoding="utf-8") as f:
            yaml.dump(d, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    
    @classmethod
    def load(cls, path) -> "MotorCalibration":
        """YAML에서 로드. 파일 없으면 기본값 반환."""
        path = Path(path)
        if not path.exists():
            print(f"[Calibration] {path} 없음 → 기본값 사용")
            return cls.default()
        
        with open(path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f) or {}
        
        cal = cls()
        cal.transform_servos = [ServoCalibration(**s) for s in d.get("transform_servos", [])]
        cal.steer_servos = [ServoCalibration(**s) for s in d.get("steer_servos", [])]
        if "dc_left" in d:
            cal.dc_left = DcMotorCalibration(**d["dc_left"])
        if "dc_right" in d:
            cal.dc_right = DcMotorCalibration(**d["dc_right"])
        
        # 누락 시 기본값 보충
        if not cal.transform_servos or not cal.steer_servos:
            print(f"[Calibration] {path} 불완전 → 기본값으로 보충")
            default = cls.default()
            if not cal.transform_servos:
                cal.transform_servos = default.transform_servos
            if not cal.steer_servos:
                cal.steer_servos = default.steer_servos
        
        return cal
