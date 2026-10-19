"""
Message Protocol

노트북 ↔ 라즈베리파이 통신용 메시지 정의.

[채널 분리]
1. UDP 9999: 영상 (라파 → 노트북)
   - 청크 분할 (한 프레임이 여러 패킷)
   - 손실 허용
   
2. TCP 9998: 제어 명령 (노트북 → 라파)
   - 손실 불가
   - 길이 prefix 방식 (4바이트 big-endian + 페이로드)
   
3. TCP 9997: 텔레메트리 (라파 → 노트북)
   - 손실 불가
   - 같은 길이 prefix 방식

[메시지 형식]
JSON으로 직렬화. dataclass → dict → JSON.
타입은 "type" 필드로 디스패치.

[버전 관리]
프로토콜 버전을 모든 메시지에 포함. 호환성 깨질 때 감지.
"""

import json
from dataclasses import dataclass, asdict, field
from typing import Optional, List, Dict, Any
from enum import Enum


PROTOCOL_VERSION = "1.0"


# ════════════════════════════════════════════════════════════════
# 메시지 타입 (디스패치용)
# ════════════════════════════════════════════════════════════════

class MessageType(str, Enum):
    # 노트북 → 라파 (제어)
    DRIVE = "drive"                  # 주행 명령 (throttle, steer)
    WHEEL_SIZE = "wheel_size"        # 바퀴 사이즈 변경
    STEERING_MODE = "steering_mode"  # 조향 모드 전환
    STOP = "stop"                    # 즉시 정지
    RETURN_TO_BASE = "return_to_base" # 자동 복귀
    
    # 라파 → 노트북 (텔레메트리)
    TELEMETRY = "telemetry"          # 센서/상태 종합
    DETECTION_RESULT = "detection"   # YOLO 결과 (노트북→라파 방향이지만 같은 클래스 재사용 가능)
    LOG = "log"                      # 로그 메시지


# ════════════════════════════════════════════════════════════════
# 노트북 → 라파 (제어 메시지)
# ════════════════════════════════════════════════════════════════

@dataclass
class DriveCommand:
    """주행 명령. 매 프레임/매 키 입력에 갱신."""
    type: str = MessageType.DRIVE.value
    throttle: float = 0.0    # [-1, 1] 후진~전진
    steer: float = 0.0       # [-1, 1] 우~좌


@dataclass
class WheelSizeCommand:
    """바퀴 사이즈 변경.
    
    각 바퀴 독립 제어 (sizes 6개) 또는 그룹 제어 (front/middle/rear) 둘 다 지원.
    sizes가 None이 아니면 우선 사용, 아니면 front/middle/rear로 6개 확장.
    
    sizes = [FL, FR, ML, MR, RL, RR] (선택, 6개)
    front, middle, rear: 단순 인터페이스 (좌우 같은 값)
    """
    type: str = MessageType.WHEEL_SIZE.value
    front: float = 0.0       # [0, 1] 앞 (FL, FR)
    rear: float = 0.0        # [0, 1] 뒤 (RL, RR)
    middle: float = 0.0      # [0, 1] 중간 (ML, MR)
    sizes: Optional[List[float]] = None   # [FL, FR, ML, MR, RL, RR] - 비대칭 제어용


@dataclass
class SteeringModeCommand:
    """조향 모드 전환."""
    type: str = MessageType.STEERING_MODE.value
    mode: str = "Ackermann"  # "Ackermann", "SkidSteer", "Crab", "DoubleAckermann"


@dataclass
class StopCommand:
    """즉시 정지."""
    type: str = MessageType.STOP.value


@dataclass
class ReturnToBaseCommand:
    """통신 정상 상태에서도 강제 복귀 트리거."""
    type: str = MessageType.RETURN_TO_BASE.value


# ════════════════════════════════════════════════════════════════
# 라파 → 노트북 (텔레메트리)
# ════════════════════════════════════════════════════════════════

@dataclass
class UltrasonicReading:
    """초음파 센서 한 개의 측정"""
    name: str
    distance: float          # [m], 측정 안 됐으면 -1.0


@dataclass
class TelemetryMessage:
    """라파의 종합 상태. 매 N Hz 송신."""
    type: str = MessageType.TELEMETRY.value
    timestamp: float = 0.0   # [s] 라파 monotonic time
    
    # 자세 추정 (odometry 또는 EKF)
    pose: Dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "psi": 0.0})
    velocity: Dict[str, float] = field(default_factory=lambda: {"v_x": 0.0, "v_y": 0.0, "yaw_rate": 0.0})
    
    # 현재 상태
    steering_mode: str = "Ackermann"
    wheel_size: Dict[str, float] = field(default_factory=lambda: {"front": 0.5, "middle": 0.5, "rear": 0.5})
    
    # 센서
    ultrasonic: List[Dict[str, Any]] = field(default_factory=list)
    
    # 시스템 상태
    battery_voltage: float = 0.0   # [V], 0이면 측정 안 됨
    cpu_temp: float = 0.0          # [°C]
    
    # 마지막 받은 명령 정보 (디버깅용)
    last_command_age: float = 0.0  # [s] 마지막 명령으로부터 경과


@dataclass
class LogMessage:
    """라파에서 로그 메시지"""
    type: str = MessageType.LOG.value
    level: str = "INFO"      # "INFO", "WARNING", "ERROR"
    message: str = ""
    timestamp: float = 0.0


# ════════════════════════════════════════════════════════════════
# 직렬화/역직렬화
# ════════════════════════════════════════════════════════════════

def encode_message(msg) -> bytes:
    """dataclass → JSON bytes"""
    if hasattr(msg, '__dataclass_fields__'):
        d = asdict(msg)
    elif isinstance(msg, dict):
        d = msg
    else:
        raise TypeError(f"인코딩 불가능한 타입: {type(msg)}")
    
    d['_version'] = PROTOCOL_VERSION
    return json.dumps(d, ensure_ascii=False).encode('utf-8')


def decode_message(data: bytes) -> dict:
    """JSON bytes → dict (type 필드로 디스패치는 호출자 몫)"""
    return json.loads(data.decode('utf-8'))


# ════════════════════════════════════════════════════════════════
# 디스패처 (편의)
# ════════════════════════════════════════════════════════════════

def parse_command(data: bytes):
    """제어 명령을 적절한 dataclass로 변환"""
    msg = decode_message(data)
    msg_type = msg.get('type')
    
    # _version 등 메타 필드 제거
    payload = {k: v for k, v in msg.items() if not k.startswith('_')}
    
    if msg_type == MessageType.DRIVE.value:
        return DriveCommand(**payload)
    elif msg_type == MessageType.WHEEL_SIZE.value:
        return WheelSizeCommand(**payload)
    elif msg_type == MessageType.STEERING_MODE.value:
        return SteeringModeCommand(**payload)
    elif msg_type == MessageType.STOP.value:
        return StopCommand(**payload)
    elif msg_type == MessageType.RETURN_TO_BASE.value:
        return ReturnToBaseCommand(**payload)
    else:
        raise ValueError(f"알 수 없는 명령 타입: {msg_type}")


def parse_telemetry(data: bytes):
    """텔레메트리 메시지 파싱"""
    msg = decode_message(data)
    msg_type = msg.get('type')
    payload = {k: v for k, v in msg.items() if not k.startswith('_')}
    
    if msg_type == MessageType.TELEMETRY.value:
        return TelemetryMessage(**payload)
    elif msg_type == MessageType.LOG.value:
        return LogMessage(**payload)
    else:
        raise ValueError(f"알 수 없는 텔레메트리 타입: {msg_type}")
