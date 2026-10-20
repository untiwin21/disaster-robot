"""
Ultrasonic Sensor HAL

여러 초음파 센서(HC-SR04 등)를 추상화.
시뮬용 가짜 + 실제 GPIO용 stub.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional
import math
import random
import time


@dataclass
class UltrasonicReading:
    """단일 센서 측정값"""
    name: str
    distance: float       # [m], 측정 실패면 -1.0
    timestamp: float      # [s] monotonic time
    
    def is_valid(self) -> bool:
        return self.distance > 0


class UltrasonicHAL(ABC):
    """초음파 센서 추상 인터페이스"""
    
    @abstractmethod
    def read_all(self) -> List[UltrasonicReading]:
        """모든 센서 읽기"""
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        pass


# ════════════════════════════════════════════════════════════════
# 시뮬용
# ════════════════════════════════════════════════════════════════

class SimUltrasonicHAL(UltrasonicHAL):
    """가짜 초음파. 시간에 따라 변하는 가상 장애물 거리 반환."""
    
    def __init__(self, sensor_configs, base_distance: float = 1.5,
                 noise_std: float = 0.05, max_range: float = 4.0,
                 min_range: float = 0.02):
        """
        Args:
            sensor_configs: List[UltrasonicSensor] from config
            base_distance: 기본 가상 장애물 거리 [m]
            noise_std: 측정 노이즈 표준편차 [m]
        """
        self.sensors = sensor_configs
        self.base_distance = base_distance
        self.noise_std = noise_std
        self.max_range = max_range
        self.min_range = min_range
        self._start_time = time.monotonic()
    
    def read_all(self) -> List[UltrasonicReading]:
        """방향별로 다른 가상 장애물 패턴 생성 (시연 효과)"""
        now = time.monotonic()
        elapsed = now - self._start_time
        
        readings = []
        for sensor in self.sensors:
            # 방향에 따라 다른 패턴
            # - front: 천천히 가까워지는 장애물
            # - rear/sides: 일정 거리 + 노이즈
            if sensor.name == "front":
                # 5초 주기로 1.5m → 0.3m 변화 (sin wave)
                dist = 0.9 + 0.6 * math.sin(elapsed * 0.4)
            elif "front" in sensor.name:
                dist = self.base_distance + 0.3 * math.sin(elapsed * 0.5 + hash(sensor.name) % 10)
            else:
                dist = self.base_distance + 0.5
            
            # 노이즈 추가
            dist += random.gauss(0, self.noise_std)
            
            # 측정 범위 클램프
            if dist > self.max_range:
                dist = -1.0  # 측정 실패
            elif dist < self.min_range:
                dist = self.min_range
            
            readings.append(UltrasonicReading(
                name=sensor.name,
                distance=dist,
                timestamp=now,
            ))
        return readings
    
    def shutdown(self) -> None:
        pass


# ════════════════════════════════════════════════════════════════
# 실제 GPIO용 (stub)
# ════════════════════════════════════════════════════════════════

class GpioUltrasonicHAL(UltrasonicHAL):
    """
    실제 HC-SR04 GPIO 제어. 하드웨어 도착 후 구현.
    
    [HC-SR04 동작]
    1. TRIG 핀에 10μs 펄스 보냄
    2. ECHO 핀이 HIGH 되는 시간 측정
    3. 거리 = (시간 × 343 m/s) / 2
    
    [핀 부족 시 대안]
    - GPIO 확장 보드 (MCP23017 등)
    - 멀티플렉서로 한 ECHO 핀 공유
    """
    
    def __init__(self, sensor_configs, pin_mappings):
        raise NotImplementedError(
            "GpioUltrasonicHAL: 하드웨어 결정 후 구현 필요"
        )
    
    def read_all(self) -> List[UltrasonicReading]:
        raise NotImplementedError
    
    def shutdown(self) -> None:
        raise NotImplementedError


def create_ultrasonic_hal(config) -> UltrasonicHAL:
    """Config 기반 자동 생성"""
    us_config = config.sensors.ultrasonic
    
    if config.is_simulation:
        return SimUltrasonicHAL(
            sensor_configs=us_config.sensors,
            base_distance=config.simulation.fake_obstacle_distance if config.simulation else 1.5,
            noise_std=config.simulation.fake_obstacle_noise if config.simulation else 0.05,
            max_range=us_config.max_range,
            min_range=us_config.min_range,
        )
    
    try:
        return GpioUltrasonicHAL(us_config.sensors, pin_mappings={})
    except (ImportError, NotImplementedError) as e:
        print(f"[Sensor] GPIO 초음파 사용 불가 ({e}), 시뮬 사용")
        return SimUltrasonicHAL(
            sensor_configs=us_config.sensors,
            max_range=us_config.max_range,
            min_range=us_config.min_range,
        )
