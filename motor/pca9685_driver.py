"""
PCA9685 Driver Wrapper

PCA9685 16채널 PWM 보드를 추상화. 라이브러리 의존성을 한 곳에 모음.

[하드웨어]
- I2C 통신 (기본 주소 0x40)
- 16채널 PWM 출력
- 12-bit 해상도 (0~4095)
- 주파수 24Hz~1526Hz (서보용 50Hz 표준)

[라이브러리]
adafruit-circuitpython-pca9685 사용. 설치:
    pip install adafruit-circuitpython-pca9685
    sudo apt install python3-smbus i2c-tools

[I2C 활성화]
    sudo raspi-config → Interface Options → I2C → Enable
    sudo reboot
    
연결 확인:
    sudo i2cdetect -y 1
    # 0x40 보여야 함

[사용]
    pca = PCA9685Driver()
    pca.set_pulse_us(channel=0, pulse_us=1500)  # 펄스폭 직접 지정
    pca.shutdown()
"""

from typing import Optional


# 표준 서보 주파수
DEFAULT_FREQ_HZ = 50

# 12-bit 해상도
PCA9685_RESOLUTION = 4096


class PCA9685Driver:
    """PCA9685 PWM 보드 추상화. 펄스폭(us) 단위로 사용."""
    
    def __init__(self,
                 i2c_address: int = 0x40,
                 frequency_hz: int = DEFAULT_FREQ_HZ,
                 i2c_bus_number: int = 1):
        """
        Args:
            i2c_address: PCA9685 I2C 주소 (기본 0x40)
            frequency_hz: PWM 주파수 (서보는 50Hz 표준)
            i2c_bus_number: 라파의 I2C 버스 번호 (기본 1)
        """
        self._frequency_hz = frequency_hz
        self._period_us = 1_000_000 / frequency_hz   # 50Hz면 20000us
        self._available = False
        self._pca = None
        
        # 라이브러리 import 시도
        try:
            import board
            import busio
            from adafruit_pca9685 import PCA9685
            
            i2c = busio.I2C(board.SCL, board.SDA)
            self._pca = PCA9685(i2c, address=i2c_address)
            self._pca.frequency = frequency_hz
            self._available = True
            print(f"[PCA9685] 연결 완료 (0x{i2c_address:02X}, {frequency_hz}Hz)")
        except ImportError as e:
            print(f"[PCA9685] 라이브러리 없음: {e}")
            print("[PCA9685] 설치: pip install adafruit-circuitpython-pca9685")
        except Exception as e:
            print(f"[PCA9685] 초기화 실패: {e}")
            print("[PCA9685] I2C 활성화 + 0x40 주소 연결 확인")
    
    @property
    def is_available(self) -> bool:
        return self._available
    
    def set_pulse_us(self, channel: int, pulse_us: float) -> None:
        """채널에 펄스폭(us) 출력
        
        Args:
            channel: 0~15
            pulse_us: 펄스폭 [us]. 서보 50Hz면 보통 500~2500.
        """
        if not self._available:
            return
        if not (0 <= channel <= 15):
            raise ValueError(f"채널 0~15 범위, 받음: {channel}")
        
        # us → 12-bit duty (0~4095)
        duty_fraction = pulse_us / self._period_us
        duty_16bit = int(duty_fraction * 0xFFFF)
        duty_16bit = max(0, min(0xFFFF, duty_16bit))
        
        self._pca.channels[channel].duty_cycle = duty_16bit
    
    def set_duty(self, channel: int, duty: float) -> None:
        """채널에 듀티 사이클 [0,1] 출력 (DC 모터 PWM용 등)"""
        if not self._available:
            return
        if not (0 <= channel <= 15):
            raise ValueError(f"채널 0~15 범위, 받음: {channel}")
        
        duty = max(0.0, min(1.0, duty))
        duty_16bit = int(duty * 0xFFFF)
        self._pca.channels[channel].duty_cycle = duty_16bit
    
    def disable(self, channel: int) -> None:
        """채널 신호 끔 (서보 힘 빠짐)"""
        if not self._available:
            return
        self._pca.channels[channel].duty_cycle = 0
    
    def disable_all(self) -> None:
        """모든 채널 끔"""
        if not self._available:
            return
        for ch in range(16):
            self._pca.channels[ch].duty_cycle = 0
    
    def shutdown(self) -> None:
        """리소스 정리"""
        if not self._available:
            return
        try:
            self.disable_all()
            self._pca.deinit()
        except Exception as e:
            print(f"[PCA9685] 종료 중 에러: {e}")
        finally:
            self._available = False
