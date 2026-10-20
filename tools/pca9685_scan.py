"""
PCA9685 연결 확인 도구

라파에서 가장 먼저 실행. I2C가 활성화돼있고 PCA9685가 연결됐는지 확인.

[사용법]
    python3 tools/pca9685_scan.py

[기대 출력 (성공)]
    [Scan] I2C 버스 1 스캔 중...
    [Scan] 발견: 0x40 (PCA9685 가능성)
    [Scan] PCA9685 직접 연결 시도...
    [Scan] ✓ 연결 성공! 주파수 50Hz 설정됨
    [Scan] 채널 0 펄스 1500us 출력 (1초)...
    [Scan] 완료. 서보 1개 채널 0에 연결해서 중간 위치인지 확인.

[실패 시]
    1. I2C 활성화 확인:
       sudo raspi-config → Interface Options → I2C → Enable
       sudo reboot
    
    2. 연결 확인:
       sudo i2cdetect -y 1
       # 0x40 표시돼야 함
    
    3. 라이브러리 설치:
       pip install adafruit-circuitpython-pca9685
       sudo apt install python3-smbus i2c-tools
"""

import sys
import time
from pathlib import Path

# 경로 설정
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def scan_i2c():
    """I2C 버스에 연결된 디바이스 검색"""
    print("[Scan] I2C 버스 1 스캔 중...")
    
    try:
        from smbus2 import SMBus
    except ImportError:
        try:
            from smbus import SMBus
        except ImportError:
            print("[Scan] smbus 라이브러리 없음. 설치:")
            print("  sudo apt install python3-smbus")
            return []
    
    found = []
    try:
        with SMBus(1) as bus:
            for addr in range(0x03, 0x78):
                try:
                    bus.read_byte(addr)
                    found.append(addr)
                except OSError:
                    pass
    except FileNotFoundError:
        print("[Scan] /dev/i2c-1 없음. I2C 활성화 안 됨.")
        print("  sudo raspi-config → Interface Options → I2C → Enable")
        print("  sudo reboot")
        return []
    except Exception as e:
        print(f"[Scan] I2C 스캔 에러: {e}")
        return []
    
    if not found:
        print("[Scan] 연결된 I2C 디바이스 없음")
    else:
        for addr in found:
            label = " (PCA9685 가능성)" if addr == 0x40 else ""
            print(f"[Scan] 발견: 0x{addr:02X}{label}")
    
    return found


def test_pca9685():
    """PCA9685에 직접 연결 + 채널 0 테스트"""
    print("\n[Scan] PCA9685 직접 연결 시도...")
    
    try:
        from motor.pca9685_driver import PCA9685Driver
    except ImportError as e:
        print(f"[Scan] 드라이버 import 실패: {e}")
        return False
    
    pca = PCA9685Driver()
    if not pca.is_available:
        print("[Scan] ✗ PCA9685 연결 실패")
        return False
    
    print("[Scan] ✓ 연결 성공! 주파수 50Hz 설정됨")
    print("[Scan] 채널 0 펄스 1500us 출력 (1초)...")
    pca.set_pulse_us(channel=0, pulse_us=1500)
    time.sleep(1)
    print("[Scan] 채널 0 펄스 1000us 출력 (1초)...")
    pca.set_pulse_us(channel=0, pulse_us=1000)
    time.sleep(1)
    print("[Scan] 채널 0 펄스 2000us 출력 (1초)...")
    pca.set_pulse_us(channel=0, pulse_us=2000)
    time.sleep(1)
    print("[Scan] 채널 0 정지...")
    pca.disable(0)
    
    pca.shutdown()
    print("[Scan] 완료. 채널 0 서보가 1000→1500→2000us로 움직였는지 확인.")
    return True


def main():
    print("=" * 60)
    print("PCA9685 연결 확인 도구")
    print("=" * 60)
    
    # 1. I2C 스캔
    devices = scan_i2c()
    
    # 2. PCA9685 직접 연결
    if 0x40 in devices:
        test_pca9685()
    else:
        print("\n[Scan] 0x40에 디바이스 없음. PCA9685 연결 확인:")
        print("  1. SDA → GPIO 2, SCL → GPIO 3 연결")
        print("  2. PCA9685 V+ 외부 6V 전원 (서보용)")
        print("  3. PCA9685 VCC 5V (라파 5V 또는 외부)")
        print("  4. GND 공통 접지")


if __name__ == "__main__":
    main()
