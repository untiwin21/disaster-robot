"""
Servo Calibration Tool

12개 서보 (변형 6 + 조향 6) 의 min/center/max 펄스폭 측정.
결과를 calibration.yaml 에 저장.

[사용법]
    python3 raspberry-pi/tools/servo_calibration.py

[키 조작]
    채널 선택:
        Tab     : 다음 채널
        Shift+Tab : 이전 채널
        0~9, q,w,e,r : 채널 0~11 직접 선택
    
    펄스 조작:
        ←/→     : -10us / +10us
        ↑/↓     : -50us / +50us
        Page Up/Down : -1us / +1us (미세 조정)
        Home    : 1500us 리셋
    
    기록:
        s       : 현재 펄스를 MIN으로 저장
        c       : 현재 펄스를 CENTER로 저장
        d       : 현재 펄스를 MAX로 저장
        r       : 현재 채널 캘리브레이션 리셋 (기본값으로)
    
    파일:
        F2      : calibration.yaml 저장
        F3      : 자동 sweep (현재 채널을 min→center→max→center 천천히 이동)
    
    종료:
        q 또는 Esc : 종료 (저장 안 함, 별도 F2 필요)

[권장 절차]
    1. PCA9685 + 서보 12개 연결 + 전원 인가
    2. 메커니즘 조립 완료 (사이즈 조절, 조향 링크)
    3. 이 도구 실행
    4. 채널 0부터 차례로:
       a. 1500us에서 시작
       b. 천천히 펄스 줄이며 한쪽 끝 한계 찾기 → s 저장
       c. 천천히 펄스 늘이며 반대쪽 한계 찾기 → d 저장
       d. 직진/중간 위치로 이동 → c 저장
    5. F2 저장
    6. 다른 도구 (메인 코드)에서 자동 사용

[안전]
    - 절대 한계 (500~2500us) 자동 적용
    - 천천히 이동 (급격한 움직임 방지)
    - 메커니즘에 무리 가는 소리/충돌 보이면 즉시 Ctrl+C
"""

import sys
import time
import os
from pathlib import Path

# 경로 설정
ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "raspberry-pi"))

from motor.calibration import (
    MotorCalibration, ServoCalibration,
    ABSOLUTE_MIN_PULSE_US, ABSOLUTE_MAX_PULSE_US,
    SAFE_MIN_PULSE_US, SAFE_MAX_PULSE_US,
    DEFAULT_CENTER_PULSE_US,
)


CALIBRATION_FILE = ROOT / "config" / "calibration.yaml"


def get_channel_label(channel: int, cal: MotorCalibration) -> str:
    """채널 번호 → 사람이 읽는 이름"""
    for s in cal.transform_servos:
        if s.channel == channel:
            return f"ch{channel} ({s.name})"
    for s in cal.steer_servos:
        if s.channel == channel:
            return f"ch{channel} ({s.name})"
    return f"ch{channel}"


def find_servo_cal(channel: int, cal: MotorCalibration):
    """채널에 해당하는 ServoCalibration 찾기"""
    for s in cal.transform_servos:
        if s.channel == channel:
            return s
    for s in cal.steer_servos:
        if s.channel == channel:
            return s
    return None


def clear_screen():
    os.system('clear' if os.name == 'posix' else 'cls')


def print_status(channel: int, current_pulse: int, cal: MotorCalibration, msg: str = ""):
    """현재 상태 화면 그리기"""
    clear_screen()
    print("=" * 70)
    print("Servo Calibration Tool".center(70))
    print("=" * 70)
    print()
    
    label = get_channel_label(channel, cal)
    print(f"  현재 채널: {label}")
    print(f"  현재 펄스: {current_pulse} us")
    
    servo = find_servo_cal(channel, cal)
    if servo:
        print(f"  현재 캘: MIN={servo.min_pulse_us}  CENTER={servo.center_pulse_us}  MAX={servo.max_pulse_us}")
    
    print()
    print("  ─────────────────────────────────────────────────────")
    print("  [Tab/Shift+Tab] 채널 변경    [0~9 q w e r] 직접 선택")
    print("  [←/→] ±10us    [↑/↓] ±50us    [Home] 1500us 리셋")
    print("  [s] MIN 저장   [c] CENTER 저장   [d] MAX 저장")
    print("  [F2 / w] 파일 저장   [F3] sweep 테스트   [Esc/q] 종료")
    print("  ─────────────────────────────────────────────────────")
    print()
    if msg:
        print(f"  >>> {msg}")
        print()


def get_key():
    """단일 키 입력 (Linux/Mac terminal raw mode)"""
    import termios, tty
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        # ESC 시퀀스 (방향키 등)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                if ch3 == 'A': return 'UP'
                if ch3 == 'B': return 'DOWN'
                if ch3 == 'C': return 'RIGHT'
                if ch3 == 'D': return 'LEFT'
                if ch3 == 'H': return 'HOME'
                if ch3 == 'Z': return 'SHIFT_TAB'
                if ch3 in '5':
                    sys.stdin.read(1)  # ~
                    return 'PGUP'
                if ch3 in '6':
                    sys.stdin.read(1)
                    return 'PGDN'
                # F2, F3 같은 거
                if ch3 == '1':
                    rest = sys.stdin.read(2)
                    if rest == '2~': return 'F2'
                    if rest == '3~': return 'F3'
                return f'ESC[{ch3}'
            return f'ESC{ch2}'
        if ch == '\t':
            return 'TAB'
        if ch == '\x03':  # Ctrl+C
            return 'CTRL_C'
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def safe_pulse(pulse: int) -> int:
    """절대 안전 한계 적용"""
    return max(ABSOLUTE_MIN_PULSE_US, min(ABSOLUTE_MAX_PULSE_US, pulse))


def sweep_channel(pca, channel: int, cal: MotorCalibration, current_pulse: int):
    """현재 채널 sweep 테스트 (min → center → max → center, 천천히)"""
    servo = find_servo_cal(channel, cal)
    if servo is None:
        return current_pulse
    
    print(f"\n  Sweep 시작: {servo.min_pulse_us} → {servo.center_pulse_us} → {servo.max_pulse_us} → {servo.center_pulse_us}")
    print("  Ctrl+C 로 중단")
    
    waypoints = [
        servo.min_pulse_us,
        servo.center_pulse_us,
        servo.max_pulse_us,
        servo.center_pulse_us,
    ]
    
    try:
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            step = 5 if end > start else -5
            for p in range(start, end + step, step):
                p_safe = safe_pulse(p)
                pca.set_pulse_us(channel, p_safe)
                time.sleep(0.02)
            time.sleep(0.5)  # 각 지점 잠시 멈춤
        return waypoints[-1]
    except KeyboardInterrupt:
        print("\n  Sweep 중단")
        return current_pulse


def main():
    # 캘리브레이션 로드
    cal = MotorCalibration.load(CALIBRATION_FILE) if CALIBRATION_FILE.exists() else MotorCalibration.default()
    
    # PCA9685 연결
    try:
        from motor.pca9685_driver import PCA9685Driver
    except ImportError as e:
        print(f"드라이버 import 실패: {e}")
        return
    
    pca = PCA9685Driver()
    if not pca.is_available:
        print("PCA9685 연결 안 됨. pca9685_scan.py 먼저 실행.")
        return
    
    # 시작 상태
    channel = 0
    current_pulse = DEFAULT_CENTER_PULSE_US
    msg = "준비 완료. 채널 0부터 시작."
    
    pca.set_pulse_us(channel, current_pulse)
    print_status(channel, current_pulse, cal, msg)
    
    try:
        while True:
            key = get_key()
            msg = ""
            
            # 종료
            if key in ('q', 'Q', 'CTRL_C') or (len(key) == 1 and ord(key) == 27):
                break
            
            # 채널 변경
            elif key == 'TAB':
                channel = (channel + 1) % 12
                current_pulse = DEFAULT_CENTER_PULSE_US
                pca.set_pulse_us(channel, current_pulse)
                msg = f"채널 {channel} 선택"
            elif key == 'SHIFT_TAB':
                channel = (channel - 1) % 12
                current_pulse = DEFAULT_CENTER_PULSE_US
                pca.set_pulse_us(channel, current_pulse)
                msg = f"채널 {channel} 선택"
            
            # 채널 직접 선택 (0~9, q,w,e,r → 10,11... 근데 q는 종료라 안 씀)
            elif key in '0123456789':
                channel = int(key)
                current_pulse = DEFAULT_CENTER_PULSE_US
                pca.set_pulse_us(channel, current_pulse)
                msg = f"채널 {channel} 선택"
            elif key == 'w':
                channel = 10
                current_pulse = DEFAULT_CENTER_PULSE_US
                pca.set_pulse_us(channel, current_pulse)
                msg = f"채널 {channel} 선택"
            elif key == 'e':
                channel = 11
                current_pulse = DEFAULT_CENTER_PULSE_US
                pca.set_pulse_us(channel, current_pulse)
                msg = f"채널 {channel} 선택"
            
            # 펄스 조정
            elif key == 'LEFT':
                current_pulse = safe_pulse(current_pulse - 10)
                pca.set_pulse_us(channel, current_pulse)
            elif key == 'RIGHT':
                current_pulse = safe_pulse(current_pulse + 10)
                pca.set_pulse_us(channel, current_pulse)
            elif key == 'DOWN':
                current_pulse = safe_pulse(current_pulse - 50)
                pca.set_pulse_us(channel, current_pulse)
            elif key == 'UP':
                current_pulse = safe_pulse(current_pulse + 50)
                pca.set_pulse_us(channel, current_pulse)
            elif key == 'PGUP':
                current_pulse = safe_pulse(current_pulse + 1)
                pca.set_pulse_us(channel, current_pulse)
            elif key == 'PGDN':
                current_pulse = safe_pulse(current_pulse - 1)
                pca.set_pulse_us(channel, current_pulse)
            elif key == 'HOME':
                current_pulse = DEFAULT_CENTER_PULSE_US
                pca.set_pulse_us(channel, current_pulse)
                msg = "1500us 리셋"
            
            # 캘리브레이션 저장
            elif key == 's':
                servo = find_servo_cal(channel, cal)
                if servo:
                    servo.min_pulse_us = current_pulse
                    msg = f"MIN = {current_pulse}us 저장됨 (메모리). F2로 파일 저장."
            elif key == 'c':
                servo = find_servo_cal(channel, cal)
                if servo:
                    servo.center_pulse_us = current_pulse
                    msg = f"CENTER = {current_pulse}us 저장됨 (메모리). F2로 파일 저장."
            elif key == 'd':
                servo = find_servo_cal(channel, cal)
                if servo:
                    servo.max_pulse_us = current_pulse
                    msg = f"MAX = {current_pulse}us 저장됨 (메모리). F2로 파일 저장."
            
            # 파일 저장 (F2 또는 w - SSH 환경에서 F-key 안 먹을 때 대안)
            elif key == 'F2' or key == 'x':
                cal.save(CALIBRATION_FILE)
                msg = f"파일 저장 완료: {CALIBRATION_FILE}"
            
            # Sweep
            elif key == 'F3':
                current_pulse = sweep_channel(pca, channel, cal, current_pulse)
                msg = "Sweep 완료"
            
            print_status(channel, current_pulse, cal, msg)
    
    finally:
        # 종료 시 모든 채널 끔
        pca.disable_all()
        pca.shutdown()
        print()
        print("종료. F2 누르고 종료했어야 영구 저장됨.")
        print(f"파일: {CALIBRATION_FILE}")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n에러: {e}")
        import traceback
        traceback.print_exc()