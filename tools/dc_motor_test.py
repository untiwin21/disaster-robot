"""
DC Motor Test Tool

BTS7960 듀얼 채널 (좌/우 그룹) DC 모터를 키보드로 직접 조작.
- 정/역회전 동작 확인
- 시작 임계 듀티 (이 아래는 안 돌고 발열) 측정
- 좌/우 동기화 확인

[사용법]
    python3 tools/dc_motor_test.py

[키 조작]
    좌측 그룹:
        w : 듀티 +5%
        s : 듀티 -5%
        e : 방향 토글
        a : 좌 정지
    
    우측 그룹:
        i : 듀티 +5%
        k : 듀티 -5%
        o : 방향 토글
        j : 우 정지
    
    공통:
        SPACE : 모두 정지
        b     : 양쪽 모두 같은 명령으로 (직진 테스트)
        v     : 양쪽 반대 방향 (제자리 회전 테스트)
        q     : 종료

[캘리브레이션 측정]
    "이 듀티에서 처음으로 돌기 시작한다" 값을 찾는 게 목적:
    1. SPACE로 정지 시작
    2. w 또는 i 천천히 누르면서 듀티 올림
    3. 모터가 처음 도는 듀티 = min_duty (캘리브레이션 값)
    4. calibration.yaml의 dc_left/dc_right.min_duty 수동 편집
"""

import sys
import time
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def clear_screen():
    os.system('clear' if os.name == 'posix' else 'cls')


def get_key():
    import termios, tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x03':
            return 'CTRL_C'
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def print_status(left_duty: float, left_dir: int, right_duty: float, right_dir: int, msg: str):
    clear_screen()
    print("=" * 70)
    print("DC Motor Test Tool".center(70))
    print("=" * 70)
    print()
    
    def fmt(d, dir_):
        arrow = "→" if dir_ >= 0 else "←"
        return f"{arrow} {d*100:5.1f}%"
    
    print(f"  좌 그룹 (FL,ML,RL): {fmt(left_duty, left_dir)}")
    print(f"  우 그룹 (FR,MR,RR): {fmt(right_duty, right_dir)}")
    print()
    print("  ─────────────────────────────────────────────────────")
    print("  좌: [w] +5%  [s] -5%  [e] 방향  [a] 정지")
    print("  우: [i] +5%  [k] -5%  [o] 방향  [j] 정지")
    print("  공통: [SPACE] 모두 정지  [b] 양쪽 같이  [v] 양쪽 반대")
    print("       [q] 종료")
    print("  ─────────────────────────────────────────────────────")
    print()
    if msg:
        print(f"  >>> {msg}")
        print()


def main():
    try:
        from motor.bts7960_driver import BTS7960Driver
    except ImportError as e:
        print(f"라이브러리 없음: {e}")
        return
    
    # 회로도 기준 핀 (분기 없이 R_EN/L_EN 각각 직결, lgpio chip handle 자동 공유)
    left = BTS7960Driver(rpwm_pin=18, lpwm_pin=12, r_en_pin=6, l_en_pin=16)
    right = BTS7960Driver(rpwm_pin=19, lpwm_pin=13, r_en_pin=23, l_en_pin=24)
    
    if not left.is_available or not right.is_available:
        print("BTS7960 초기화 실패")
        return
    left.enable()
    right.enable()
    left_duty = 0.0
    left_dir = 1
    right_duty = 0.0
    right_dir = 1
    
    DUTY_STEP = 0.05
    
    msg = "준비 완료. 안전한 위치에서 시작 (바퀴 들어 올리거나 무부하 상태)."
    print_status(left_duty, left_dir, right_duty, right_dir, msg)
    
    try:
        while True:
            key = get_key()
            msg = ""
            
            if key in ('q', 'Q', 'CTRL_C'):
                break
            
            # 좌 그룹
            elif key == 'w':
                left_duty = min(1.0, left_duty + DUTY_STEP)
            elif key == 's':
                left_duty = max(0.0, left_duty - DUTY_STEP)
            elif key == 'e':
                left_dir *= -1
                msg = f"좌 방향 토글: {'정' if left_dir > 0 else '역'}"
            elif key == 'a':
                left_duty = 0.0
            
            # 우 그룹
            elif key == 'i':
                right_duty = min(1.0, right_duty + DUTY_STEP)
            elif key == 'k':
                right_duty = max(0.0, right_duty - DUTY_STEP)
            elif key == 'o':
                right_dir *= -1
                msg = f"우 방향 토글: {'정' if right_dir > 0 else '역'}"
            elif key == 'j':
                right_duty = 0.0
            
            # 공통
            elif key == ' ':
                left_duty = 0.0
                right_duty = 0.0
                msg = "모두 정지"
            elif key == 'b':
                # 양쪽 같이 (직진)
                avg = (left_duty + right_duty) / 2
                left_duty = right_duty = avg
                left_dir = right_dir = 1
                msg = "양쪽 같은 방향, 같은 듀티"
            elif key == 'v':
                # 양쪽 반대 (제자리 회전)
                avg = (left_duty + right_duty) / 2
                left_duty = right_duty = avg
                left_dir = 1
                right_dir = -1
                msg = "양쪽 반대 방향 (제자리 회전)"
            
            # 적용
            left.set(left_duty, left_dir)
            right.set(right_duty, right_dir)
            
            print_status(left_duty, left_dir, right_duty, right_dir, msg)
    
    finally:
        left.stop()
        right.stop()
        left.disable()
        right.disable()
        left.shutdown()
        right.shutdown()
        print()
        print("종료. 측정한 min_duty 값을 calibration.yaml 의")
        print("  dc_left.min_duty, dc_right.min_duty 에 수동 입력해.")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n에러: {e}")
        import traceback
        traceback.print_exc()