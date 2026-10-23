"""
Raspberry Pi Main

SSH 터미널에서 로봇을 직접 조작한다. 네트워크/GUI 원격 제어는 없다.

    [SSH 키 입력] ──▶ [좌/우 모터 직접 제어] ──▶ [Motor HAL]

W/S는 왼쪽 모터, ↑/↓는 오른쪽 모터를 앞뒤로 돌린다 (탱크식). 조향 서보는 중앙 고정.

[안전]
- 주행 키 입력이 끊기면 데드맨 타이머가 모터를 정지
- 종료/예외 시 모터 정지 + GPIO 해제
"""

import sys
import os
import time
import argparse
from pathlib import Path

# 경로 설정
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from shared.config import load_config, Config

from motor import SimMotorHAL, create_motor_hal


# 주행 키 → (모터, 방향). 모터 0: 왼쪽, 1: 오른쪽
DRIVE_KEYS = {
    "w": (0, 1), "s": (0, -1),
    "up": (1, 1), "down": (1, -1),
}


def split_keys(buf: str) -> tuple[list[str], str]:
    """입력 버퍼를 키 단위로 나눈다. 화살표 ↑/↓는 "up"/"down"으로 바꾼다.

    끝이 잘린 escape 시퀀스는 다음 입력과 이어 붙이도록 나머지로 돌려준다.
    """
    keys = []
    i = 0
    while i < len(buf):
        if buf[i] != "\x1b":
            keys.append(buf[i])
            i += 1
            continue
        if i + 1 >= len(buf):
            break
        if buf[i + 1] not in "[O":
            i += 1  # 단독 ESC는 무시
            continue
        # CSI(ESC [ ... final) / SS3(ESC O final). final 바이트는 0x40~0x7E
        j = i + 2
        while j < len(buf) and not ("\x40" <= buf[j] <= "\x7e"):
            j += 1
        if j >= len(buf):
            break
        if buf[j] == "A":
            keys.append("up")
        elif buf[j] == "B":
            keys.append("down")
        i = j + 1
    return keys, buf[i:]


def terminal_control(config: Config) -> None:
    """SSH TTY에서 네트워크/GUI 없이 로봇을 직접 조작한다."""
    if not sys.stdin.isatty():
        raise RuntimeError("터미널 제어는 TTY가 필요합니다. ssh -t 로 접속하세요.")

    import select
    import termios
    import tty

    if not config.is_simulation:
        print("[Terminal] 실제 모터와 서보를 초기화합니다.")

    motor = create_motor_hal(config)
    if not config.is_simulation and isinstance(motor, SimMotorHAL):
        motor.shutdown()
        raise RuntimeError("실제 모터 초기화에 실패하여 터미널 제어를 중단합니다.")
    motor.set_steer_angles([0.0] * 6)

    max_velocity = config.robot.motion.max_velocity
    drive = [0, 0]      # [왼쪽, 오른쪽] 1: 전진, -1: 후진, 0: 정지
    speed = 0.5         # 속도 단계 [0.1, 1.0] (출력 = 방향 × speed)
    speed_step = 0.1
    wheel_sizes = [0.0] * 6
    drive_deadline = 0.0
    last_drive_key = None
    deadman_timeout = 0.5
    # 키를 누른 직후에는 OS 키 반복 지연(보통 0.5초 이상)과 SSH 지연이 겹치므로
    # 새 키의 첫 입력에만 여유를 더 준다.
    initial_deadman_timeout = 1.0
    loop_interval = 1.0 / max(1, config.control.loop_rate)
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    pending = ""

    def show_status(message: str = "") -> None:
        status = (
            f"speed={speed * 100:3.0f}% left={drive[0] * speed:+.1f} right={drive[1] * speed:+.1f} "
            f"wheel_size={wheel_sizes[0]:.1f}"
        )
        if message:
            status += f" | {message}"
        print(f"\r{status:<110}", end="", flush=True)

    print("[Terminal] W/S: 왼쪽 모터 전진/후진  ↑/↓: 오른쪽 모터 전진/후진 (누르는 동안)")
    print("[Terminal] 1~9/0: 속도 10~90/100%  +/-: 속도 ±10%")
    print("[Terminal] R/F: 바퀴 크기 +/-  Space: 정지  Q: 종료")
    print(
        f"[Terminal] 주행 키 입력이 {deadman_timeout:.1f}초 끊기면 자동 정지합니다. "
        f"(첫 입력 후 {initial_deadman_timeout:.1f}초)"
    )

    try:
        tty.setcbreak(fd)
        show_status("준비 완료")
        running = True

        while running:
            now = time.monotonic()
            readable, _, _ = select.select([sys.stdin], [], [], 0)
            message = ""

            if readable:
                data = os.read(fd, 64)
                if not data:
                    message = "SSH 입력 종료"
                    break
                keys, pending = split_keys(pending + data.decode(errors="ignore"))

                for key in keys:
                    key_lower = key.lower()
                    if key_lower == "q" or key == "\x03":
                        message = "종료"
                        running = False
                        break
                    elif key == " ":
                        drive = [0, 0]
                        drive_deadline = 0.0
                        last_drive_key = None
                        motor.emergency_stop()
                        message = "EMERGENCY STOP"
                    elif key_lower in DRIVE_KEYS:
                        # 터미널은 마지막에 누른 키만 반복 입력하므로 데드맨은 좌/우 공용.
                        # 두 키를 같이 누르고 있으면 양쪽 모두 계속 돈다.
                        side, direction = DRIVE_KEYS[key_lower]
                        drive[side] = direction
                        timeout = deadman_timeout if key_lower == last_drive_key else initial_deadman_timeout
                        drive_deadline = max(drive_deadline, now + timeout)
                        last_drive_key = key_lower
                    elif key.isdigit():
                        speed = 1.0 if key == "0" else int(key) / 10
                        message = f"속도 → {speed * 100:.0f}%"
                    elif key in ("+", "=", "-", "_"):
                        delta = speed_step if key in ("+", "=") else -speed_step
                        speed = round(max(speed_step, min(1.0, speed + delta)), 2)
                        message = f"속도 → {speed * 100:.0f}%"
                    elif key_lower in ("r", "f"):
                        if any(drive):
                            message = "정지 상태에서만 바퀴 크기 변경 가능"
                        else:
                            delta = 0.1 if key_lower == "r" else -0.1
                            size = max(0.0, min(1.0, wheel_sizes[0] + delta))
                            wheel_sizes = [size] * 6
                            motor.set_wheel_sizes(wheel_sizes)
                            message = f"바퀴 크기 → {size:.1f}"

                show_status(message)

            now = time.monotonic()
            if any(drive) and now >= drive_deadline:
                drive = [0, 0]
                last_drive_key = None
                motor.emergency_stop()
                show_status("데드맨 타임아웃 → 자동 정지")

            v_left = drive[0] * speed * max_velocity
            v_right = drive[1] * speed * max_velocity
            motor.set_wheel_velocities([v_left, v_right] * 3)  # FL, FR, ML, MR, RL, RR
            time.sleep(loop_interval)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            motor.emergency_stop()
        finally:
            try:
                motor.shutdown()
            finally:
                try:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                except termios.error:
                    pass
        print("\n[Terminal] 모터 정지 및 GPIO 해제 완료")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true',
                        help='실제 GPIO 없이 시뮬레이션 (config/sim.yaml)')
    parser.add_argument('--no-record', action='store_true',
                        help='USB 카메라 영상/소리 녹화 끄기')
    parser.add_argument('--config', default=None,
                        help='config YAML 경로 (기본: config/real.yaml, --sim이면 config/sim.yaml)')
    # 이전 실행 명령 호환용 (항상 터미널 제어)
    parser.add_argument('--terminal', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()

    config_path = args.config or ('config/sim.yaml' if args.sim else 'config/real.yaml')
    config = load_config(ROOT / config_path)
    print(f"[Main] Config 로드 완료 (mode={config.mode})")

    recorder = None
    if not args.no_record:
        try:
            from camera.recorder import create_recorder
            recorder = create_recorder(config, ROOT)
        except Exception as e:
            print(f"[Main] 녹화 기능 비활성화: {e}")
    if recorder is not None:
        recorder.start()

    try:
        terminal_control(config)
    finally:
        if recorder is not None:
            recorder.stop()


if __name__ == "__main__":
    main()
