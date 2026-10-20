"""
Raspberry Pi Main

SSH 터미널에서 로봇을 직접 조작한다. 네트워크/GUI 원격 제어는 없다.

    [SSH 키 입력] ──▶ [Kinematics] ──▶ [Motor HAL]

[안전]
- W/S 입력이 끊기면 데드맨 타이머가 모터를 정지
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
from kinematics import KinematicsManager


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
    kinematics_mgr = KinematicsManager(config.robot)

    drive_dir = 0       # 1: 전진, -1: 후진, 0: 정지
    speed = 0.5         # 속도 단계 [0.1, 1.0] (throttle = drive_dir × speed)
    speed_step = 0.1
    steer = 0.0
    wheel_sizes = [0.0] * 6
    drive_deadline = 0.0
    deadman_timeout = 0.5
    # 키를 누른 직후에는 OS 키 반복 지연(보통 0.5초 이상)과 SSH 지연이 겹치므로
    # 첫 입력에만 여유를 더 준다.
    initial_deadman_timeout = 1.0
    loop_interval = 1.0 / max(1, config.control.loop_rate)
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def show_status(message: str = "") -> None:
        status = (
            f"mode={kinematics_mgr.current_name:<17} "
            f"speed={speed * 100:3.0f}% throttle={drive_dir * speed:+.1f} steer={steer:+.1f} "
            f"wheel_size={wheel_sizes[0]:.1f}"
        )
        if message:
            status += f" | {message}"
        print(f"\r{status:<110}", end="", flush=True)

    print("[Terminal] W/S: 전진/후진(누르는 동안)  A/D: 조향  C: 조향 중앙")
    print("[Terminal] 1~9/0: 속도 10~90/100%  +/-: 속도 ±10%")
    print("[Terminal] R/F: 바퀴 크기 +/-  M: 조향 모드  Space: 정지  Q: 종료")
    print(
        f"[Terminal] W/S 입력이 {deadman_timeout:.1f}초 끊기면 자동 정지합니다. "
        f"(첫 입력 후 {initial_deadman_timeout:.1f}초)"
    )

    try:
        tty.setcbreak(fd)
        show_status("준비 완료")

        while True:
            now = time.monotonic()
            readable, _, _ = select.select([sys.stdin], [], [], 0)
            message = ""

            if readable:
                key = os.read(fd, 1).decode(errors="ignore")
                if key == "":
                    message = "SSH 입력 종료"
                    break

                key_lower = key.lower()
                if key_lower == "q" or key == "\x03":
                    message = "종료"
                    break
                elif key == " ":
                    drive_dir = 0
                    steer = 0.0
                    drive_deadline = 0.0
                    motor.emergency_stop()
                    message = "EMERGENCY STOP"
                elif key_lower in ("w", "s"):
                    new_dir = 1 if key_lower == "w" else -1
                    timeout = deadman_timeout if drive_dir == new_dir else initial_deadman_timeout
                    drive_dir = new_dir
                    drive_deadline = now + timeout
                elif key.isdigit():
                    speed = 1.0 if key == "0" else int(key) / 10
                    message = f"속도 → {speed * 100:.0f}%"
                elif key in ("+", "=", "-", "_"):
                    delta = speed_step if key in ("+", "=") else -speed_step
                    speed = round(max(speed_step, min(1.0, speed + delta)), 2)
                    message = f"속도 → {speed * 100:.0f}%"
                elif key_lower == "a":
                    steer = min(1.0, steer + 0.2)
                elif key_lower == "d":
                    steer = max(-1.0, steer - 0.2)
                elif key_lower == "c":
                    steer = 0.0
                elif key_lower == "m":
                    drive_dir = 0
                    drive_deadline = 0.0
                    motor.emergency_stop()
                    message = f"조향 모드 → {kinematics_mgr.cycle_next()}"
                elif key_lower in ("r", "f"):
                    if drive_dir != 0:
                        message = "정지 상태에서만 바퀴 크기 변경 가능"
                    else:
                        delta = 0.1 if key_lower == "r" else -0.1
                        size = max(0.0, min(1.0, wheel_sizes[0] + delta))
                        wheel_sizes = [size] * 6
                        motor.set_wheel_sizes(wheel_sizes)
                        message = f"바퀴 크기 → {size:.1f}"

                show_status(message)

            now = time.monotonic()
            if drive_dir != 0 and now >= drive_deadline:
                drive_dir = 0
                motor.emergency_stop()
                show_status("데드맨 타임아웃 → 자동 정지")

            command = kinematics_mgr.compute(drive_dir * speed, steer)
            motor.set_wheel_velocities([wheel.velocity for wheel in command.wheels])
            motor.set_steer_angles([wheel.steer_angle for wheel in command.wheels])
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
