"""
Raspberry Pi Main

전체 흐름:
    [카메라] ──jpeg──▶ [VideoSender] ──UDP──▶ 노트북
    
    노트북 ──TCP──▶ [Command Receiver] ──▶ [Kinematics] ──▶ [Motor HAL]
    
    [Sensors] + [Odometry] ──▶ [Telemetry Sender] ──TCP──▶ 노트북

[스레드 구성]
- main: 명령 수신 + 모터 제어 + 메인 루프
- video_thread: 카메라 → 영상 송신 (별도 스레드, 메인 영향 X)
- telemetry_thread: 센서 + 텔레메트리 송신 (별도 스레드)

[안전]
- 명령 timeout: 일정 시간 명령 없으면 정지
- 영상 송신 실패: 로그 후 무시 (메인 멈추지 않음)
- 통신 끊김 감지 시 return_to_base
"""

import sys
import os
import time
import socket
import threading
import argparse
import math
from pathlib import Path
from collections import deque

# 경로 설정
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "raspberry-pi"))

import cv2
import numpy as np

from shared.config import load_config, Config
from shared.tcp_framing import send_framed, recv_framed
from shared.udp_video import VideoSender
from shared.messages import (
    DriveCommand, WheelSizeCommand, SteeringModeCommand, StopCommand,
    TelemetryMessage, LogMessage, MessageType,
    parse_command, encode_message,
)

from camera import create_camera
from motor import SimMotorHAL, create_motor_hal
from sensor import create_ultrasonic_hal
from kinematics import KinematicsManager
from estimation import DifferentialOdometry, WheelEncoderData


# ════════════════════════════════════════════════════════════════
# 영상 송신 스레드
# ════════════════════════════════════════════════════════════════

def video_loop(camera, video_sender: VideoSender, jpeg_quality: int,
               send_size: tuple, stop_event: threading.Event):
    """카메라 프레임을 JPEG으로 압축해서 UDP 청크로 송신"""
    print("[Video] 영상 송신 스레드 시작")
    sw, sh = send_size
    frame_count = 0
    last_log_time = time.monotonic()
    fps_count = 0
    
    while not stop_event.is_set():
        frame = camera.read()
        if frame is None:
            time.sleep(0.05)
            continue
        
        # 송신용 다운스케일
        if frame.shape[1] != sw or frame.shape[0] != sh:
            frame = cv2.resize(frame, (sw, sh))
        
        # JPEG 인코딩
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        if not ok:
            continue
        
        try:
            video_sender.send(encoded.tobytes())
            fps_count += 1
        except OSError as e:
            print(f"[Video] 송신 실패: {e}")
        
        frame_count += 1
        
        # 1초마다 FPS 로그
        now = time.monotonic()
        if now - last_log_time >= 5.0:
            fps = fps_count / (now - last_log_time)
            print(f"[Video] {fps:.1f} fps, {frame_count} frames sent")
            fps_count = 0
            last_log_time = now
    
    print("[Video] 영상 송신 스레드 종료")


# ════════════════════════════════════════════════════════════════
# 텔레메트리 송신 스레드
# ════════════════════════════════════════════════════════════════

def telemetry_loop(telemetry_conn, ultrasonic_hal, robot_state,
                   ultrasonic_rate: float, stop_event: threading.Event):
    """센서 + 자세 추정 → 텔레메트리 메시지 주기 송신"""
    print("[Telemetry] 텔레메트리 송신 스레드 시작")
    interval = 1.0 / ultrasonic_rate
    
    while not stop_event.is_set():
        loop_start = time.monotonic()
        
        # 센서 읽기
        try:
            us_readings = ultrasonic_hal.read_all()
        except Exception as e:
            print(f"[Telemetry] 센서 읽기 실패: {e}")
            us_readings = []
        
        # 텔레메트리 메시지 구성
        msg = TelemetryMessage(
            timestamp=time.monotonic(),
            pose={
                "x": robot_state['pose'][0],
                "y": robot_state['pose'][1],
                "psi": robot_state['pose'][2],
            },
            velocity={
                "v_x": robot_state['velocity'][0],
                "v_y": robot_state['velocity'][1],
                "yaw_rate": robot_state['velocity'][2],
            },
            steering_mode=robot_state['steering_mode'],
            wheel_size={
                "front": robot_state['wheel_size_front'],
                "middle": robot_state.get('wheel_size_middle', 0.5),
                "rear": robot_state['wheel_size_rear'],
            },
            ultrasonic=[
                {"name": r.name, "distance": r.distance}
                for r in us_readings
            ],
            last_command_age=time.monotonic() - robot_state['last_command_time'],
        )
        
        # 송신
        try:
            send_framed(telemetry_conn, encode_message(msg))
        except (OSError, BrokenPipeError) as e:
            print(f"[Telemetry] 송신 실패 (연결 끊김?): {e}")
            stop_event.set()
            break
        
        # 다음 주기까지 대기
        elapsed = time.monotonic() - loop_start
        sleep_time = max(0, interval - elapsed)
        time.sleep(sleep_time)
    
    print("[Telemetry] 텔레메트리 송신 스레드 종료")


# ════════════════════════════════════════════════════════════════
# 메인
# ════════════════════════════════════════════════════════════════

def terminal_control(config: Config) -> None:
    """SSH TTY에서 네트워크/GUI 없이 로봇을 직접 조작한다."""
    if not sys.stdin.isatty():
        raise RuntimeError("터미널 제어는 TTY가 필요합니다. ssh -t 로 접속하세요.")

    import select
    import termios
    import tty

    if not config.is_simulation:
        print("[Terminal] 실제 모터와 서보를 초기화합니다.")
        print("[Terminal] 로봇을 바닥에서 띄우거나 주변을 비운 뒤 진행하세요.")
        if input("계속하려면 RUN 입력: ").strip() != "RUN":
            print("[Terminal] 취소됨")
            return

    motor = create_motor_hal(config)
    if not config.is_simulation and isinstance(motor, SimMotorHAL):
        motor.shutdown()
        raise RuntimeError("실제 모터 초기화에 실패하여 터미널 제어를 중단합니다.")
    kinematics_mgr = KinematicsManager(config.robot)

    throttle = 0.0
    steer = 0.0
    wheel_sizes = [0.0] * 6
    drive_deadline = 0.0
    deadman_timeout = 0.5
    loop_interval = 1.0 / max(1, config.control.loop_rate)
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def show_status(message: str = "") -> None:
        status = (
            f"mode={kinematics_mgr.current_name:<17} "
            f"throttle={throttle:+.1f} steer={steer:+.1f} "
            f"wheel_size={wheel_sizes[0]:.1f}"
        )
        if message:
            status += f" | {message}"
        print(f"\r{status:<110}", end="", flush=True)

    print("[Terminal] W/S: 전진/후진(누르는 동안)  A/D: 조향  C: 조향 중앙")
    print("[Terminal] R/F: 바퀴 크기 +/-  M: 조향 모드  Space: 정지  Q: 종료")
    print(f"[Terminal] W/S 입력이 {deadman_timeout:.1f}초 끊기면 자동 정지합니다.")

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
                    throttle = 0.0
                    steer = 0.0
                    drive_deadline = 0.0
                    motor.emergency_stop()
                    message = "EMERGENCY STOP"
                elif key_lower == "w":
                    throttle = 1.0
                    drive_deadline = now + deadman_timeout
                elif key_lower == "s":
                    throttle = -1.0
                    drive_deadline = now + deadman_timeout
                elif key_lower == "a":
                    steer = min(1.0, steer + 0.2)
                elif key_lower == "d":
                    steer = max(-1.0, steer - 0.2)
                elif key_lower == "c":
                    steer = 0.0
                elif key_lower == "m":
                    throttle = 0.0
                    drive_deadline = 0.0
                    motor.emergency_stop()
                    message = f"조향 모드 → {kinematics_mgr.cycle_next()}"
                elif key_lower in ("r", "f"):
                    if throttle != 0.0:
                        message = "정지 상태에서만 바퀴 크기 변경 가능"
                    else:
                        delta = 0.1 if key_lower == "r" else -0.1
                        size = max(0.0, min(1.0, wheel_sizes[0] + delta))
                        wheel_sizes = [size] * 6
                        motor.set_wheel_sizes(wheel_sizes)
                        message = f"바퀴 크기 → {size:.1f}"

                show_status(message)

            now = time.monotonic()
            if throttle != 0.0 and now >= drive_deadline:
                throttle = 0.0
                motor.emergency_stop()
                show_status("데드맨 타임아웃 → 자동 정지")

            command = kinematics_mgr.compute(throttle, steer)
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
    parser.add_argument('--config', default='config/sim.yaml',
                        help='config YAML 경로')
    parser.add_argument('--terminal', action='store_true',
                        help='SSH 터미널에서 GUI/네트워크 없이 직접 조작')
    parser.add_argument('--mode', choices=['dev-wired', 'dev-wireless'],
                        default=None,
                        help='네트워크 모드 (기본: 시연용)')
    args = parser.parse_args()
    
    config = load_config(ROOT / args.config)
    
    # --mode 옵션으로 laptop_ip 오버라이드
    if args.mode:
        mode_cfg = config.network.modes.get(args.mode, {})
        if 'laptop_ip' in mode_cfg:
            config.network.laptop_ip = mode_cfg['laptop_ip']
            print(f"[Main] 네트워크 모드: {args.mode} (laptop_ip → {config.network.laptop_ip})")

    if args.terminal:
        terminal_control(config)
        return
    
    print(f"[Main] Config 로드 완료 (mode={config.mode})")
    
    # ─── 모듈 초기화 ───
    camera = create_camera(config)
    motor = create_motor_hal(config)
    ultrasonic = create_ultrasonic_hal(config)
    kinematics_mgr = KinematicsManager(config.robot)
    odometry = DifferentialOdometry(track_width=config.robot.track)
    
    # 통신 - 노트북에 연결 시도
    laptop_ip = config.network.laptop_ip
    
    print(f"[Main] 노트북 ({laptop_ip}) 연결 시도...")
    
    # Retry 로직: 노트북이 먼저 listen해야 하는데 timing 이슈 방지
    def connect_with_retry(port: int, name: str, retries: int = 30, delay: float = 1.0):
        for attempt in range(retries):
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((laptop_ip, port))
                return s
            except (ConnectionRefusedError, OSError) as e:
                if attempt == retries - 1:
                    raise
                if attempt % 5 == 0:
                    print(f"[Main] {name} 연결 재시도 {attempt+1}/{retries}...")
                time.sleep(delay)
        return None
    
    # TCP: 명령 수신 (노트북이 서버, 라파가 클라이언트)
    cmd_sock = connect_with_retry(config.network.command_port, "명령")
    print(f"[Main] 명령 채널 연결 완료 (port {config.network.command_port})")
    
    # TCP: 텔레메트리 송신
    tele_sock = connect_with_retry(config.network.telemetry_port, "텔레메트리")
    print(f"[Main] 텔레메트리 채널 연결 완료 (port {config.network.telemetry_port})")
    
    # UDP: 영상 송신
    video_sender = VideoSender(
        target_host=laptop_ip,
        target_port=config.network.video_port,
        chunk_payload_size=config.network.video_chunk_size,
    )
    
    # ─── 공유 상태 (스레드 간) ───
    robot_state = {
        'pose': [0.0, 0.0, 0.0],          # x, y, psi
        'velocity': [0.0, 0.0, 0.0],      # v_x, v_y, yaw_rate
        'steering_mode': kinematics_mgr.current_name,
        'wheel_size_front': config.robot.wheel.radius_default / config.robot.wheel.radius_max,
        'wheel_size_middle': config.robot.wheel.radius_default / config.robot.wheel.radius_max,
        'wheel_size_rear': config.robot.wheel.radius_default / config.robot.wheel.radius_max,
        'wheel_sizes': [config.robot.wheel.radius_default / config.robot.wheel.radius_max] * 6,  # [FL,FR,ML,MR,RL,RR]
        'last_command_time': time.monotonic(),
    }
    
    # 명령 히스토리 (return_to_base용)
    command_history = deque(maxlen=500)
    
    stop_event = threading.Event()
    
    # ─── 스레드 시작 ───
    video_thread = threading.Thread(
        target=video_loop,
        args=(camera, video_sender, config.camera.jpeg_quality,
              (config.camera.send_width, config.camera.send_height), stop_event),
        daemon=True,
    )
    video_thread.start()
    
    telemetry_thread = threading.Thread(
        target=telemetry_loop,
        args=(tele_sock, ultrasonic, robot_state,
              config.sensors.ultrasonic.update_rate, stop_event),
        daemon=True,
    )
    telemetry_thread.start()
    
    # ─── 메인 루프 (명령 수신 + 모터 제어) ───
    cmd_sock.setblocking(True)
    cmd_sock.settimeout(0.1)  # 100ms 타임아웃 (non-blocking 흉내)
    
    print("[Main] 메인 루프 시작")
    last_command_time = time.monotonic()
    last_drive = DriveCommand()
    
    try:
        while not stop_event.is_set():
            loop_start = time.monotonic()
            
            # ─── 명령 수신 (non-blocking) ───
            try:
                payload = recv_framed(cmd_sock, timeout=0.05)
                if payload is None:
                    print("[Main] 명령 채널 끊김")
                    break
                
                cmd = parse_command(payload)
                last_command_time = time.monotonic()
                robot_state['last_command_time'] = last_command_time
                
                # 명령 처리
                if isinstance(cmd, DriveCommand):
                    last_drive = cmd
                    command_history.append({
                        'time': last_command_time,
                        'cmd': cmd,
                    })
                elif isinstance(cmd, WheelSizeCommand):
                    # sizes가 명시되면 우선 사용 (비대칭 제어), 아니면 그룹 확장
                    if cmd.sizes is not None and len(cmd.sizes) == 6:
                        sizes = list(cmd.sizes)
                    else:
                        # 6개 사이즈로 확장: 앞 2개=front, 중간 2개=middle, 뒤 2개=rear
                        sizes = [cmd.front, cmd.front, cmd.middle, cmd.middle, cmd.rear, cmd.rear]
                    print(f"[Main] WheelSize 수신: {[f'{s:.2f}' for s in sizes]}")
                    motor.set_wheel_sizes(sizes)
                    robot_state['wheel_sizes'] = sizes
                    # 그룹 평균값도 같이 저장 (디스플레이용)
                    robot_state['wheel_size_front'] = (sizes[0] + sizes[1]) / 2
                    robot_state['wheel_size_middle'] = (sizes[2] + sizes[3]) / 2
                    robot_state['wheel_size_rear'] = (sizes[4] + sizes[5]) / 2
                elif isinstance(cmd, SteeringModeCommand):
                    if kinematics_mgr.set_mode(cmd.mode):
                        robot_state['steering_mode'] = cmd.mode
                        print(f"[Main] 조향 모드: {cmd.mode}")
                elif isinstance(cmd, StopCommand):
                    motor.emergency_stop()
                    last_drive = DriveCommand()
            except socket.timeout:
                pass
            except Exception as e:
                print(f"[Main] 명령 처리 에러: {e}")
            
            # ─── 명령 timeout 체크 ───
            now = time.monotonic()
            if now - last_command_time > config.safety.command_timeout:
                print(f"[Main] 명령 timeout ({config.safety.command_timeout}s) → 정지")
                motor.emergency_stop()
                last_drive = DriveCommand()
                # timeout 후엔 이 메시지 한 번만 (다시 명령 오면 풀림)
                last_command_time = now  # 스팸 방지
            
            # ─── Kinematics → 모터 명령 ───
            kin_cmd = kinematics_mgr.compute(last_drive.throttle, last_drive.steer)
            
            velocities = [w.velocity for w in kin_cmd.wheels]
            # 조향각 6개 (모든 바퀴) - 회로도 기준
            steer_angles = [w.steer_angle for w in kin_cmd.wheels]
            
            motor.set_wheel_velocities(velocities)
            motor.set_steer_angles(steer_angles)
            
            # ─── Odometry 업데이트 ───
            # 시뮬에선 명령 속도를 그대로 사용 (실제론 엔코더 사용)
            from kinematics import FL, FR, ML, MR, RL, RR
            # 좌측 평균 (FL, ML, RL), 우측 평균 (FR, MR, RR)
            v_left_avg = (velocities[FL] + velocities[ML] + velocities[RL]) / 3
            v_right_avg = (velocities[FR] + velocities[MR] + velocities[RR]) / 3
            
            dt = 0.02  # 메인 루프 주기 가정
            encoder = WheelEncoderData(d_left=v_left_avg * dt, d_right=v_right_avg * dt)
            odom_state = odometry.update(encoder)
            
            robot_state['pose'][0] = odom_state.x
            robot_state['pose'][1] = odom_state.y
            robot_state['pose'][2] = odom_state.psi
            robot_state['velocity'][0] = (v_left_avg + v_right_avg) / 2  # v_x 근사
            robot_state['velocity'][2] = (v_right_avg - v_left_avg) / config.robot.track  # yaw rate
            
            # 루프 주기 유지
            elapsed = time.monotonic() - loop_start
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n[Main] Ctrl+C 종료")
    finally:
        print("[Main] 종료 처리...")
        stop_event.set()
        motor.emergency_stop()
        motor.shutdown()
        camera.release()
        video_sender.close()
        cmd_sock.close()
        tele_sock.close()
        ultrasonic.shutdown()
        video_thread.join(timeout=2)
        telemetry_thread.join(timeout=2)
        print("[Main] 종료 완료")


if __name__ == "__main__":
    main()
