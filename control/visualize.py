"""
Control Visualization

1. PID 응답: setpoint step 입력 시 measurement 추종
2. PID 튜닝 비교: P only / PI / PID
3. Path tracking: Pure Pursuit vs Stanley 추종 성능
"""

import sys
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt
import numpy as np

from shared.config import load_config, PIDConfig
from control import PIDController, PurePursuitController, StanleyController
from dynamics import PacejkaTireModel, BicycleDynamics, VehicleState, VehicleInputs


# ════════════════════════════════════════════════════════════════
# PID 시각화
# ════════════════════════════════════════════════════════════════

def simulate_pid_step_response(pid: PIDController,
                                setpoint: float = 1.0,
                                duration: float = 5.0,
                                dt: float = 0.02,
                                plant_tau: float = 0.5):
    """
    1차 plant 모델: τ·dy/dt + y = u
    PID가 u를 출력하면 y가 setpoint를 따라가야 함.
    """
    times = np.arange(0, duration, dt)
    measurements = []
    outputs = []
    
    y = 0.0  # 초기 측정값
    for t in times:
        u = pid.update(setpoint, y, dt=dt)
        # 1차 시스템 적분: y_new = y + dt/τ · (u - y)
        y += dt / plant_tau * (u - y)
        measurements.append(y)
        outputs.append(u)
    
    return times, np.array(measurements), np.array(outputs)


def plot_pid_responses(ax_meas, ax_out, config: PIDConfig):
    """다양한 게인 설정 비교"""
    setpoint = 1.0
    
    # 시각화용으로 더 큰 게인 사용 (config 그대로면 응답이 약함)
    base_kp = 3.0
    base_ki = 2.0
    base_kd = 0.3
    
    p_only = PIDConfig(kp=base_kp, ki=0.0, kd=0.0,
                       integral_limit=10.0, output_limit=5.0)
    pi_only = PIDConfig(kp=base_kp, ki=base_ki, kd=0.0,
                        integral_limit=10.0, output_limit=5.0)
    pid_full = PIDConfig(kp=base_kp, ki=base_ki, kd=base_kd,
                         integral_limit=10.0, output_limit=5.0)
    # 진동 비교용: Kp 너무 큰 케이스
    aggressive = PIDConfig(kp=10.0, ki=8.0, kd=0.0,
                           integral_limit=10.0, output_limit=5.0)
    
    configs = [
        ("P only (Kp=3)", p_only, 'tab:red'),
        ("PI (Kp=3, Ki=2)", pi_only, 'tab:orange'),
        ("PID (Kp=3, Ki=2, Kd=0.3)", pid_full, 'tab:blue'),
        ("Aggressive (Kp=10, Ki=8)", aggressive, 'tab:purple'),
    ]
    
    for name, cfg, color in configs:
        controller = PIDController(cfg)
        t, meas, out = simulate_pid_step_response(controller, setpoint=setpoint)
        ax_meas.plot(t, meas, color=color, linewidth=2, label=name)
        ax_out.plot(t, out, color=color, linewidth=2, label=name)
    
    ax_meas.axhline(setpoint, color='gray', linestyle='--', alpha=0.7, label='setpoint')
    ax_meas.set_xlabel("time [s]"); ax_meas.set_ylabel("measurement")
    ax_meas.set_title("Step Response - Various PID Tunings")
    ax_meas.legend(loc='lower right'); ax_meas.grid(True, alpha=0.3)
    
    ax_out.set_xlabel("time [s]"); ax_out.set_ylabel("controller output")
    ax_out.set_title("Control Output")
    ax_out.legend(loc='upper right'); ax_out.grid(True, alpha=0.3)


# ════════════════════════════════════════════════════════════════
# Path Tracking 시각화 (Pure Pursuit vs Stanley)
# ════════════════════════════════════════════════════════════════

def make_test_path():
    """테스트 경로: 직선 + 코너 + 직선 ('S자' 비슷한 모양)"""
    # 직선 진입
    seg1_x = np.linspace(0, 2, 40)
    seg1_y = np.zeros_like(seg1_x)
    
    # 좌회전 코너 (반지름 1.5, 90도)
    R1 = 1.5
    theta1 = np.linspace(-math.pi/2, 0, 30)
    seg2_x = 2 + R1 * np.cos(theta1)
    seg2_y = R1 + R1 * np.sin(theta1)
    
    # 직진 (위쪽)
    seg3_y = np.linspace(R1, R1 + 2, 30)
    seg3_x = np.full_like(seg3_y, 2 + R1)
    
    # 우회전 코너
    theta2 = np.linspace(math.pi, math.pi/2, 30)
    seg4_x = (2 + R1 + R1) + R1 * np.cos(theta2)
    seg4_y = (R1 + 2) + R1 * np.sin(theta2)
    
    # 직진 (위쪽 끝)
    seg5_x = np.linspace(2 + R1 + R1, 2 + R1 + R1 + 2, 30)
    seg5_y = np.full_like(seg5_x, R1 + 2 + R1)
    
    xs = np.concatenate([seg1_x, seg2_x, seg3_x, seg4_x, seg5_x])
    ys = np.concatenate([seg1_y, seg2_y, seg3_y, seg4_y, seg5_y])
    return list(zip(xs.tolist(), ys.tolist()))


def simulate_tracker(tracker_name, controller, dynamics, path,
                     duration=15.0, dt=0.02, target_speed=0.4):
    """
    경로 추종 시뮬레이션.
    tracker_name: "pure_pursuit" or "stanley"
    """
    state = VehicleState(v_x=0.1)  # 작게 시작
    
    history_x, history_y = [], []
    last_target = 0
    
    if tracker_name == "stanley":
        # path에 yaw 추가
        yaws = StanleyController.compute_path_yaws(path)
        path_with_yaw = [(p[0], p[1], y) for p, y in zip(path, yaws)]
    
    steps = int(duration / dt)
    for _ in range(steps):
        pose = (state.x, state.y, state.psi)
        
        # ─── 조향 결정 ───
        if tracker_name == "pure_pursuit":
            result = controller.compute(path, pose, state.v_x,
                                        max_velocity=0.5,
                                        last_target_idx=last_target)
            if result is None:
                break
            steer = result.steer
            last_target = result.target_index
        else:
            result = controller.compute(path_with_yaw, pose, state.v_x)
            if result is None:
                break
            steer = result.steer
        
        # 조향각 saturation (max_steer)
        max_steer = 0.5  # rad
        steer = max(-max_steer, min(max_steer, steer))
        
        # ─── 속도 제어 (간단 P 제어) ───
        speed_error = target_speed - state.v_x
        F_drive = 5.0 * speed_error  # 간단한 P
        F_drive = max(-3.0, min(3.0, F_drive))
        
        # ─── 차량 업데이트 ───
        inputs = VehicleInputs(steer_angle=steer, F_drive=F_drive)
        state = dynamics.step(state, inputs, dt)
        
        history_x.append(state.x)
        history_y.append(state.y)
        
        # 목적지 근처 도달 시 종료
        if math.hypot(state.x - path[-1][0], state.y - path[-1][1]) < 0.2:
            break
    
    return np.array(history_x), np.array(history_y)


# ════════════════════════════════════════════════════════════════
# Main
# ════════════════════════════════════════════════════════════════

def main():
    config = load_config(ROOT / "config" / "sim.yaml")
    
    # ────────────────────────────────────
    # PID 시각화
    # ────────────────────────────────────
    fig1, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig1.suptitle("PID Step Response", fontsize=13, fontweight='bold')
    plot_pid_responses(axes[0], axes[1], config.control.velocity_pid)
    plt.tight_layout()
    plt.savefig(ROOT / "docs" / "pid_response.png", dpi=120)
    plt.close()
    print("[OK] PID response saved")
    
    # ────────────────────────────────────
    # Path tracking 시각화
    # ────────────────────────────────────
    tire = PacejkaTireModel(config.tire, normal_load=8.0)
    dynamics = BicycleDynamics(config.robot, tire)
    
    path = make_test_path()
    path_x = [p[0] for p in path]
    path_y = [p[1] for p in path]
    
    pp_controller = PurePursuitController(config.control.pure_pursuit,
                                          wheelbase=config.robot.wheelbase)
    stanley_controller = StanleyController(config.robot, k=2.0)
    
    pp_x, pp_y = simulate_tracker("pure_pursuit", pp_controller, dynamics, path)
    st_x, st_y = simulate_tracker("stanley", stanley_controller, dynamics, path)
    
    fig2, ax = plt.subplots(figsize=(11, 8))
    ax.plot(path_x, path_y, 'k--', linewidth=1.5, alpha=0.7, label='Reference path')
    ax.plot(pp_x, pp_y, 'tab:blue', linewidth=2, label='Pure Pursuit')
    ax.plot(st_x, st_y, 'tab:red', linewidth=2, label='Stanley')
    ax.plot(0, 0, 'go', markersize=10, label='start')
    ax.plot(path_x[-1], path_y[-1], 'ks', markersize=10, label='goal')
    
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Path Tracking: Pure Pursuit vs Stanley", fontsize=13, fontweight='bold')
    ax.legend(); ax.grid(True, alpha=0.3); ax.set_aspect('equal')
    
    plt.tight_layout()
    plt.savefig(ROOT / "docs" / "path_tracking.png", dpi=120)
    plt.close()
    print("[OK] Path tracking saved")


if __name__ == "__main__":
    main()
