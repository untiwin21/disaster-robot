"""
Dynamics Visualization

1. Pacejka tire 곡선 (slip angle vs lateral force)
2. Bicycle 모델 시뮬: 일정 조향각 → 시간에 따른 궤적/슬립각/yaw rate
3. Kinematic vs Dynamic 비교 (저속/고속에서 차이)
"""

import sys
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt
import numpy as np

from shared.config import load_config
from dynamics import PacejkaTireModel, BicycleDynamics, VehicleState, VehicleInputs


def plot_tire_curve(ax, tire_model: PacejkaTireModel):
    """Pacejka 곡선 그리기"""
    slip_angles_deg = np.linspace(-15, 15, 200)
    slip_angles_rad = np.radians(slip_angles_deg)
    
    forces = [tire_model.lateral_force(a) for a in slip_angles_rad]
    
    ax.plot(slip_angles_deg, forces, 'b-', linewidth=2)
    ax.axhline(0, color='gray', linewidth=0.5)
    ax.axvline(0, color='gray', linewidth=0.5)
    
    # peak 지점 표시
    peak_idx = np.argmax(np.abs(forces))
    ax.plot(slip_angles_deg[peak_idx], forces[peak_idx], 'ro', markersize=8)
    ax.annotate(f'Peak\n({slip_angles_deg[peak_idx]:.1f}°, {forces[peak_idx]:.1f}N)',
                xy=(slip_angles_deg[peak_idx], forces[peak_idx]),
                xytext=(8, 10), textcoords='offset points',
                fontsize=9, color='red')
    
    ax.set_xlabel("Slip angle [deg]")
    ax.set_ylabel("Lateral force [N]")
    ax.set_title(f"Pacejka Tire Model (B={tire_model.B}, C={tire_model.C}, "
                 f"D={tire_model.D:.1f}, E={tire_model.E})",
                 fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3)


def simulate_bicycle(dynamics: BicycleDynamics,
                     initial_v_x: float,
                     steer_angle: float,
                     duration: float,
                     dt: float = 0.01):
    """일정 조향각으로 일정 속도 진입한 후 자유 진행."""
    state = VehicleState(v_x=initial_v_x)
    inputs = VehicleInputs(steer_angle=steer_angle, F_drive=0.0)
    
    times = [0.0]
    history = [state]
    
    steps = int(duration / dt)
    for i in range(steps):
        state = dynamics.step(state, inputs, dt)
        times.append((i + 1) * dt)
        history.append(state)
    
    return np.array(times), history


def main():
    config = load_config(ROOT / "config" / "sim.yaml")
    
    # ─────────────────────────────────────────
    # Figure 1: Tire curve
    # ─────────────────────────────────────────
    tire = PacejkaTireModel(config.tire, normal_load=8.0)  # 5kg/6 ≈ 8N per wheel
    
    fig1, ax1 = plt.subplots(figsize=(8, 5))
    plot_tire_curve(ax1, tire)
    plt.tight_layout()
    plt.savefig(ROOT / "docs" / "tire_curve.png", dpi=120)
    plt.close()
    print(f"[OK] Tire curve saved")
    
    # ─────────────────────────────────────────
    # Figure 2: Bicycle simulation
    # 시나리오: 0.3 m/s로 진입, steer=0.1 rad(약 5.7도) 일정 유지
    # ─────────────────────────────────────────
    dynamics = BicycleDynamics(config.robot, tire)
    
    fig2, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig2.suptitle("Bicycle Dynamics: steer=0.1 rad, initial v_x=0.3 m/s",
                  fontsize=13, fontweight='bold')
    
    # 여러 속도에서 비교
    speeds = [0.2, 0.4, 0.5]
    colors = ['tab:blue', 'tab:green', 'tab:red']
    duration = 5.0
    steer_input = 0.15  # rad
    
    for v0, color in zip(speeds, colors):
        times, hist = simulate_bicycle(dynamics, v0, steer_input, duration)
        xs = [s.x for s in hist]
        ys = [s.y for s in hist]
        v_ys = [s.v_y for s in hist]
        rs = [s.r for s in hist]
        
        # Slip angles 계산
        slips = []
        for s in hist:
            v_x_safe = s.v_x if abs(s.v_x) > 0.1 else 0.1
            alpha_f = steer_input - math.atan2(s.v_y + dynamics.L_f * s.r, v_x_safe)
            slips.append(math.degrees(alpha_f))
        
        # 1) Trajectory
        axes[0, 0].plot(xs, ys, color=color, label=f"v₀={v0} m/s", linewidth=2)
        axes[0, 0].plot(xs[0], ys[0], 'o', color=color, markersize=8)
        axes[0, 0].plot(xs[-1], ys[-1], 's', color=color, markersize=8)
        
        # 2) Yaw rate
        axes[0, 1].plot(times, rs, color=color, label=f"v₀={v0} m/s", linewidth=2)
        
        # 3) Lateral velocity
        axes[1, 0].plot(times, v_ys, color=color, label=f"v₀={v0} m/s", linewidth=2)
        
        # 4) Front slip angle
        axes[1, 1].plot(times, slips, color=color, label=f"v₀={v0} m/s", linewidth=2)
    
    axes[0, 0].set_xlabel("x [m]"); axes[0, 0].set_ylabel("y [m]")
    axes[0, 0].set_title("Trajectory"); axes[0, 0].legend()
    axes[0, 0].set_aspect('equal'); axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].set_xlabel("time [s]"); axes[0, 1].set_ylabel("yaw rate [rad/s]")
    axes[0, 1].set_title("Yaw rate"); axes[0, 1].legend(); axes[0, 1].grid(True, alpha=0.3)
    
    axes[1, 0].set_xlabel("time [s]"); axes[1, 0].set_ylabel("v_y [m/s]")
    axes[1, 0].set_title("Lateral velocity (sideslip)"); axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].set_xlabel("time [s]"); axes[1, 1].set_ylabel("front slip angle [deg]")
    axes[1, 1].set_title("Front wheel slip angle"); axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(ROOT / "docs" / "bicycle_dynamics.png", dpi=120)
    plt.close()
    print(f"[OK] Bicycle dynamics saved")
    
    # ─────────────────────────────────────────
    # Figure 3: Kinematic vs Dynamic 비교
    # ─────────────────────────────────────────
    from kinematics import KinematicsManager, FL, FR, ML, MR, RL, RR
    
    fig3, ax3 = plt.subplots(figsize=(10, 7))
    
    # Kinematic Ackermann (slip 무시)
    mgr = KinematicsManager(config.robot, initial_mode="Ackermann")
    
    def simulate_kinematic(throttle, steer, duration, dt=0.01):
        """단순 kinematic bicycle 적분"""
        x, y, psi = 0.0, 0.0, 0.0
        L = config.robot.wheelbase
        v_max = config.robot.motion.max_velocity
        max_steer = config.robot.steering.max_angle
        
        v = throttle * v_max
        delta = steer * max_steer
        
        history = [(x, y)]
        steps = int(duration / dt)
        for _ in range(steps):
            if abs(delta) > 1e-6:
                omega = v * math.tan(delta) / L
            else:
                omega = 0.0
            x += v * math.cos(psi) * dt
            y += v * math.sin(psi) * dt
            psi += omega * dt
            history.append((x, y))
        return np.array(history)
    
    test_speeds = [0.2, 0.5, 0.8]
    steer_normalized = 0.5  # 정규화 입력
    delta_rad = steer_normalized * config.robot.steering.max_angle
    
    for v0, color in zip(test_speeds, ['tab:cyan', 'tab:orange', 'tab:purple']):
        # Kinematic
        throttle_norm = v0 / config.robot.motion.max_velocity
        traj_k = simulate_kinematic(throttle_norm, steer_normalized, 4.0)
        ax3.plot(traj_k[:, 0], traj_k[:, 1], '--', color=color,
                 label=f"Kinematic v₀={v0}", linewidth=1.5, alpha=0.7)
        
        # Dynamic - 정상상태 유지를 위해 F_drive 약간 줘서 속도 유지
        state = VehicleState(v_x=v0)
        # 마찰/항력 무시한 모델이라 F_drive=0으로 시작 후 감속 일어남
        # 비교 의도상 단순화해서 같은 초기조건만
        inputs = VehicleInputs(steer_angle=delta_rad, F_drive=0.0)
        traj_d_x, traj_d_y = [0.0], [0.0]
        for _ in range(400):
            state = dynamics.step(state, inputs, 0.01)
            traj_d_x.append(state.x)
            traj_d_y.append(state.y)
        ax3.plot(traj_d_x, traj_d_y, '-', color=color,
                 label=f"Dynamic v₀={v0}", linewidth=2)
    
    ax3.set_xlabel("x [m]"); ax3.set_ylabel("y [m]")
    ax3.set_title("Kinematic (dashed) vs Dynamic (solid) Bicycle Model\n"
                  f"Same input: steer={steer_normalized}, varying speed",
                  fontsize=11, fontweight='bold')
    ax3.legend(); ax3.grid(True, alpha=0.3); ax3.set_aspect('equal')
    
    plt.tight_layout()
    plt.savefig(ROOT / "docs" / "kinematic_vs_dynamic.png", dpi=120)
    plt.close()
    print(f"[OK] Kinematic vs Dynamic saved")


if __name__ == "__main__":
    main()
