"""
Estimation Visualization

3가지 시나리오:
1. Odometry drift: 노이즈 있는 엔코더로 위치 추정 → drift 누적
2. EKF: odometry(예측) + 노이즈 있는 GPS 같은 측정(갱신) → 깔끔한 추정
3. KF gain 효과: R 작/크 비교
"""

import sys
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt
import numpy as np

from shared.config import load_config
from estimation import (
    DifferentialOdometry, WheelEncoderData,
    RobotPoseEKF,
)


def make_circular_trajectory(radius=1.0, omega=0.3, duration=20.0, dt=0.05):
    """원형 궤적 생성 (참값)"""
    times = np.arange(0, duration, dt)
    xs, ys, psis = [], [], []
    
    # ω·R = v 가 되는 v로 원 그리기
    v = radius * omega
    x, y, psi = 0.0, 0.0, 0.0
    for _ in times:
        xs.append(x); ys.append(y); psis.append(psi)
        x += v * math.cos(psi) * dt
        y += v * math.sin(psi) * dt
        psi += omega * dt
    
    return times, np.array(xs), np.array(ys), np.array(psis), v


def odometry_with_noise(true_v_left, true_v_right, dt, noise_std=0.05):
    """엔코더 측정 시뮬: 진짜 속도에 노이즈 추가"""
    v_left_noisy = true_v_left + np.random.randn() * noise_std
    v_right_noisy = true_v_right + np.random.randn() * noise_std
    return WheelEncoderData(
        d_left=v_left_noisy * dt,
        d_right=v_right_noisy * dt,
    )


def main():
    np.random.seed(42)
    config = load_config(ROOT / "config" / "sim.yaml")
    
    # ────────────────────────────────────
    # 시나리오 1: Odometry drift
    # ────────────────────────────────────
    times, true_x, true_y, true_psi, v_center = make_circular_trajectory(
        radius=1.0, omega=0.3, duration=30.0
    )
    dt = times[1] - times[0]
    track = config.robot.track
    
    # 진짜 좌우 속도 (원 운동 가정)
    omega = 0.3
    v_left = v_center - omega * track / 2
    v_right = v_center + omega * track / 2
    
    # Odometry 시뮬
    odom = DifferentialOdometry(track_width=track)
    odom_xs, odom_ys = [0.0], [0.0]
    
    for _ in range(1, len(times)):
        encoder = odometry_with_noise(v_left, v_right, dt, noise_std=0.03)
        state = odom.update(encoder)
        odom_xs.append(state.x)
        odom_ys.append(state.y)
    
    # ────────────────────────────────────
    # 시나리오 2: EKF (odometry + noisy GPS)
    # ────────────────────────────────────
    ekf = RobotPoseEKF(
        dt=dt,
        process_noise_std=(0.05, 0.05, 0.02),
        measurement_noise_std=(0.15, 0.15, 0.10),
    )
    
    ekf_xs, ekf_ys = [0.0], [0.0]
    gps_xs, gps_ys = [], []  # 측정값 시각화용
    
    gps_noise_std = 0.15
    gps_update_every = 5  # 매 5스텝마다 GPS 측정 사용 (낮은 갱신율 시뮬)
    
    for k in range(1, len(times)):
        # Predict (control input: 진짜 v, omega - 실제론 명령값)
        # 명령값 자체가 정확하지 않다고 가정해서 Q에 노이즈 반영
        ekf.predict(v=v_center, omega=omega)
        
        # 가끔만 측정 갱신
        if k % gps_update_every == 0:
            x_meas = true_x[k] + np.random.randn() * gps_noise_std
            y_meas = true_y[k] + np.random.randn() * gps_noise_std
            psi_meas = true_psi[k] + np.random.randn() * 0.1
            ekf.update(x_meas, y_meas, psi_meas)
            gps_xs.append(x_meas); gps_ys.append(y_meas)
        
        s = ekf.state
        ekf_xs.append(s[0])
        ekf_ys.append(s[1])
    
    # ────────────────────────────────────
    # Plot
    # ────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(15, 7))
    
    # Left: Odometry drift
    ax = axes[0]
    ax.plot(true_x, true_y, 'k-', linewidth=2.5, label='Ground truth', alpha=0.8)
    ax.plot(odom_xs, odom_ys, 'tab:red', linewidth=1.5, label='Odometry (with noise)')
    ax.plot(0, 0, 'go', markersize=10, label='start')
    ax.plot(true_x[-1], true_y[-1], 'k^', markersize=10, label='true end')
    ax.plot(odom_xs[-1], odom_ys[-1], 'r^', markersize=10, label='odometry end')
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Odometry Drift\n(encoder noise accumulates over time)",
                 fontsize=12, fontweight='bold')
    ax.legend(); ax.grid(True, alpha=0.3); ax.set_aspect('equal')
    
    # Right: EKF
    ax = axes[1]
    ax.plot(true_x, true_y, 'k-', linewidth=2.5, label='Ground truth', alpha=0.8)
    ax.plot(odom_xs, odom_ys, 'tab:red', linewidth=1, alpha=0.4, label='Odometry only')
    ax.scatter(gps_xs, gps_ys, color='tab:orange', s=15, alpha=0.5,
               label='Noisy measurements')
    ax.plot(ekf_xs, ekf_ys, 'tab:blue', linewidth=2, label='EKF estimate')
    ax.plot(0, 0, 'go', markersize=10, label='start')
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Extended Kalman Filter\n(prediction + noisy measurement = optimal estimate)",
                 fontsize=12, fontweight='bold')
    ax.legend(); ax.grid(True, alpha=0.3); ax.set_aspect('equal')
    
    fig.suptitle("State Estimation: Odometry vs EKF",
                 fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(ROOT / "docs" / "estimation.png", dpi=120)
    plt.close()
    print("[OK] Estimation visualization saved")
    
    # ────────────────────────────────────
    # 추가: 오차 시계열
    # ────────────────────────────────────
    odom_xs_arr = np.array(odom_xs)
    odom_ys_arr = np.array(odom_ys)
    ekf_xs_arr = np.array(ekf_xs)
    ekf_ys_arr = np.array(ekf_ys)
    
    odom_err = np.sqrt((true_x - odom_xs_arr)**2 + (true_y - odom_ys_arr)**2)
    ekf_err = np.sqrt((true_x - ekf_xs_arr)**2 + (true_y - ekf_ys_arr)**2)
    
    fig2, ax2 = plt.subplots(figsize=(11, 5))
    ax2.plot(times, odom_err, 'tab:red', linewidth=2, label='Odometry error')
    ax2.plot(times, ekf_err, 'tab:blue', linewidth=2, label='EKF error')
    ax2.set_xlabel("time [s]"); ax2.set_ylabel("position error [m]")
    ax2.set_title("Position Error Over Time",
                  fontsize=12, fontweight='bold')
    ax2.legend(); ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(ROOT / "docs" / "estimation_error.png", dpi=120)
    plt.close()
    print("[OK] Error plot saved")


if __name__ == "__main__":
    main()
