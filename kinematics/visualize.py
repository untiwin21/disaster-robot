"""
Kinematics Visualization

4가지 조향 모드의 동작을 시각화.
- 각 모드별로 동일한 (throttle, steer) 입력에 대해
  바퀴들의 속도/조향각을 그림으로 표현
- 10초 동안 시뮬레이션한 차체 궤적 비교
"""

import sys
import math
from pathlib import Path

# 프로젝트 루트 경로 추가
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt
import numpy as np

from shared.config import load_config
from kinematics import (
    KinematicsManager, KinematicsCommand,
    FL, FR, ML, MR, RL, RR, WHEEL_NAMES,
)


def draw_robot(ax, cmd: KinematicsCommand, robot, title: str):
    """로봇 탑뷰: 바퀴 위치 + 조향각 + 속도 화살표"""
    L = robot.wheelbase
    W = robot.track
    offset = robot.middle_axle_offset
    
    # 바퀴 위치 (차체 중심 = 앞축과 뒷축 중점)
    # 앞축은 +L/2, 뒷축은 -L/2 위치
    positions = {
        FL: (+L/2, +W/2),
        FR: (+L/2, -W/2),
        ML: (offset, +W/2),
        MR: (offset, -W/2),
        RL: (-L/2, +W/2),
        RR: (-L/2, -W/2),
    }
    
    # 차체 외곽 (사각형)
    body_x = [+L/2 + 0.05, +L/2 + 0.05, -L/2 - 0.05, -L/2 - 0.05, +L/2 + 0.05]
    body_y = [+W/2 + 0.03, -W/2 - 0.03, -W/2 - 0.03, +W/2 + 0.03, +W/2 + 0.03]
    ax.plot(body_y, body_x, 'k-', linewidth=1.5, alpha=0.7)
    
    # 전방 표시 (삼각형)
    ax.plot([0], [+L/2 + 0.05], 'k^', markersize=10)
    
    # 각 바퀴
    wheel_len = 0.10
    max_v = robot.motion.max_velocity
    
    for idx, (x, y) in positions.items():
        wc = cmd[idx]
        delta = wc.steer_angle
        v = wc.velocity
        
        # 바퀴 그리기 (직사각형, 조향각만큼 회전)
        # 바퀴 본체 방향 = +x (전방), 조향 시 그만큼 yaw
        dx = wheel_len * math.cos(delta)
        dy = wheel_len * math.sin(delta)
        # 좌표계 매핑: matplotlib의 x ← 로봇 y (좌측이 +), y ← 로봇 x (전방이 +)
        ax.plot([y - dy/2, y + dy/2], [x - dx/2, x + dx/2],
                'b-', linewidth=4, solid_capstyle='round')
        
        # 속도 화살표 (바퀴 진행 방향)
        if abs(v) > 1e-3:
            arrow_scale = 0.10 * (v / max_v)  # max_v면 화살표 길이 0.1m
            ax.arrow(y, x, -arrow_scale * math.sin(delta), arrow_scale * math.cos(delta),
                     head_width=0.015, head_length=0.015,
                     fc='red' if v > 0 else 'orange', ec='red' if v > 0 else 'orange',
                     length_includes_head=True)
        
        # 라벨
        ax.text(y, x + 0.03, f"{WHEEL_NAMES[idx]}\nv={v:+.2f}",
                fontsize=7, ha='center', color='gray')
    
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("y (left+) [m]")
    ax.set_ylabel("x (front+) [m]")
    ax.set_title(title, fontsize=11, fontweight='bold')
    ax.set_xlim(-W, W)
    ax.set_ylim(-L, L)


def simulate_trajectory(kinematics, throttle, steer, duration=10.0, dt=0.02):
    """
    바퀴 명령으로부터 차체 궤적을 적분.
    
    가장 단순한 적분: 차체 중앙 속도와 yaw rate를 추출해 차체 운동 갱신.
    - Ackermann/Double: ICR 알고 있으니 (v_center, omega) 명확
    - Skid: (v_left+v_right)/2 = v_center, (v_right-v_left)/W = omega  
    - Crab: 모든 바퀴 평행, 차체 yaw 안 변함, 속도 벡터 (v cosδ, v sinδ)
    
    여기선 일반화 위해 좌측/우측 그룹 평균 + 조향각 평균 사용.
    """
    # 차체 상태
    x, y, theta = 0.0, 0.0, 0.0
    history = [(x, y, theta)]
    
    steps = int(duration / dt)
    for _ in range(steps):
        cmd = kinematics.compute(throttle, steer)
        
        # 차체 운동 추출 (단순화: 좌우 평균 + 조향각 평균)
        v_left  = (cmd[FL].velocity + cmd[ML].velocity + cmd[RL].velocity) / 3
        v_right = (cmd[FR].velocity + cmd[MR].velocity + cmd[RR].velocity) / 3
        v_center = (v_left + v_right) / 2
        
        # 조향각 평균 (앞바퀴 기준)
        front_steer = (cmd[FL].steer_angle + cmd[FR].steer_angle) / 2
        rear_steer  = (cmd[RL].steer_angle + cmd[RR].steer_angle) / 2
        
        # yaw rate 계산: 모드별로 다름
        mode_name = kinematics.current_name
        if mode_name == "SkidSteer":
            # ω = (v_R - v_L) / W
            W = kinematics.current.W
            omega = (v_right - v_left) / W
            vx_body = v_center
            vy_body = 0.0
        elif mode_name == "Crab":
            # 차체 yaw 변화 없음, 평행 이동
            omega = 0.0
            vx_body = v_center * math.cos(front_steer)
            vy_body = v_center * math.sin(front_steer)
        else:
            # Ackermann / Double Ackermann: bicycle 모델 근사
            L = kinematics.current.L
            if mode_name == "DoubleAckermann":
                # 회전중심이 차체 중앙: ω = v / (L/2 / tan(δ))
                if abs(front_steer) > 1e-6:
                    omega = v_center * math.tan(front_steer) / (L / 2)
                else:
                    omega = 0.0
            else:
                # 일반 Ackermann: bicycle model
                if abs(front_steer) > 1e-6:
                    omega = v_center * math.tan(front_steer) / L
                else:
                    omega = 0.0
            vx_body = v_center
            vy_body = 0.0
        
        # 차체 좌표계 → 월드 좌표계 (theta는 차체 yaw)
        vx_world = vx_body * math.cos(theta) - vy_body * math.sin(theta)
        vy_world = vx_body * math.sin(theta) + vy_body * math.cos(theta)
        
        x += vx_world * dt
        y += vy_world * dt
        theta += omega * dt
        
        history.append((x, y, theta))
    
    return np.array(history)


def main():
    config = load_config(ROOT / "config" / "sim.yaml")
    robot = config.robot
    
    # 동일 입력으로 4가지 모드 비교
    THROTTLE = 0.5
    STEER = 1.0   # 최대 조향으로 차이 명확하게
    
    fig = plt.figure(figsize=(16, 9))
    fig.suptitle(f"Kinematics Comparison (throttle={THROTTLE}, steer={STEER})",
                 fontsize=14, fontweight='bold')
    
    mode_names = ["Ackermann", "SkidSteer", "Crab", "DoubleAckermann"]
    
    # 상단: 각 모드 로봇 그림 (4개)
    for i, mode in enumerate(mode_names):
        ax = fig.add_subplot(2, 4, i + 1)
        mgr = KinematicsManager(robot, initial_mode=mode)
        cmd = mgr.compute(THROTTLE, STEER)
        draw_robot(ax, cmd, robot, mode)
    
    # 하단: 궤적 비교 (전체 통합 1개 + 개별 3개)
    ax_traj = fig.add_subplot(2, 4, (5, 8))
    
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']
    for mode, color in zip(mode_names, colors):
        mgr = KinematicsManager(robot, initial_mode=mode)
        traj = simulate_trajectory(mgr, THROTTLE, STEER, duration=8.0)
        ax_traj.plot(traj[:, 0], traj[:, 1], color=color, linewidth=2, label=mode)
        # 시작점/끝점
        ax_traj.plot(traj[0, 0], traj[0, 1], 'o', color=color, markersize=8)
        ax_traj.plot(traj[-1, 0], traj[-1, 1], 's', color=color, markersize=8)
        # 화살표로 차체 방향 표시 (끝점에서)
        end_x, end_y, end_theta = traj[-1]
        ax_traj.arrow(end_x, end_y,
                      0.15 * math.cos(end_theta), 0.15 * math.sin(end_theta),
                      head_width=0.05, fc=color, ec=color, alpha=0.7)
    
    ax_traj.set_aspect('equal')
    ax_traj.grid(True, alpha=0.3)
    ax_traj.set_xlabel("x [m] (forward)")
    ax_traj.set_ylabel("y [m] (left)")
    ax_traj.set_title("Trajectory (8s integration) - ○ start, □ end",
                      fontsize=11, fontweight='bold')
    ax_traj.legend(loc='best')
    
    plt.tight_layout()
    out = ROOT / "docs" / "kinematics_comparison.png"
    out.parent.mkdir(exist_ok=True)
    plt.savefig(out, dpi=120, bbox_inches='tight')
    print(f"[OK] 시각화 저장: {out}")
    plt.close()


if __name__ == "__main__":
    main()
