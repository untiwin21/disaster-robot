# LEARNING NOTES

코드 분석할 때 참고할 노트. 각 모듈의 핵심 개념, 수식, 분석 가이드.

---

## 전체 구조

```
disaster-robot/
├── config/                   # YAML 파라미터 (sim.yaml 시뮬용, real.yaml 실제용)
├── shared/
│   └── config.py            # YAML → dataclass 로더 + 검증
└── raspberry-pi/            # 로봇 (라파)
    ├── main.py              # 진입점 (SSH 터미널 제어)
    ├── camera/              # 카메라 HAL
    ├── motor/               # 모터 HAL
    ├── sensor/              # 센서 HAL
    ├── kinematics/          # 조향 알고리즘
    ├── dynamics/            # 동역학
    ├── control/             # 제어기
    └── estimation/          # 상태 추정
```

---

## Phase별 분석 가이드

### Phase 1: Config 시스템 (`shared/config.py`)

**핵심 개념**
- 모든 물리 파라미터를 YAML 외부 파일로
- dataclass로 type-safe하게 로드
- 모드별 검증 (real 모드에선 0.0 placeholder 발견 시 에러)

**볼 것**
- `Config` 클래스 트리: 어떻게 nested dataclass로 구조화?
- `_validate_real_robot()`: 어떤 검증 규칙?

---

### Phase 2: Kinematics (`raspberry-pi/kinematics/`)

#### Ackermann (`ackermann.py`)

**핵심 수식**
```
R = L / tan(δ)                      # 회전 반경 (뒷차축 기준)
δ_inner = atan(L / (R - W/2))       # 안쪽 바퀴 (더 많이 꺾음)
δ_outer = atan(L / (R + W/2))       # 바깥쪽 바퀴
v_wheel = ω · r_wheel                # 회전중심 거리 비례
```

**왜 이래야 하나**
- 모든 바퀴가 미끄러짐 없이 굴러가려면 모든 회전축의 연장선이 한 점(ICR)에서 만나야 함
- 안쪽/바깥쪽 바퀴가 그리는 원의 반지름이 다름 → 다른 각도

#### Skid Steer (`skid_steer.py`)

**핵심 수식**
```
v_left  = v - ω · W/2
v_right = v + ω · W/2
```

좌우 속도 차이만으로 회전. 스티어링 메커니즘 없음. 험지/실내 강력.

#### Crab (`crab.py`)

모든 (조향 가능) 바퀴를 같은 각도로 평행 조향. 차체 yaw 안 변하고 사선 평행 이동.

#### Double Ackermann (`double_ackermann.py`)

앞뒤 바퀴 반대 방향 조향. 회전 중심이 차체 중앙 → 회전 반경 절반.

**시각화 결과** (`docs/kinematics_comparison.png`)
- 같은 입력에 4가지 모드의 궤적 차이 한눈에

---

### Phase 3: Dynamics (`raspberry-pi/dynamics/`)

#### Pacejka Tire Model (`tire_model.py`)

**Magic Formula**
```
F = D · sin(C · arctan(B·α - E·(B·α - arctan(B·α))))
```
- α: slip angle [rad]
- B, C, D, E: 4개 계수
- 한 줄 공식이 타이어의 비선형 그립을 정확히 표현

**그래프 분석** (`docs/tire_curve.png`)
- 작은 슬립: 선형 증가 (잘 그립)
- 적정 슬립: peak (최대 그립)
- 큰 슬립: 감소 (미끄러짐)

#### Bicycle Dynamics (`bicycle_model.py`)

**3개 미분 방정식 (Newton-Euler)**
```
m·(dv_x/dt - v_y·r) = ΣF_x      # 종방향
m·(dv_y/dt + v_x·r) = ΣF_y      # 횡방향 (코너링)
I_z·(dr/dt)         = ΣM_z      # yaw 회전
```

**RK4 적분**
오일러 대신 4차 Runge-Kutta. 같은 dt에 정확도 ~10000배.

**시각화 결과** (`docs/bicycle_dynamics.png`, `kinematic_vs_dynamic.png`)
- 속도별 yaw rate, sideslip, slip angle 응답
- Kinematic vs Dynamic 차이 (Dynamic이 코너에서 안쪽으로)

---

### Phase 4: Control (`raspberry-pi/control/`)

#### PID (`pid.py`)

```
output = Kp·e + Ki·∫e dt + Kd·de/dt
```

**구현 디테일**
- Anti-windup: `_integral` 클램프 → 발산 방지
- Derivative on measurement: setpoint 갑자기 바꿀 때 출력 안 튐
- Output saturation: 액추에이터 한계

**그래프 분석** (`docs/pid_response.png`)
- P only: steady-state error 남음
- +I: 천천히 정확히 도달
- +D: 안정성 향상
- 너무 크면: 진동/오버슈트

#### Pure Pursuit (`pure_pursuit.py`)

**원리**: 운전할 때 "멀리 있는 한 점을 보고 거기로 향한다"

```
δ = arctan(2·L·y_t / L_d²)
```
- L: 휠베이스
- L_d: lookahead distance
- y_t: 차체 좌표계에서 목표점 횡방향 거리

**lookahead 트레이드오프**
- 짧으면: 정확하지만 진동
- 길면: 부드럽지만 코너에서 컷 인

#### Stanley (`stanley.py`)

**원리**: 앞축이 가장 가까운 경로점에 닿도록

```
δ = ψ_e + arctan(k · e / v)
```
- ψ_e: heading error (차체 yaw vs 경로 yaw)
- e: cross-track error (앞축 ~ 경로 횡방향 거리)
- v: 속도 (1/v라 빠를수록 보정 약함)

Pure Pursuit과 차이: 기준점이 앞축, 미래 점 안 봄.

**그래프 분석** (`docs/path_tracking.png`)
- Pure Pursuit: 코너에서 컷 인 (안쪽으로)
- Stanley: 더 정확

---

### Phase 5: Estimation (`raspberry-pi/estimation/`)

#### Odometry (`odometry.py`)

**핵심 수식 (differential drive)**
```
d  = (d_left + d_right) / 2          # 평균 거리
Δψ = (d_right - d_left) / W           # yaw 변화
x  ← x + d · cos(ψ + Δψ/2)           # 중간점 적분 (Runge)
y  ← y + d · sin(ψ + Δψ/2)
ψ  ← ψ + Δψ
```

**한계**
- 누적 오차 (drift)
- 미끄러짐 못 잡음
- 평지 가정

#### Kalman Filter (`kalman.py`)

**2단계 (Predict + Update)**
```
# Predict
x̂ = F·x + B·u
P = F·P·Fᵀ + Q

# Update
y = z - H·x̂                  (innovation)
S = H·P·Hᵀ + R                (innovation covariance)
K = P·Hᵀ·S⁻¹                  (Kalman gain)
x̂ = x̂ + K·y
P = (I - K·H)·P
```

**Kalman gain의 의미**: "측정을 얼마나 신뢰할까?" 자동 계산
- R 작음 (측정 신뢰): K 큼 → 측정 따름
- R 큼 (측정 의심): K 작음 → 모델 따름

#### EKF (`ekf.py`)

비선형 시스템용 KF. 매 스텝 자코비안 계산해서 선형 근사.

**로봇 모델 (RobotPoseEKF)**
```
상태: [x, y, ψ]
입력: [v, ω]
모델:
  x_{k+1} = x_k + v·cos(ψ)·dt
  y_{k+1} = y_k + v·sin(ψ)·dt
  ψ_{k+1} = ψ_k + ω·dt

자코비안 F:
  [1  0  -v·sin(ψ)·dt]
  [0  1   v·cos(ψ)·dt]
  [0  0       1      ]
```

**그래프 분석** (`docs/estimation.png`, `estimation_error.png`)
- Odometry: 부드럽지만 drift 누적
- EKF: 측정 노이즈로 출렁이지만 평균은 안 흐름
- Q와 R의 비율로 트레이드오프 튜닝

---

### Phase 6: 시스템

#### HAL Pattern (`raspberry-pi/motor/`, `sensor/`, `camera/`)

같은 인터페이스로 시뮬/실제 둘 다.
```python
motor = create_motor_hal(config)  # 자동 선택
motor.set_wheel_velocities([...])  # 같은 메서드
```

하드웨어 도착 후 `gpio_motor.py` 채우면 다른 코드 안 건드려도 됨.

---

## 시각화 결과 모음 (`docs/`)

| 파일 | 내용 |
|---|---|
| `kinematics_comparison.png` | 4가지 조향 모드 궤적 비교 |
| `tire_curve.png` | Pacejka 곡선 |
| `bicycle_dynamics.png` | 속도별 동역학 응답 |
| `kinematic_vs_dynamic.png` | 두 모델 비교 |
| `pid_response.png` | PID 튜닝 비교 |
| `path_tracking.png` | Pure Pursuit vs Stanley |
| `estimation.png` | Odometry vs EKF |
| `estimation_error.png` | 오차 시계열 |

각 시각화는 해당 모듈의 `visualize.py` 실행으로 재생성 가능.

---

## TODO (하드웨어 도착 후)

1. `config/real.yaml`의 0.0 값들 채우기 (기계팀 설계)
2. `raspberry-pi/motor/gpio_motor.py` 구현
   - PCA9685 또는 직접 GPIO PWM
   - 캘리브레이션 (PWM ↔ 속도/각도)
3. `raspberry-pi/sensor/ultrasonic.py`의 `GpioUltrasonicHAL` 구현
4. 변형 바퀴와 odometry 통합 (반경 가변 처리)

---

## 학습 우선순위 (분석 순서 추천)

1. **Config + Kinematics** (Ackermann 먼저) - 직관적
2. **Pacejka Tire** - 한 공식 이해
3. **Bicycle Dynamics** - Newton-Euler 적용
4. **PID** - 가장 단순한 제어
5. **Pure Pursuit** - 직관적 path tracking
6. **Stanley** - PP와 비교
7. **Odometry** - 단순
8. **KF → EKF** - 가장 어려움. 천천히
9. **시스템 (HAL)** - 코드 양 많지만 패턴 단순
