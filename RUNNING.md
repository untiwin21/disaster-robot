# RUNNING

## 1. 환경 설정 (conda)

```bash
# 환경 생성
conda env create -f environment.yml

# 활성화
conda activate disaster-robot
```

## 2. 시뮬레이션 실행 (하드웨어 없이)

TTY가 있는 터미널에서:

```bash
python raspberry-pi/main.py --sim
```

## 3. 시각화 (이론 모듈 결과 확인)

```bash
# Kinematics 4가지 모드 비교
python raspberry-pi/kinematics/visualize.py

# Dynamics (Pacejka, Bicycle)
python raspberry-pi/dynamics/visualize.py

# Control (PID, Path tracking)
python raspberry-pi/control/visualize.py

# Estimation (Odometry vs EKF)
python raspberry-pi/estimation/visualize.py
```

결과는 `docs/` 폴더에 PNG 파일로 저장.

## 4. 실제 하드웨어 실행

`config/real.yaml`의 TBD 값들을 채우고, 라즈베리파이에 SSH로 접속해 실행한다.

```bash
python raspberry-pi/main.py
```

접속 방법, 조작키, 문제 해결은 [`README.md`](README.md)를 참고한다.

## 트러블슈팅

**Real 모드에서 ConfigValidationError**
- `config/real.yaml`의 0.0 placeholder를 실제 값으로 채워야 함
- 에러 메시지에 어느 필드가 비었는지 명시됨
