# RUNNING

## 1. 환경 설정 (conda)

```bash
# 환경 생성
conda env create -f environment.yml

# 활성화
conda activate disaster-robot
```

GPU 쓰려면 (NVIDIA 카드 있을 때): `environment.yml`의 `pytorch` 부분을 다음처럼 수정 후 재생성

```yaml
  - pytorch
  - torchvision
  - pytorch-cuda=12.1   # 추가
```

## 2. 시뮬레이션 실행 (하드웨어 없이)

**같은 컴퓨터 두 터미널**에서:

```bash
# 터미널 1: 노트북 측 GUI 먼저 (서버)
python laptop/main.py --config config/sim.yaml
```

GUI가 떠서 "라즈베리파이 연결 대기..." 상태가 되면

```bash
# 터미널 2: 라파 측 (시뮬레이션)
python raspberry-pi/main.py --config config/sim.yaml
```

GUI가 라파 연결 받으면 영상/레이더/시계열 다 흐르기 시작.

YOLO 없이 빠르게 테스트하려면:
```bash
python laptop/main.py --config config/sim.yaml --no-yolo
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

## 4. 실제 하드웨어 실행 (하드웨어 도착 후)

`config/real.yaml`의 TBD 값들 채우고:

```bash
# 라즈베리파이에서
python raspberry-pi/main.py --config config/real.yaml

# 노트북에서
python laptop/main.py --config config/real.yaml
```

### SSH 터미널 직접 제어 (GUI/Tailscale 불필요)

별도 문서 [`README_SSH.md`](README_SSH.md)를 참고한다.

GUI 네트워크 모드에서만 `real.yaml`의 `network.laptop_ip`와 `rpi_ip`가 필요하다.
`--terminal` 모드에서는 이 두 설정을 사용하지 않는다.

## 조작법

GUI 창에서 영상 위젯 클릭 후:

| 키 | 동작 |
|---|---|
| W/S | 전진/후진 (누르고 있는 동안) |
| A/D | 좌/우 조향 |
| Space | 정지 |
| R/F | 앞바퀴 사이즈 +/- |
| T/G | 뒷바퀴 사이즈 +/- |
| M | 조향 모드 순환 (Ackermann → SkidSteer → Crab → DoubleAckermann) |
| H | 단축키 도움말 토글 |
| Q | 종료 |

## 트러블슈팅

**라파 연결 안 됨**
- 노트북을 먼저 실행해야 함 (서버이므로)
- 방화벽이 9997, 9998, 9999 포트 차단했는지 확인

**키보드 입력 반응 없음**
- GUI 윈도우의 영상 위젯에 포커스가 있어야 함 (한 번 클릭)

**YOLO 로드 느림**
- 첫 실행 시 yolov8n.pt 다운로드 (~6MB)
- `laptop/yolov8n.pt`에 저장됨

**Real 모드에서 ConfigValidationError**
- `config/real.yaml`의 0.0 placeholder를 실제 값으로 채워야 함
- 에러 메시지에 어느 필드가 비었는지 명시됨
