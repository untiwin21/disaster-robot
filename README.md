# Disaster Rescue Robot

재난 구조 로봇 원격 조종 시스템

## 시스템 구성

```
┌───────────────────────┐                     ┌──────────────────────┐
│   노트북 (조종석)        │                    │  라즈베리파이 (로봇)    │
│                       │  TCP 9998: 제어 →   │                      │
│  - PySide6 GUI       │ ──────────────────▶ │  - 카메라 / 센서       │
│  - YOLOv8 사람 탐지    │                    │  - 모터 컨트롤러       │
│  - 키보드 조종         │ ◀── UDP 9999: 영상 │  - 기구학/제어        │
│  - 레이더/시계열 시각화 │                    │                      │
│                       │ ◀── TCP 9997: 텔레 │                      │
└───────────────────────┘                     └──────────────────────┘
```

## 디렉토리

```
disaster-robot/
├── config/                   # YAML 파라미터 (하드웨어 스펙, 제어 게인 등)
├── laptop/                   # 조종석
│   ├── communication/        # 소켓 송수신
│   ├── detection/            # YOLO
│   ├── gui/                  # PySide6 위젯
│   └── ui/                   # 키보드 입력
├── raspberry-pi/             # 로봇
│   ├── communication/        # 소켓 송수신
│   ├── camera/               # 영상 캡처
│   ├── sensor/               # 초음파 등
│   ├── motor/                # HAL (시뮬/실제)
│   ├── kinematics/           # Ackermann, Skid, Crab, Double Ackermann
│   ├── dynamics/             # Bicycle model, Pacejka tire
│   ├── control/              # PID, Pure pursuit, Stanley
│   ├── estimation/           # Odometry, Kalman filter
│   └── protocol/             # 명령/텔레메트리 메시지 정의
├── shared/                   # 양쪽 공유 (프로토콜, 유틸)
└── docs/                     # 학습 노트, 설계 결정 기록
```

## 실행 (시뮬레이션)

```bash
# 터미널 1: 로봇 시뮬레이터
cd raspberry-pi
python main.py --config ../config/sim.yaml

# 터미널 2: 조종석 GUI
cd laptop
python main.py --config ../config/sim.yaml
```

## 실행 (실제 하드웨어)

```bash
# 라파에서:
cd raspberry-pi
python main.py --config ../config/real.yaml

# 노트북에서:
cd laptop
python main.py --config ../config/real.yaml
```

## SSH 터미널에서 직접 조작

별도 문서 [`README_SSH.md`](README_SSH.md)를 참고한다.

## 조작

| 키 | 동작 |
|---|---|
| W/S | 전진/후진 |
| A/D | 좌/우 조향 |
| Space | 정지 |
| R/F | 앞바퀴 사이즈 +/- |
| T/G | 뒷바퀴 사이즈 +/- |
| M | 조향 모드 전환 (Ackermann/Skid/Crab/Double) |
| H | 단축키 도움말 토글 |
| Q | 종료 |

터미널 모드에서는 `A`/`D`로 조향값을 단계적으로 바꾸고 `C`로 중앙에 맞춘다.
`R`/`F` 바퀴 크기 변경과 `M` 모드 변경은 정지 상태에서 사용한다.
