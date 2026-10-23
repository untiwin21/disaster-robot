# Disaster Rescue Robot

재난 구조 로봇 제어 시스템

로봇은 라즈베리파이에 SSH로 접속한 터미널에서만 조작한다. 네트워크 원격 제어나 GUI는 없다.
SSH로 접속할 수 있는 네트워크라면 Wi-Fi, 유선 LAN 등 연결 방식은 상관없다.

```
[SSH 터미널 키 입력] ──▶ main.py ──▶ Motor HAL (BTS7960 좌/우)
```

## 디렉토리

```
disaster-robot/
├── main.py                   # 엔트리 포인트 (SSH 터미널 제어)
├── config/                   # YAML 파라미터 (하드웨어 스펙, 제어 게인 등)
├── camera/                   # 영상 캡처
├── sensor/                   # 초음파 등
├── motor/                    # HAL (시뮬/실제)
├── kinematics/               # Ackermann, Skid, Crab, Double Ackermann
├── dynamics/                 # Bicycle model, Pacejka tire
├── control/                  # PID, Pure pursuit, Stanley
├── estimation/               # Odometry, Kalman filter
├── tools/                    # 하드웨어 점검/캘리브레이션 스크립트
├── shared/                   # Config 로더
├── docs/                     # 시각화 결과
└── WIRING.md                 # 결선 정보
```

## 실행 환경 (라즈베리파이)

- 프로젝트: `~/disaster-robot`
- Python 환경: `~/dr-env`
- 프로젝트 내부 `venv/`는 사용하지 않는다.

## 실행

### Windows CMD에서 바로 실행

현재 라즈베리파이 주소가 `10.92.223.146`일 때:

```cmd
ssh -t pi@10.92.223.146 "cd ~/disaster-robot && source ~/dr-env/bin/activate && exec python main.py"
```

IP가 바뀌면 `pi@` 뒤의 주소만 변경한다.

### 이미 SSH로 접속한 경우

```bash
cd ~/disaster-robot
source ~/dr-env/bin/activate
python main.py
```

옵션 없이 실행하면 실제 하드웨어 모드(`config/real.yaml`)로 동작하며, 실행 즉시 GPIO와
모터 드라이버를 초기화한다. 실행 전에 주변이 비어 있고 로봇이 갑자기 움직여도 위험하지
않은 상태인지 먼저 확인한다.

### 하드웨어 없이 테스트

```bash
python main.py --sim
```

`--sim`을 붙이면 `config/sim.yaml`을 사용하며 실제 GPIO를 사용하지 않는다.
다른 설정 파일은 `--config <경로>`로 지정할 수 있다.

## USB 카메라 녹화

`main.py`를 실행하면 USB 카메라 연결을 감시하다가, 카메라가 연결되면 영상과 소리(카메라
마이크)를 자동으로 녹화한다. 카메라를 뽑으면 녹화를 멈추고, 다시 꽂으면 새 파일로 녹화한다.

- 저장 위치: `~/disaster-robot/recordings/YYYYmmdd_HHMMSS.mp4` (H.264 + AAC)
- 10분(`segment_seconds`)마다 새 파일로 나눠 저장한다.
- 라즈베리파이 하드웨어 H.264 인코더를 우선 사용하고, 없으면 CPU(libx264)로 인코딩한다.
- 녹화 중 전원이 끊기거나 카메라가 빠져도 그 직전까지는 재생된다 (fragmented MP4).
- 남은 저장 공간이 500MB(`min_free_mb`) 미만이면 녹화를 멈춘다.
- 녹화를 끄려면 `--no-record`를 붙이거나 config의 `recording.enabled`를 `false`로 바꾼다.
- ffmpeg 에러 로그는 `recordings/ffmpeg.log`에 남는다.

처음 한 번 ffmpeg를 설치해야 한다.

```bash
sudo apt install ffmpeg
```

녹화 파일을 노트북으로 가져오려면 노트북에서:

```bash
scp "pi@10.92.223.146:~/disaster-robot/recordings/*.mp4" .
```

## 조작키

| 키 | 동작 |
|---|---|
| `W` / `S` | 왼쪽 모터 전진 / 후진 (누르는 동안) |
| `↑` / `↓` | 오른쪽 모터 전진 / 후진 (누르는 동안) |
| `1`~`9` / `0` | 속도 10~90% / 100% (기본 50%). 주행 중에도 바로 반영 |
| `+` / `-` | 속도 10%씩 증가 / 감소 (최소 10%) |
| `R` / `F` | 정지 상태에서 전체 바퀴 크기 증가 / 감소 |
| `Space` | 즉시 정지 |
| `Q` | 모터 정지, GPIO 해제 후 종료 |
| `Ctrl+C` | 안전 종료 |

조향 모드는 없다. 조향 서보는 중앙에 고정되고, 좌/우 모터 속도 차이로 회전한다(탱크식).
양쪽을 같이 누르면 직진/후진, 한쪽만 누르면 그쪽 반대 방향으로 회전, 서로 반대로 누르면 제자리 회전.

주행 키(`W`/`S`/`↑`/`↓`) 입력이 0.5초 이상 없으면 데드맨 타이머가 DC 모터를 자동 정지한다.
키를 처음 누른 직후에는 키보드 반복 입력이 시작되기까지 지연이 있으므로 1초까지 기다린다.

터미널은 키를 뗀 것을 알 수 없고 마지막에 누른 키만 반복 입력하므로, 데드맨은 좌/우 공용이다.
예를 들어 `W`와 `↑`를 같이 누르고 있다가 `W`만 떼면 `↑` 입력이 계속 들어오는 동안 왼쪽 모터도
계속 돈다. 한쪽만 멈추려면 키를 모두 뗐다가 다시 누르거나 `Space`를 누른다.
SSH 입력이 종료되거나 프로그램에서 예외가 발생해도 종료 처리에서 모터 정지와
GPIO 해제를 시도한다.

정상 종료 메시지는 다음과 같다.

```text
[Terminal] 모터 정지 및 GPIO 해제 완료
```

## 문제 해결

### `터미널 제어는 TTY가 필요합니다`

SSH 접속 시 `-t` 옵션을 사용한다.

```cmd
ssh -t pi@10.92.223.146
```

### 실제 모터 초기화 실패

반드시 `~/dr-env`을 활성화했는지 확인한다.

```bash
source ~/dr-env/bin/activate
python -c "import cv2, lgpio, board, adafruit_pca9685; print('OK')"
/usr/sbin/i2cdetect -y 1
```

I2C 표에 PCA9685 주소 `40`이 보여야 한다.

### 녹화가 시작되지 않음

```bash
ls /dev/v4l/by-id/          # usb-...-video-index0 이 보여야 함
arecord -l                  # 카메라 마이크가 capture 장치로 보여야 함
tail -n 30 ~/disaster-robot/recordings/ffmpeg.log
```

### 라즈베리파이 IP 확인

라즈베리파이 터미널에서:

```bash
hostname -I
```

표시된 LAN 주소를 SSH 명령의 `pi@<IP>`에 사용한다.

### Real 모드에서 ConfigValidationError

`config/real.yaml`의 0.0 placeholder를 실제 값으로 채워야 한다. 에러 메시지에 어느 필드가
비었는지 표시된다.

## 시각화 (이론 모듈 결과 확인)

```bash
python kinematics/visualize.py   # 조향 모드 비교
python dynamics/visualize.py     # Pacejka, Bicycle
python control/visualize.py      # PID, Path tracking
python estimation/visualize.py   # Odometry vs EKF
```

결과는 `docs/` 폴더에 PNG 파일로 저장된다.
