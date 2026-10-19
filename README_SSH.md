# SSH 터미널 제어

라즈베리파이에 SSH로 접속해 노트북 GUI 없이 로봇을 직접 조작하는 방법이다.
SSH로 접속할 수 있는 네트워크라면 Wi-Fi, 유선 LAN 등 연결 방식은 상관없다.

## 실행 환경

- 프로젝트: `~/disaster-robot`
- Python 환경: `~/dr-env`
- 프로젝트 내부 `venv/`는 사용하지 않는다.

## Windows CMD에서 바로 실행

현재 라즈베리파이 주소가 `10.92.223.146`일 때:

```cmd
ssh -t pi@10.92.223.146 "cd ~/disaster-robot && source ~/dr-env/bin/activate && exec python raspberry-pi/main.py --config config/real.yaml --terminal"
```

IP가 바뀌면 `pi@` 뒤의 주소만 변경한다.

## 이미 SSH로 접속한 경우

```bash
cd ~/disaster-robot
source ~/dr-env/bin/activate
python raspberry-pi/main.py --config config/real.yaml --terminal
```

실제 하드웨어 모드에서는 안전 확인 후 `RUN`을 입력해야 GPIO와 모터 드라이버를
초기화한다. 주변이 비어 있고 로봇이 갑자기 움직여도 위험하지 않은 상태인지 먼저
확인한다.

## 조작키

| 키 | 동작 |
|---|---|
| `W` / `S` | 전진 / 후진. 누르는 동안 반복 입력되며 0.5초 끊기면 자동 정지 |
| `A` / `D` | 조향값을 왼쪽 / 오른쪽으로 단계 조절 |
| `C` | 조향 중앙 |
| `R` / `F` | 정지 상태에서 전체 바퀴 크기 증가 / 감소 |
| `M` | 정지 후 조향 모드 순환 |
| `Space` | 즉시 정지 및 조향 중앙 |
| `Q` | 모터 정지, GPIO 해제 후 종료 |
| `Ctrl+C` | 안전 종료 |

`W` 또는 `S` 입력이 0.5초 이상 없으면 데드맨 타이머가 DC 모터를 자동 정지한다.
SSH 입력이 종료되거나 프로그램에서 예외가 발생해도 종료 처리에서 모터 정지와
GPIO 해제를 시도한다.

## 하드웨어 없이 테스트

```bash
cd ~/disaster-robot
source ~/dr-env/bin/activate
python raspberry-pi/main.py --config config/sim.yaml --terminal
```

시뮬레이션 모드는 `RUN` 확인 없이 시작하며 실제 GPIO를 사용하지 않는다.

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

### 라즈베리파이 IP 확인

라즈베리파이 터미널에서:

```bash
hostname -I
```

표시된 LAN 주소를 SSH 명령의 `pi@<IP>`에 사용한다.

## 종료 후 확인

정상 종료 메시지는 다음과 같다.

```text
[Terminal] 모터 정지 및 GPIO 해제 완료
```
