"""
USB 카메라 영상/소리 녹화

USB 카메라가 연결되면 자동으로 녹화를 시작하고, 분리되면 멈춘다.
다시 연결하면 새 파일로 이어서 녹화한다.

[흐름]
    watch 스레드 (poll_interval마다)
      ├─ /dev/v4l/by-id/usb-*-video-index0 로 USB 카메라 탐색
      ├─ 같은 USB 장치의 ALSA 마이크 탐색 (없으면 다른 USB 마이크, 그것도 없으면 영상만)
      └─ ffmpeg 프로세스 실행 → recordings/YYYYmmdd_HHMMSS.mkv

[인코딩]
- 1순위: 카메라 MJPEG 스트림을 재인코딩 없이 저장 (CPU 거의 안 씀)
- MJPEG 미지원 카메라: raw 입력을 libx264 ultrafast로 인코딩
- 소리: AAC
- 컨테이너는 MKV: 전원 차단/카메라 분리로 중간에 끊겨도 앞부분은 재생 가능
- segment_seconds마다 파일을 나눠서 손상 범위를 줄인다

[필요]
    sudo apt install ffmpeg
"""

import glob
import os
import shutil
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional


# 실행 직후 이 시간 안에 ffmpeg가 죽으면 입력 형식 문제로 보고 다음 프로필 시도
_EARLY_EXIT_SEC = 3.0
# 모든 방식이 실패하면 이 시간 뒤에 처음부터 다시 시도
_RETRY_DELAY_SEC = 30.0


@dataclass
class _Profile:
    name: str
    input_format: Optional[str]   # v4l2 입력 형식 (None이면 카메라 기본값)
    video_codec: List[str]


_PROFILES = [
    _Profile("mjpeg-copy", "mjpeg", ["-c:v", "copy"]),
    _Profile("raw-h264", None, ["-c:v", "libx264", "-preset", "ultrafast", "-pix_fmt", "yuv420p"]),
]


def _log(msg: str) -> None:
    # 터미널 제어 상태줄(\r)과 섞이지 않도록 줄을 바꿔서 출력
    print(f"\n[Recorder] {msg}", flush=True)


def _usb_device_path(sys_device: str) -> Optional[str]:
    """sysfs의 USB 인터페이스 경로 → 상위 USB 장치 경로 (같은 장치 판별용)"""
    try:
        return os.path.dirname(os.path.realpath(sys_device))
    except OSError:
        return None


def find_usb_camera() -> Optional[str]:
    """USB 카메라의 영상 캡처 노드 (/dev/v4l/by-id/...-video-index0). 없으면 None."""
    links = sorted(glob.glob("/dev/v4l/by-id/usb-*-video-index0"))
    return links[0] if links else None


def find_usb_microphone(video_link: str) -> Optional[str]:
    """카메라와 같은 USB 장치의 마이크를 ALSA 장치 이름으로 반환. 없으면 다른 USB 마이크."""
    video_node = os.path.basename(os.path.realpath(video_link))
    camera_usb = _usb_device_path(f"/sys/class/video4linux/{video_node}/device")

    same_device, other_usb = None, None
    for card_dir in sorted(glob.glob("/proc/asound/card[0-9]*")):
        card = os.path.basename(card_dir)
        capture_pcms = sorted(glob.glob(f"{card_dir}/pcm*c"))
        if not capture_pcms or not os.path.exists(f"{card_dir}/usbid"):
            continue
        try:
            card_id = Path(f"{card_dir}/id").read_text().strip()
        except OSError:
            continue
        pcm_num = os.path.basename(capture_pcms[0])[3:-1]
        # plughw: 카메라 마이크가 지원하는 샘플레이트/채널로 자동 변환
        alsa_name = f"plughw:CARD={card_id},DEV={pcm_num}"

        if camera_usb and _usb_device_path(f"/sys/class/sound/{card}/device") == camera_usb:
            same_device = alsa_name
            break
        if other_usb is None:
            other_usb = alsa_name

    return same_device or other_usb


class UsbAvRecorder:
    """USB 카메라 연결을 감시하면서 영상/소리를 파일로 녹화"""

    def __init__(self, output_dir: Path, width: int, height: int, fps: int,
                 segment_seconds: int = 600, record_audio: bool = True,
                 min_free_mb: int = 500, poll_interval: float = 2.0):
        self.output_dir = Path(output_dir)
        self.width = width
        self.height = height
        self.fps = fps
        self.segment_seconds = segment_seconds
        self.record_audio = record_audio
        self.min_free_mb = min_free_mb
        self.poll_interval = poll_interval

        self._proc: Optional[subprocess.Popen] = None
        self._proc_started = 0.0
        self._proc_log = None
        self._device: Optional[str] = None
        self._profile_idx = 0
        self._retry_at = 0.0
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ─── 공개 API ───

    def start(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(target=self._watch_loop, name="usb-recorder", daemon=True)
        self._thread.start()
        _log(f"USB 카메라 감시 시작 (저장 위치: {self.output_dir})")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=10)
        self._stop_ffmpeg()

    # ─── 감시 루프 ───

    def _watch_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._tick()
            except Exception as e:
                _log(f"감시 중 에러: {e}")
            self._stop_event.wait(self.poll_interval)

    def _tick(self) -> None:
        device = find_usb_camera()

        if self._proc is not None and self._proc.poll() is not None:
            self._on_ffmpeg_exit(device)

        if device is None:
            if self._proc is not None:
                _log("USB 카메라 분리됨 → 녹화 중지")
                self._stop_ffmpeg()
            self._device = None
            self._profile_idx = 0
            self._retry_at = 0.0
            return

        if not self._has_free_space():
            if self._proc is not None:
                _log(f"저장 공간 부족 (<{self.min_free_mb}MB) → 녹화 중지")
                self._stop_ffmpeg()
            return

        if self._proc is None and time.monotonic() >= self._retry_at:
            if device != self._device:
                self._device = device
                self._profile_idx = 0
            self._start_ffmpeg(device)

    def _on_ffmpeg_exit(self, device: Optional[str]) -> None:
        code = self._proc.returncode
        ran_for = time.monotonic() - self._proc_started
        self._close_log()
        self._proc = None
        if device is None:
            return  # 분리되면서 종료된 경우
        if ran_for >= _EARLY_EXIT_SEC:
            _log(f"ffmpeg 종료됨 (code {code}) → 다시 시작")
        elif self._profile_idx + 1 < len(_PROFILES):
            self._profile_idx += 1
            _log(f"ffmpeg 시작 실패 (code {code}) → '{_PROFILES[self._profile_idx].name}' 방식으로 재시도")
        else:
            self._profile_idx = 0
            self._retry_at = time.monotonic() + _RETRY_DELAY_SEC
            _log(f"녹화 시작 실패. {_RETRY_DELAY_SEC:.0f}초 후 재시도. 로그: {self.output_dir / 'ffmpeg.log'}")

    def _has_free_space(self) -> bool:
        free_mb = shutil.disk_usage(self.output_dir).free / (1024 * 1024)
        return free_mb >= self.min_free_mb

    # ─── ffmpeg ───

    def _build_command(self, device: str, mic: Optional[str]) -> List[str]:
        profile = _PROFILES[self._profile_idx]
        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "warning", "-nostdin"]

        cmd += ["-f", "v4l2", "-thread_queue_size", "512"]
        if profile.input_format:
            cmd += ["-input_format", profile.input_format]
        cmd += ["-video_size", f"{self.width}x{self.height}",
                "-framerate", str(self.fps), "-i", device]

        if mic:
            cmd += ["-f", "alsa", "-thread_queue_size", "1024", "-i", mic]

        cmd += ["-map", "0:v"]
        if mic:
            cmd += ["-map", "1:a", "-c:a", "aac", "-b:a", "128k"]
        cmd += profile.video_codec

        pattern = str(self.output_dir / "%Y%m%d_%H%M%S.mkv")
        cmd += ["-f", "segment", "-segment_time", str(self.segment_seconds),
                "-segment_format", "matroska", "-reset_timestamps", "1",
                "-strftime", "1", pattern]
        return cmd

    def _start_ffmpeg(self, device: str) -> None:
        mic = find_usb_microphone(device) if self.record_audio else None
        cmd = self._build_command(device, mic)

        self._proc_log = open(self.output_dir / "ffmpeg.log", "a", encoding="utf-8")
        self._proc_log.write(f"\n=== {time.strftime('%Y-%m-%d %H:%M:%S')} {' '.join(cmd)}\n")
        self._proc_log.flush()

        # 별도 세션: 터미널 Ctrl+C가 ffmpeg에 직접 가지 않고 stop()에서 정리
        self._proc = subprocess.Popen(cmd, stdout=self._proc_log, stderr=self._proc_log,
                                      stdin=subprocess.DEVNULL, start_new_session=True)
        self._proc_started = time.monotonic()
        audio = f"소리: {mic}" if mic else "소리 없음 (USB 마이크 미검출)"
        _log(f"녹화 시작: {os.path.basename(device)} ({_PROFILES[self._profile_idx].name}, {audio})")

    def _stop_ffmpeg(self) -> None:
        proc = self._proc
        if proc is None:
            return
        if proc.poll() is None:
            # SIGINT를 받으면 ffmpeg가 현재 파일을 정상 마무리하고 종료
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            _log("녹화 파일 저장 완료")
        self._proc = None
        self._close_log()

    def _close_log(self) -> None:
        if self._proc_log is not None:
            self._proc_log.close()
            self._proc_log = None


def create_recorder(config, root: Path) -> Optional[UsbAvRecorder]:
    """config.recording 기반 녹화기 생성. 비활성/ffmpeg 없음/리눅스 아님이면 None."""
    rec = config.recording
    if not rec.enabled:
        return None
    if not os.path.exists("/sys/class/video4linux"):
        return None  # 리눅스 V4L2 환경이 아님 (개발용 맥/윈도우 등)
    if shutil.which("ffmpeg") is None:
        _log("ffmpeg가 없어 녹화를 사용할 수 없습니다. 설치: sudo apt install ffmpeg")
        return None

    output_dir = Path(rec.output_dir)
    if not output_dir.is_absolute():
        output_dir = root / output_dir
    cam = config.camera
    return UsbAvRecorder(
        output_dir=output_dir,
        width=cam.width, height=cam.height, fps=cam.fps,
        segment_seconds=rec.segment_seconds,
        record_audio=rec.audio,
        min_free_mb=rec.min_free_mb,
    )
