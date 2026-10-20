"""
Camera HAL

실제 USB 웹캠 / 시뮬용 가짜 영상.
시뮬은 움직이는 패턴 (테스트용 합성 영상) 생성.
"""

from abc import ABC, abstractmethod
from typing import Optional
import time
import math
import numpy as np
import cv2


class CameraHAL(ABC):
    @abstractmethod
    def read(self) -> Optional[np.ndarray]:
        """BGR 이미지. 실패 시 None."""
        pass
    
    @abstractmethod
    def release(self) -> None:
        pass


class OpenCVCamera(CameraHAL):
    """실제 USB 웹캠 (OpenCV VideoCapture)"""
    
    def __init__(self, device_index: int = 0,
                 width: int = 640, height: int = 480, fps: int = 30):
        self.cap = cv2.VideoCapture(device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라 {device_index} 열기 실패")
    
    def read(self) -> Optional[np.ndarray]:
        ret, frame = self.cap.read()
        return frame if ret else None
    
    def release(self) -> None:
        self.cap.release()


class SyntheticCamera(CameraHAL):
    """
    가짜 영상 생성기. 시뮬용.
    
    - 그라데이션 배경
    - 움직이는 도형 (사람 흉내용 직사각형)
    - 타임스탬프 텍스트
    
    YOLO가 진짜 사람으로 인식하진 않을 텐데, 영상 파이프라인 테스트엔 충분.
    """
    
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self.width = width
        self.height = height
        self.fps = fps
        self._start = time.monotonic()
        self._frame_count = 0
        self._last_read = 0.0
    
    def read(self) -> Optional[np.ndarray]:
        # FPS 제한 (실제 카메라 흉내)
        now = time.monotonic()
        target_dt = 1.0 / self.fps
        if now - self._last_read < target_dt:
            time.sleep(max(0, target_dt - (now - self._last_read)))
        self._last_read = time.monotonic()
        
        elapsed = self._last_read - self._start
        frame = self._generate_frame(elapsed)
        self._frame_count += 1
        return frame
    
    def _generate_frame(self, t: float) -> np.ndarray:
        h, w = self.height, self.width
        
        # 그라데이션 배경 (회색 → 진한 회색)
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        for y in range(h):
            shade = int(40 + (y / h) * 80)
            frame[y, :] = (shade, shade, shade + 10)
        
        # 움직이는 직사각형 (가상 "사람")
        cx = int(w / 2 + (w / 3) * math.sin(t * 0.5))
        cy = int(h / 2)
        rect_w, rect_h = 60, 140
        x1, y1 = cx - rect_w//2, cy - rect_h//2
        x2, y2 = cx + rect_w//2, cy + rect_h//2
        cv2.rectangle(frame, (x1, y1), (x2, y2), (180, 150, 100), -1)
        # 머리
        cv2.circle(frame, (cx, y1 - 25), 25, (200, 170, 130), -1)
        
        # 타임스탬프
        cv2.putText(frame, f"SIM {t:.1f}s f={self._frame_count}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2)
        cv2.putText(frame, "Synthetic Camera",
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200), 1)
        
        # 격자 (배경 움직임 인지용)
        for x in range(0, w, 80):
            cv2.line(frame, (x, 0), (x, h), (60, 60, 70), 1)
        for y in range(0, h, 80):
            cv2.line(frame, (0, y), (w, y), (60, 60, 70), 1)
        
        return frame
    
    def release(self) -> None:
        pass


def create_camera(config) -> CameraHAL:
    """
    Config 기반 카메라 생성.
    
    시뮬/실제 모드 무관하게 실제 카메라 먼저 시도.
    카메라 없으면 자동으로 합성 영상으로 fallback.
    
    이렇게 하면:
    - 노트북에 웹캠 연결돼 있으면: 진짜 영상 사용 (YOLO 실전 테스트 가능)
    - 웹캠 없으면: 합성 영상 (코드 동작 검증)
    - 라파에 카메라 없을 때도 자동 fallback
    """
    cam = config.camera
    try:
        camera = OpenCVCamera(
            device_index=cam.device_index,
            width=cam.width, height=cam.height, fps=cam.fps,
        )
        print(f"[Camera] 실제 카메라 사용 (device {cam.device_index}, {cam.width}x{cam.height})")
        return camera
    except RuntimeError as e:
        print(f"[Camera] 실제 카메라 사용 불가 ({e})")
        print(f"[Camera] → 합성 영상으로 fallback")
        return SyntheticCamera(width=cam.width, height=cam.height, fps=cam.fps)
