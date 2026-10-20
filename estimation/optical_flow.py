"""
Optical Flow Estimator

OpenCV Farneback dense optical flow로 프레임 간 평균 흐름 추정.
엔코더 슬립을 보정할 독립 측정원으로 활용.

[원리]
- 두 연속 프레임 사이의 pixel별 motion vector 계산
- 평균 vector → 카메라(=로봇) 진행 방향 측정
- 픽셀 단위 → m/s 변환에 캘리브레이션 계수 필요

[전방 카메라 한계]
- 깊이 정보 없으면 변환 부정확
- 가까운 물체와 먼 물체의 픽셀 흐름 다름 (parallax)
- 일단 ROI(중앙 하단)로 한정해서 노이즈 감소
- 정확한 변환은 캘리브레이션 후 (현재는 placeholder scale)

[Farneback 선택 이유]
- Lucas-Kanade는 sparse (특정 코너만 추적) → 특징점 부족하면 실패
- Farneback은 dense → 평균 안정적
- 라파4에서 320x240 해상도로 충분히 동작 가능 (5~10 FPS)

[성능 고려]
- 영상 송신 스레드에서 호출 (이미 캡처된 프레임 재사용)
- 다운샘플링 + ROI로 부하 절감
- 측정값 신뢰도 낮을 때 EKF가 자동으로 무시 (R 큼)
"""

import numpy as np
import time
from typing import Optional, Tuple


class OpticalFlowEstimator:
    """Farneback dense optical flow 기반 속도 추정기."""
    
    def __init__(self,
                 scale: float = 0.001,
                 roi_ratio: Tuple[float, float, float, float] = (0.25, 0.5, 0.75, 1.0),
                 downsample: int = 2,
                 valid_threshold: float = 0.1):
        """
        Args:
            scale: pixel/sec → m/s 변환 계수 (캘리브레이션 후 결정)
                   placeholder 0.001 = 1픽셀/sec → 1mm/s
            roi_ratio: (x_min, y_min, x_max, y_max) 비율 (0~1).
                       기본은 화면 중앙 하단 (전방 카메라용, 노이즈 적은 영역)
            downsample: 입력 프레임 다운샘플 비율 (1=원본, 2=절반)
            valid_threshold: 유효 측정 판단용 흐름 크기 임계값 [pixel/frame]
                             이보다 작으면 노이즈로 간주
        """
        try:
            import cv2
            self._cv2 = cv2
            self._available = True
        except ImportError:
            print("[OpticalFlow] OpenCV 없음")
            self._available = False
            return
        
        self.scale = scale
        self.roi_ratio = roi_ratio
        self.downsample = max(1, int(downsample))
        self.valid_threshold = valid_threshold
        
        # 이전 프레임 (gray)
        self._prev_gray: Optional[np.ndarray] = None
        self._last_time: Optional[float] = None
        
        # 최근 측정값 (디버깅용)
        self.last_flow_pixels = (0.0, 0.0)  # (vx, vy) [pixel/sec]
        self.last_velocity = 0.0            # [m/s]
        self.last_valid = False
    
    @property
    def is_available(self) -> bool:
        return self._available
    
    def update(self, frame_bgr: np.ndarray, timestamp: Optional[float] = None) -> Tuple[float, bool]:
        """프레임 입력 → 추정 속도 반환.
        
        Args:
            frame_bgr: BGR 컬러 프레임
            timestamp: 현재 시간 [s]. None이면 자동 측정
        
        Returns:
            (velocity_m_s, is_valid): 추정 속도와 유효성 플래그
        """
        if not self._available or frame_bgr is None:
            return 0.0, False
        
        cv2 = self._cv2
        now = timestamp if timestamp is not None else time.monotonic()
        
        # 다운샘플 + 그레이 변환
        if self.downsample > 1:
            frame_bgr = cv2.resize(
                frame_bgr,
                (frame_bgr.shape[1] // self.downsample,
                 frame_bgr.shape[0] // self.downsample),
                interpolation=cv2.INTER_AREA
            )
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        
        # 첫 프레임: 저장만
        if self._prev_gray is None:
            self._prev_gray = gray
            self._last_time = now
            return 0.0, False
        
        dt = now - self._last_time
        if dt <= 0:
            return 0.0, False
        
        # Farneback dense optical flow
        flow = cv2.calcOpticalFlowFarneback(
            self._prev_gray, gray,
            None,
            pyr_scale=0.5,
            levels=3,
            winsize=15,
            iterations=3,
            poly_n=5,
            poly_sigma=1.2,
            flags=0,
        )
        
        # ROI 추출
        h, w = gray.shape
        x0 = int(self.roi_ratio[0] * w)
        y0 = int(self.roi_ratio[1] * h)
        x1 = int(self.roi_ratio[2] * w)
        y1 = int(self.roi_ratio[3] * h)
        roi_flow = flow[y0:y1, x0:x1]
        
        # 평균 흐름
        mean_vx = float(np.mean(roi_flow[..., 0]))  # 좌우 [pixel/frame]
        mean_vy = float(np.mean(roi_flow[..., 1]))  # 상하 [pixel/frame]
        
        # 카메라 전방 마운트 가정: 로봇 전진 시 풍경이 화면 아래로 흐름 (vy > 0)
        # 따라서 로봇 전진 속도 ∝ vy
        # (현재는 픽셀/sec → m/s 변환에 scale 적용; 캘리브레이션 후 정확해짐)
        flow_pixel_per_sec_y = mean_vy / dt
        flow_pixel_per_sec_x = mean_vx / dt
        
        velocity = flow_pixel_per_sec_y * self.scale  # 전진 속도 [m/s]
        
        magnitude = np.sqrt(mean_vx ** 2 + mean_vy ** 2)
        valid = bool(magnitude > self.valid_threshold)
        
        # 상태 저장
        self.last_flow_pixels = (flow_pixel_per_sec_x, flow_pixel_per_sec_y)
        self.last_velocity = velocity
        self.last_valid = valid
        
        # 다음 호출 대비
        self._prev_gray = gray
        self._last_time = now
        
        return velocity, valid
    
    def reset(self) -> None:
        self._prev_gray = None
        self._last_time = None