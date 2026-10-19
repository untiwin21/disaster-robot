"""
Main Window

전체 대시보드 레이아웃:

┌──────────────────────┬───────────────┐
│                      │  Status       │
│                      ├───────────────┤
│   Video              │  Radar        │
│                      │               │
│                      │               │
├──────────────────────┴───────────────┤
│       Help    │   Time-Series       │
└──────────────────────────────────────┘

[데이터 흐름]
- QTimer가 30Hz로 동작:
  - comm.get_latest_frame() → YOLO → video widget
  - comm.get_latest_telemetry() → 모든 패널 갱신
- VideoWidget의 키 입력 시그널 → 명령 송신
"""

import sys
import time
import math
from collections import deque
from pathlib import Path
from typing import Set, Optional

import numpy as np
import cv2
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QSplitter,
    QStatusBar, QLabel, QApplication
)

from .theme import DARK_THEME
from .video_widget import VideoWidget
from .status_panel import StatusPanel
from .radar_widget import RadarPanel
from .help_widget import HelpPanel, TimeSeriesPanel


# 키 → 동작 매핑 (단발)
DISCRETE_KEY_MAP = {
    Qt.Key_Q: 'quit',
    Qt.Key_M: 'mode',
    Qt.Key_H: 'toggle_help',
    Qt.Key_Space: 'stop',
    # 기존 그룹 키 (Front, Rear 동시) - 호환성 유지
    Qt.Key_R: 'wheel_size_front_up',
    Qt.Key_F: 'wheel_size_front_down',
    Qt.Key_T: 'wheel_size_rear_up',
    Qt.Key_G: 'wheel_size_rear_down',
    # 6개 바퀴 개별 조절
    # 확대(+): Numpad 위치 그대로
    Qt.Key_7: 'wheel_FL_up',
    Qt.Key_8: 'wheel_FR_up',
    Qt.Key_4: 'wheel_ML_up',
    Qt.Key_5: 'wheel_MR_up',
    Qt.Key_1: 'wheel_RL_up',
    Qt.Key_2: 'wheel_RR_up',
    Qt.Key_0: 'wheel_ALL_up',
    # 축소(-): Numpad와 위치적으로 매칭되는 알파벳
    Qt.Key_U:      'wheel_FL_down',
    Qt.Key_I:      'wheel_FR_down',
    Qt.Key_J:      'wheel_ML_down',
    Qt.Key_K:      'wheel_MR_down',
    Qt.Key_N:      'wheel_RL_down',     # M은 mode라 N으로
    Qt.Key_Comma:  'wheel_RR_down',
    Qt.Key_Period: 'wheel_ALL_down',
}

# 바퀴 인덱스 (HAL과 동일)
WHEEL_INDEX = {'FL': 0, 'FR': 1, 'ML': 2, 'MR': 3, 'RL': 4, 'RR': 5}

STEERING_MODES = ["Ackermann", "SkidSteer", "Crab", "DoubleAckermann"]


class MainWindow(QMainWindow):
    
    # 상수
    THROTTLE_RATE = 1.5    # 키 누르고 있을 때 throttle 변화율 [/s]
    STEER_RATE = 2.0
    AUTO_DECAY = 4.0       # 키 안 누르면 throttle/steer 줄어드는 속도
    SIZE_STEP = 0.1
    SEND_RATE = 30         # 명령 송신 주기 [Hz]
    
    def __init__(self, comm, detector, config):
        super().__init__()
        self.comm = comm
        self.detector = detector
        self.config = config
        
        self.setWindowTitle(config.gui.window_title)
        self.resize(config.gui.width, config.gui.height)
        self.setStyleSheet(DARK_THEME)
        
        # ─── 상태 ───
        self.throttle = 0.0
        self.steer = 0.0
        self.wheel_size_front = 0.0
        self.wheel_size_rear = 0.0
        self.wheel_sizes = [0.0] * 6        # [FL, FR, ML, MR, RL, RR]
        self.selected_wheel = None          # 인덱스 0~5 또는 None(전체)
        self.mode_idx = 0
        self.show_help = True
        
        self._pressed_keys: Set[int] = set()
        self._last_frame: Optional[np.ndarray] = None
        self._last_telemetry = None
        
        # FPS 측정용
        self._frame_times = deque(maxlen=30)
        self._last_telemetry_time = 0.0
        
        # ─── 위젯 ───
        self._build_layout()
        
        # ─── 타이머 ───
        # 메인 업데이트: 30Hz
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._on_tick)
        self.update_timer.start(int(1000 / self.SEND_RATE))
        
        # 키보드 처리: 더 빠르게 (60Hz)
        self.input_timer = QTimer(self)
        self.input_timer.timeout.connect(self._on_input_tick)
        self.input_timer.start(16)  # ~60Hz
        self._last_input_time = time.monotonic()
        
        # 첫 포커스
        self.video.setFocus()
    
    def _build_layout(self):
        """위젯 배치"""
        # 우측 상단: 영상
        self.video = VideoWidget()
        self.video.key_pressed.connect(self._on_discrete_key)
        self.video.keys_changed.connect(self._on_keys_changed)
        
        # 우측 패널 (Status + Radar)
        self.status_panel = StatusPanel()
        us_cfg = self.config.sensors.ultrasonic
        self.radar = RadarPanel(
            sensor_configs=us_cfg.sensors,
            max_range=us_cfg.max_range,
            danger=us_cfg.danger_threshold,
            warning=us_cfg.warning_threshold,
        )
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.addWidget(self.status_panel, 0)
        right_layout.addWidget(self.radar, 1)
        
        # 상단 (영상 + 우측 패널)
        top_split = QSplitter(Qt.Horizontal)
        top_split.addWidget(self.video)
        top_split.addWidget(right)
        top_split.setStretchFactor(0, 3)
        top_split.setStretchFactor(1, 1)
        top_split.setSizes([1100, 500])
        
        # 하단: Help + Time-series
        self.help_panel = HelpPanel()
        self.timeseries = TimeSeriesPanel(
            sensor_configs=us_cfg.sensors,
            max_range=us_cfg.max_range,
        )
        bottom_split = QSplitter(Qt.Horizontal)
        bottom_split.addWidget(self.help_panel)
        bottom_split.addWidget(self.timeseries)
        bottom_split.setStretchFactor(0, 1)
        bottom_split.setStretchFactor(1, 2)
        bottom_split.setSizes([400, 1200])
        
        # 메인 (top + bottom)
        main_split = QSplitter(Qt.Vertical)
        main_split.addWidget(top_split)
        main_split.addWidget(bottom_split)
        main_split.setStretchFactor(0, 4)
        main_split.setStretchFactor(1, 1)
        main_split.setSizes([700, 200])
        
        self.setCentralWidget(main_split)
        
        # 상태바
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Ready - 키보드는 영상 위젯에 포커스 줘야 동작")
    
    # ─── 키 입력 처리 ───
    
    def _on_keys_changed(self, keys: Set[int]):
        self._pressed_keys = keys
    
    def _on_discrete_key(self, key: int):
        """단발 키 처리"""
        action = DISCRETE_KEY_MAP.get(key)
        if action == 'quit':
            self.close()
        elif action == 'mode':
            self.mode_idx = (self.mode_idx + 1) % len(STEERING_MODES)
            new_mode = STEERING_MODES[self.mode_idx]
            self.comm.send_steering_mode(new_mode)
            self.statusBar().showMessage(f"Mode → {new_mode}", 2000)
        elif action == 'toggle_help':
            self.show_help = not self.show_help
            self.help_panel.setVisible(self.show_help)
        elif action == 'stop':
            self.throttle = 0.0
            self.steer = 0.0
            self.comm.send_stop()
            self.statusBar().showMessage("STOP", 1000)
        elif action == 'wheel_size_front_up':
            self.wheel_size_front = min(1.0, self.wheel_size_front + self.SIZE_STEP)
            self.wheel_sizes[WHEEL_INDEX['FL']] = self.wheel_size_front
            self.wheel_sizes[WHEEL_INDEX['FR']] = self.wheel_size_front
            self.comm.send_wheel_sizes(self.wheel_sizes)
        elif action == 'wheel_size_front_down':
            self.wheel_size_front = max(0.0, self.wheel_size_front - self.SIZE_STEP)
            self.wheel_sizes[WHEEL_INDEX['FL']] = self.wheel_size_front
            self.wheel_sizes[WHEEL_INDEX['FR']] = self.wheel_size_front
            self.comm.send_wheel_sizes(self.wheel_sizes)
        elif action == 'wheel_size_rear_up':
            self.wheel_size_rear = min(1.0, self.wheel_size_rear + self.SIZE_STEP)
            self.wheel_sizes[WHEEL_INDEX['RL']] = self.wheel_size_rear
            self.wheel_sizes[WHEEL_INDEX['RR']] = self.wheel_size_rear
            self.comm.send_wheel_sizes(self.wheel_sizes)
        elif action == 'wheel_size_rear_down':
            self.wheel_size_rear = max(0.0, self.wheel_size_rear - self.SIZE_STEP)
            self.wheel_sizes[WHEEL_INDEX['RL']] = self.wheel_size_rear
            self.wheel_sizes[WHEEL_INDEX['RR']] = self.wheel_size_rear
            self.comm.send_wheel_sizes(self.wheel_sizes)
        # ─── 6개 바퀴 개별 조절 (wheel_XX_up 또는 wheel_XX_down) ───
        elif action and action.startswith('wheel_'):
            parts = action.split('_')   # ['wheel', 'FL', 'up'] 또는 ['wheel', 'ALL', 'down']
            target = parts[1]
            direction = parts[2]
            delta = +self.SIZE_STEP if direction == 'up' else -self.SIZE_STEP
            self._adjust_wheel(target, delta)
    
    def _adjust_wheel(self, target: str, delta: float):
        """특정 바퀴 또는 전체 사이즈 조절.
        
        Args:
            target: 'FL', 'FR', 'ML', 'MR', 'RL', 'RR', 'ALL' 중 하나
            delta: 사이즈 변화량 (양수=확대, 음수=축소)
        """
        if target == 'ALL':
            self.wheel_sizes = [max(0.0, min(1.0, s + delta)) for s in self.wheel_sizes]
            self.statusBar().showMessage(f"ALL wheels → {self.wheel_sizes[0]:.2f}", 1000)
        else:
            i = WHEEL_INDEX[target]
            self.wheel_sizes[i] = max(0.0, min(1.0, self.wheel_sizes[i] + delta))
            self.statusBar().showMessage(f"{target} → {self.wheel_sizes[i]:.2f}", 1000)
        # 그룹 평균도 업데이트 (디스플레이용)
        self.wheel_size_front = (self.wheel_sizes[0] + self.wheel_sizes[1]) / 2
        self.wheel_size_rear = (self.wheel_sizes[4] + self.wheel_sizes[5]) / 2
        print(f"[GUI] send_wheel_sizes: {[f'{s:.2f}' for s in self.wheel_sizes]}")
        self.comm.send_wheel_sizes(self.wheel_sizes)
    
    def _on_input_tick(self):
        """연속 입력 (W/A/S/D) 처리"""
        now = time.monotonic()
        dt = now - self._last_input_time
        self._last_input_time = now
        
        # WASD 키 상태에 따라 throttle/steer 변화
        if Qt.Key_W in self._pressed_keys:
            self.throttle = min(1.0, self.throttle + self.THROTTLE_RATE * dt)
        elif Qt.Key_S in self._pressed_keys:
            self.throttle = max(-1.0, self.throttle - self.THROTTLE_RATE * dt)
        else:
            # 자동 감쇠 (관성)
            if self.throttle > 0:
                self.throttle = max(0.0, self.throttle - self.AUTO_DECAY * dt)
            elif self.throttle < 0:
                self.throttle = min(0.0, self.throttle + self.AUTO_DECAY * dt)
        
        if Qt.Key_A in self._pressed_keys:
            self.steer = min(1.0, self.steer + self.STEER_RATE * dt)
        elif Qt.Key_D in self._pressed_keys:
            self.steer = max(-1.0, self.steer - self.STEER_RATE * dt)
        else:
            # 조향은 빠르게 중심 복귀
            if self.steer > 0:
                self.steer = max(0.0, self.steer - self.AUTO_DECAY * 1.5 * dt)
            elif self.steer < 0:
                self.steer = min(0.0, self.steer + self.AUTO_DECAY * 1.5 * dt)
    
    # ─── 메인 틱 ───
    
    def _on_tick(self):
        """30Hz 메인 갱신"""
        # 영상
        jpeg_bytes = self.comm.get_latest_frame()
        if jpeg_bytes is not None:
            frame = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                # YOLO
                if self.detector is not None:
                    detections = self.detector.detect(frame)
                    if detections:
                        frame = self.detector.draw_detections(frame, detections)
                self._last_frame = frame
                
                # FPS
                self._frame_times.append(time.monotonic())
                if len(self._frame_times) >= 2:
                    span = self._frame_times[-1] - self._frame_times[0]
                    fps = (len(self._frame_times) - 1) / span if span > 0 else 0
                    self.status_panel.update_video_fps(fps)
        
        if self._last_frame is not None:
            self.video.show_frame(self._last_frame)
        
        # 텔레메트리
        tele = self.comm.get_latest_telemetry()
        if tele is not None:
            self._last_telemetry = tele
            self._last_telemetry_time = time.monotonic()
            self.status_panel.update_telemetry(tele)
            self.radar.update_distances(tele.ultrasonic)
            self.timeseries.add_readings(tele.ultrasonic)
        
        # 텔레메트리 끊김 감지
        tele_age = time.monotonic() - self._last_telemetry_time
        if self._last_telemetry is None:
            self.status_panel.update_telemetry_status("WAITING", ok=False)
        elif tele_age > 2.0:
            self.status_panel.update_telemetry_status(f"STALE {tele_age:.1f}s", ok=False)
        else:
            self.status_panel.update_telemetry_status("LIVE", ok=True)
        
        # 명령 송신 (drive)
        self.comm.send_drive(self.throttle, self.steer)
        
        # 상태 패널 갱신
        self.status_panel.update_control(
            throttle=self.throttle,
            steer=self.steer,
            mode=STEERING_MODES[self.mode_idx],
            wheel_front=self.wheel_size_front,
            wheel_rear=self.wheel_size_rear,
        )
    
    def closeEvent(self, event):
        print("[GUI] 종료 중...")
        self.update_timer.stop()
        self.input_timer.stop()
        self.comm.send_stop()
        time.sleep(0.1)
        self.comm.shutdown()
        event.accept()
