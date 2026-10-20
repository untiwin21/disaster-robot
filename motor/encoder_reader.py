"""
Encoder Reader (lgpio callback 기반)

JGB37-520B 엔코더(11 PPR × 감속비 × 4 quadrature) 입력 처리.

[원리]
- A/B 두 상이 90도 phase shift된 quadrature 신호
- A의 rising edge일 때 B 값에 따라 회전 방향 결정:
    A↑ & B=LOW  → 정방향 +1
    A↑ & B=HIGH → 역방향 -1
- 4×decoding: A의 rising/falling, B의 rising/falling 모두 카운트
  → quadrature counts/rev = PPR × 4 × gear_ratio

[너 모터 (12V 107RPM, 1:90)]
- 11 PPR × 4 × 90 = 3960 counts/output_revolution
- 바퀴 둘레 204.2mm → 0.0516 mm/count (이론 분해능)

[lgpio callback]
- gpio_claim_alert로 edge 인터럽트 등록
- callback 함수가 자동 호출됨 (스레드 안전)
- 카운팅은 atomic int 사용 (threading.Lock 불필요한 수준)

[사용]
    enc = EncoderReader(c1_pin=17, c2_pin=27, counts_per_rev=3960, wheel_circumference_m=0.2042)
    # ... 메인 루프 ...
    v, dist = enc.compute_velocity(dt=0.02)   # m/s, m (이번 dt 동안 이동)
    enc.shutdown()
"""

import threading
from typing import Optional


class EncoderReader:
    """단일 엔코더(A/B 상) quadrature 카운팅."""
    
    # lgpio chip handle (다른 모듈과 공유)
    _chip_handle = None
    _refcount = 0
    
    def __init__(self,
                 c1_pin: int,
                 c2_pin: int,
                 counts_per_rev: int = 3960,
                 wheel_circumference_m: float = 0.2042,
                 chip: int = 0,
                 invert: bool = False):
        """
        Args:
            c1_pin: 엔코더 A상 GPIO (BCM)
            c2_pin: 엔코더 B상 GPIO (BCM)
            counts_per_rev: 출력축 1회전당 quadrature 카운트 (기본 3960 = 11×4×90)
            wheel_circumference_m: 바퀴 둘레 [m]
            chip: GPIO chip 번호 (라파4 = 0)
            invert: True면 카운트 부호 반전 (모터 결선 방향 반대일 때)
        """
        self._c1 = c1_pin
        self._c2 = c2_pin
        self._counts_per_rev = counts_per_rev
        self._circ = wheel_circumference_m
        self._meters_per_count = wheel_circumference_m / counts_per_rev
        self._invert = invert
        
        # 카운터 (callback에서 갱신됨)
        self._count = 0
        self._lock = threading.Lock()
        
        # 속도 계산용 (이전 측정 시점 카운트)
        self._last_velocity_count = 0
        
        self._available = False
        self._callbacks = []  # 콜백 핸들 보관 (가비지 콜렉트 방지)
        
        try:
            import lgpio
            self._lgpio = lgpio
            
            if EncoderReader._chip_handle is None:
                EncoderReader._chip_handle = lgpio.gpiochip_open(chip)
            EncoderReader._refcount += 1
            
            h = EncoderReader._chip_handle
            
            # Both edge alert (rising + falling)
            BOTH_EDGES = lgpio.BOTH_EDGES
            lgpio.gpio_claim_alert(h, c1_pin, BOTH_EDGES, lgpio.SET_PULL_UP)
            lgpio.gpio_claim_alert(h, c2_pin, BOTH_EDGES, lgpio.SET_PULL_UP)
            
            # 콜백 등록
            cb1 = lgpio.callback(h, c1_pin, BOTH_EDGES, self._on_c1)
            cb2 = lgpio.callback(h, c2_pin, BOTH_EDGES, self._on_c2)
            self._callbacks = [cb1, cb2]
            
            # 현재 상태 초기화
            self._a_state = lgpio.gpio_read(h, c1_pin)
            self._b_state = lgpio.gpio_read(h, c2_pin)
            
            self._available = True
            print(f"[Encoder] 초기화 완료 (A={c1_pin}, B={c2_pin}, "
                  f"{counts_per_rev} counts/rev, {self._meters_per_count*1000:.4f} mm/count)")
        except ImportError as e:
            print(f"[Encoder] lgpio 라이브러리 없음: {e}")
        except Exception as e:
            print(f"[Encoder] 초기화 실패: {e}")
    
    @property
    def is_available(self) -> bool:
        return self._available
    
    def _on_c1(self, chip, gpio, level, tick):
        """A상 edge 콜백.
        
        rising A:  B=0 → CW (+1),  B=1 → CCW (-1)
        falling A: B=0 → CCW (-1), B=1 → CW (+1)
        """
        if level == 2:  # watchdog timeout, 무시
            return
        self._a_state = level
        b = self._b_state
        # A의 변화 + B 상태 → 방향 결정
        if level == 1:   # rising
            delta = 1 if b == 0 else -1
        else:             # falling
            delta = 1 if b == 1 else -1
        if self._invert:
            delta = -delta
        with self._lock:
            self._count += delta
    
    def _on_c2(self, chip, gpio, level, tick):
        """B상 edge 콜백.
        
        rising B:  A=0 → CCW (-1), A=1 → CW (+1)
        falling B: A=0 → CW (+1),  A=1 → CCW (-1)
        """
        if level == 2:
            return
        self._b_state = level
        a = self._a_state
        if level == 1:
            delta = 1 if a == 1 else -1
        else:
            delta = 1 if a == 0 else -1
        if self._invert:
            delta = -delta
        with self._lock:
            self._count += delta
    
    def get_count(self) -> int:
        """현재까지 누적 카운트."""
        with self._lock:
            return self._count
    
    def get_distance(self) -> float:
        """누적 이동 거리 [m]."""
        return self.get_count() * self._meters_per_count
    
    def compute_velocity(self, dt: float) -> tuple:
        """이번 dt 동안의 속도 + 이동 거리.
        
        Args:
            dt: 마지막 호출 이후 경과 시간 [s]
        
        Returns:
            (velocity_m_s, distance_m)
        """
        with self._lock:
            current = self._count
        delta_count = current - self._last_velocity_count
        self._last_velocity_count = current
        
        distance = delta_count * self._meters_per_count
        if dt > 0:
            velocity = distance / dt
        else:
            velocity = 0.0
        return velocity, distance
    
    def reset(self) -> None:
        """카운트 0으로 리셋."""
        with self._lock:
            self._count = 0
            self._last_velocity_count = 0
    
    def shutdown(self) -> None:
        if not self._available:
            return
        try:
            # 콜백 해제
            for cb in self._callbacks:
                cb.cancel()
            self._callbacks = []
            
            h = EncoderReader._chip_handle
            self._lgpio.gpio_free(h, self._c1)
            self._lgpio.gpio_free(h, self._c2)
            
            EncoderReader._refcount -= 1
            if EncoderReader._refcount <= 0 and EncoderReader._chip_handle is not None:
                # 다른 모듈(BTS7960)이 같은 chip 쓸 수 있으니 닫지 말고 두기
                # 같은 chip handle을 BTS7960과 공유하지 않으면 여기서 닫아도 OK
                # 안전하게 그냥 두자
                pass
        except Exception as e:
            print(f"[Encoder] 종료 중 에러: {e}")
        finally:
            self._available = False
