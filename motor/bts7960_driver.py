"""
BTS7960 Driver Wrapper (lgpio version)

BTS7960 H-Bridge DC 모터 드라이버 (43A 듀얼 채널).
회로도 기준: 좌측 3개 모터 병렬 (BTS#1), 우측 3개 모터 병렬 (BTS#2).

[하드웨어 결선 — Dual PWM + Split EN 방식]
RPWM + LPWM + R_EN + L_EN 방식 (분기 없이 라파에서 직결):
  - RPWM: 전진(우회전 H-bridge half) PWM 신호
  - LPWM: 후진(좌회전 H-bridge half) PWM 신호
  - R_EN, L_EN: 각각 라파 GPIO에 직결 (빵판 분기 X)
    → 코드에서 항상 동시에 HIGH/LOW로 토글하여 묶음 효과

운영 방식:
  - 전진: RPWM=duty, LPWM=0,    R_EN=HIGH, L_EN=HIGH
  - 후진: RPWM=0,    LPWM=duty, R_EN=HIGH, L_EN=HIGH
  - 정지: RPWM=0,    LPWM=0,    R_EN=HIGH, L_EN=HIGH (브레이크)
  - 코스트: R_EN=LOW, L_EN=LOW

회로도 핀 (최종, 분기 없음):
- BTS#1 (좌측 3개 묶음): GPIO 18 (RPWM), GPIO 12 (LPWM), GPIO 6 (R_EN), GPIO 16 (L_EN)
- BTS#2 (우측 3개 묶음): GPIO 19 (RPWM), GPIO 13 (LPWM), GPIO 23 (R_EN), GPIO 24 (L_EN)

[라이브러리]
lgpio 사용 (Raspberry Pi OS Trixie 표준).
pigpio가 Trixie에서 deprecated 되어 lgpio가 표준이 됨.

설치 (이미 깔려있어야 함):
    sudo apt install python3-lgpio

[PWM 자원]
- GPIO 18, 19: 하드웨어 PWM 가능 (PWM0, PWM1)
- GPIO 12, 13: 하드웨어 PWM 가능 (각각 PWM0, PWM1 채널 공유)
- 한 BTS의 RPWM/LPWM은 절대 동시 활성 안 됨 (자연스러운 채널 충돌 회피)
- 처음엔 lgpio.tx_pwm으로 통일 (SW PWM 효과적이라 충분)

[사용]
    bts = BTS7960Driver(rpwm_pin=18, lpwm_pin=12, r_en_pin=6, l_en_pin=16)
    bts.enable()                     # R_EN, L_EN 둘 다 HIGH
    bts.set(duty=0.5, direction=1)   # 50% 전진
    bts.set(duty=0.3, direction=-1)  # 30% 후진
    bts.set(duty=0.0, direction=1)   # 정지
    bts.disable()                    # R_EN, L_EN 둘 다 LOW
    bts.shutdown()
"""

from typing import Optional


# BTS7960 권장 PWM 주파수
DEFAULT_PWM_FREQ_HZ = 1000


class BTS7960Driver:
    """BTS7960 단일 채널 (좌 또는 우 그룹)"""
    
    # 클래스 레벨에서 chip 핸들 공유 (한 번만 열기)
    _chip_handle = None
    _refcount = 0
    
    def __init__(self,
                 rpwm_pin: int,
                 lpwm_pin: int,
                 r_en_pin: int,
                 l_en_pin: int,
                 frequency_hz: int = DEFAULT_PWM_FREQ_HZ,
                 chip: int = 0):
        """
        Args:
            rpwm_pin: 전진 PWM GPIO (BCM) — RPWM
            lpwm_pin: 후진 PWM GPIO (BCM) — LPWM
            r_en_pin: R_EN GPIO (BCM) — 라파에서 직결
            l_en_pin: L_EN GPIO (BCM) — 라파에서 직결
            frequency_hz: PWM 주파수
            chip: GPIO chip 번호 (라파4는 0)
        """
        self._rpwm_pin = rpwm_pin
        self._lpwm_pin = lpwm_pin
        self._r_en_pin = r_en_pin
        self._l_en_pin = l_en_pin
        self._freq = frequency_hz
        self._available = False
        self._chip = chip
        
        try:
            import lgpio
            self._lgpio = lgpio
            
            # chip 한 번만 열기 (여러 BTS7960 인스턴스가 공유)
            if BTS7960Driver._chip_handle is None:
                BTS7960Driver._chip_handle = lgpio.gpiochip_open(chip)
            BTS7960Driver._refcount += 1
            
            h = BTS7960Driver._chip_handle
            
            # GPIO 설정 (모두 출력, 초기 LOW)
            lgpio.gpio_claim_output(h, rpwm_pin, 0)
            lgpio.gpio_claim_output(h, lpwm_pin, 0)
            lgpio.gpio_claim_output(h, r_en_pin, 0)   # 시작은 disable
            lgpio.gpio_claim_output(h, l_en_pin, 0)
            
            # 초기 PWM 0%
            lgpio.tx_pwm(h, rpwm_pin, frequency_hz, 0.0)
            lgpio.tx_pwm(h, lpwm_pin, frequency_hz, 0.0)
            
            self._available = True
            print(f"[BTS7960] 초기화 완료 (RPWM={rpwm_pin}, LPWM={lpwm_pin}, R_EN={r_en_pin}, L_EN={l_en_pin}, {frequency_hz}Hz)")
        except ImportError as e:
            print(f"[BTS7960] lgpio 라이브러리 없음: {e}")
            print("[BTS7960] 설치: sudo apt install python3-lgpio")
        except Exception as e:
            print(f"[BTS7960] 초기화 실패: {e}")
    
    @property
    def is_available(self) -> bool:
        return self._available
    
    def enable(self) -> None:
        """드라이버 활성화 (R_EN, L_EN 둘 다 HIGH)"""
        if not self._available:
            return
        try:
            h = BTS7960Driver._chip_handle
            self._lgpio.gpio_write(h, self._r_en_pin, 1)
            self._lgpio.gpio_write(h, self._l_en_pin, 1)
        except Exception as e:
            print(f"[BTS7960] enable 에러: {e}")
    
    def disable(self) -> None:
        """드라이버 비활성화 (R_EN, L_EN 둘 다 LOW) — 모터 free coast"""
        if not self._available:
            return
        try:
            h = BTS7960Driver._chip_handle
            # PWM 먼저 0으로
            self._lgpio.tx_pwm(h, self._rpwm_pin, self._freq, 0.0)
            self._lgpio.tx_pwm(h, self._lpwm_pin, self._freq, 0.0)
            # EN 두 개 다 LOW
            self._lgpio.gpio_write(h, self._r_en_pin, 0)
            self._lgpio.gpio_write(h, self._l_en_pin, 0)
        except Exception as e:
            print(f"[BTS7960] disable 에러: {e}")
    
    def set(self, duty: float, direction: int) -> None:
        """
        Args:
            duty: PWM 듀티 [0, 1]
            direction: 1 (전진/RPWM) 또는 -1 (후진/LPWM)
        """
        if not self._available:
            return
        
        duty = max(0.0, min(1.0, duty))
        duty_pct = duty * 100.0
        h = BTS7960Driver._chip_handle
        
        try:
            if direction >= 0:
                # 전진: RPWM=duty, LPWM=0
                self._lgpio.tx_pwm(h, self._lpwm_pin, self._freq, 0.0)
                self._lgpio.tx_pwm(h, self._rpwm_pin, self._freq, duty_pct)
            else:
                # 후진: RPWM=0, LPWM=duty
                self._lgpio.tx_pwm(h, self._rpwm_pin, self._freq, 0.0)
                self._lgpio.tx_pwm(h, self._lpwm_pin, self._freq, duty_pct)
        except Exception as e:
            print(f"[BTS7960] set 에러: {e}")
    
    def stop(self) -> None:
        """모터 정지 (PWM=0, EN 그대로 — 브레이크)"""
        if not self._available:
            return
        try:
            h = BTS7960Driver._chip_handle
            self._lgpio.tx_pwm(h, self._rpwm_pin, self._freq, 0.0)
            self._lgpio.tx_pwm(h, self._lpwm_pin, self._freq, 0.0)
        except Exception as e:
            print(f"[BTS7960] stop 에러: {e}")
    
    def shutdown(self) -> None:
        """리소스 정리"""
        if not self._available:
            return
        try:
            self.disable()
            h = BTS7960Driver._chip_handle
            # 핀 free
            self._lgpio.gpio_free(h, self._rpwm_pin)
            self._lgpio.gpio_free(h, self._lpwm_pin)
            self._lgpio.gpio_free(h, self._r_en_pin)
            self._lgpio.gpio_free(h, self._l_en_pin)
            
            # 마지막 인스턴스면 chip 닫기
            BTS7960Driver._refcount -= 1
            if BTS7960Driver._refcount <= 0 and BTS7960Driver._chip_handle is not None:
                self._lgpio.gpiochip_close(BTS7960Driver._chip_handle)
                BTS7960Driver._chip_handle = None
                BTS7960Driver._refcount = 0
        except Exception as e:
            print(f"[BTS7960] 종료 중 에러: {e}")
        finally:
            self._available = False
