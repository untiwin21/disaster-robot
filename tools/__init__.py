"""
Hardware test/calibration tools for the disaster robot.

Tools:
- pca9685_scan.py    : I2C + PCA9685 연결 확인
- servo_calibration.py : 12개 서보 캘리브레이션 (대화식)
- dc_motor_test.py   : DC 모터 좌/우 동작 확인 + min_duty 측정

순서:
1. pca9685_scan.py  → I2C 정상 확인
2. dc_motor_test.py → DC 모터 동작 확인
3. servo_calibration.py → 서보 한계 측정 + 저장
"""
