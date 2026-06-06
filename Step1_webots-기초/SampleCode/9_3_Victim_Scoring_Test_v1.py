# Victim_Scoring_Test_v2.py - GPS 좌표 포함 전송으로 서버 크래시 해결
from controller import Robot, DistanceSensor, Camera, Emitter, GPS
import struct
import time

# OpenCV 라이브러리
try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False
    print("⚠️ OpenCV가 설치되지 않았습니다.")

# --- 로봇 설정 ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 1. 카메라 설정
camera = robot.getDevice('camera_centre')
if camera is None: camera = robot.getDevice('camera')
camera.enable(timestep)

# 2. GPS 설정 (좌표 획득용)
gps = robot.getDevice('gps')
gps.enable(timestep)

# 3. 거리 센서
ps0 = robot.getDevice('ps0') # 좌측 전방
ps7 = robot.getDevice('ps7') # 우측 전방
ps0.enable(timestep)
ps7.enable(timestep)

# 4. 통신 장치
emitter = robot.getDevice('emitter')

# 5. 모터
left_motor = robot.getDevice('wheel1 motor')
right_motor = robot.getDevice('wheel2 motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# --- 상태 상수 ---
STATE_APPROACH = 0  
STATE_STOP = 1      
STATE_REPORT = 2    
STATE_FINISH = 3    

current_state = STATE_APPROACH
stop_start_time = 0.0
STOP_DISTANCE = 0.12 # 12cm 앞 정지
WAIT_TIME = 1.3      # 1.3초 대기

# 비전 인식 함수
def detect_black_contour(camera):
    if not OPENCV_AVAILABLE: return False
    img_data = camera.getImage()
    w, h = camera.getWidth(), camera.getHeight()
    img = np.frombuffer(img_data, np.uint8).reshape((h, w, 4))
    gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
    _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if 200 < cv2.contourArea(cnt) < 10000: return True
    return False

# --- 메인 루프 ---
print("🚀 피해자 식별 v2 (GPS 좌표 포함) 시작!")

while robot.step(timestep) != -1:
    
    # 센서 값
    dist_front = (ps0.getValue() + ps7.getValue()) / 2
    victim_detected = detect_black_contour(camera)
    
    # GPS 값 읽기 (미터 단위 -> cm 단위 변환 필요)
    gps_vals = gps.getValues()
    current_x_cm = int(gps_vals[0] * 100) # 미터 * 100 = cm
    current_z_cm = int(gps_vals[2] * 100)

    # --- 상태 머신 ---
    
    if current_state == STATE_APPROACH:
        if victim_detected and dist_front < STOP_DISTANCE and dist_front > 0.05:
            print(f"🛑 발견! 정지 (거리: {dist_front:.3f})")
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            stop_start_time = robot.getTime()
            current_state = STATE_STOP
        elif dist_front < 0.05: # 충돌 방지
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print("🧱 벽 충돌!")
            current_state = STATE_FINISH
        else:
            # 접근 중
            if victim_detected:
                print("👀 타겟 접근 중...")
                left_motor.setVelocity(2.0)
                right_motor.setVelocity(2.0)
            else:
                left_motor.setVelocity(4.0)
                right_motor.setVelocity(4.0)

    elif current_state == STATE_STOP:
        if robot.getTime() - stop_start_time > WAIT_TIME:
            print("⏱️ 대기 완료. 신고 전송...")
            current_state = STATE_REPORT

    elif current_state == STATE_REPORT:
        try:
            # ★ 핵심 수정: 좌표(x, z)와 타입(c)을 묶어서 전송
            # pack('i i c') -> int(4byte), int(4byte), char(1byte)
            # 좌표는 cm 단위 정수로 보내는 것이 일반적입니다.
            
            message = struct.pack('i i c', current_x_cm, current_z_cm, b'H')
            emitter.send(message)
            
            print(f"📡 [전송 성공] Pos:({current_x_cm}, {current_z_cm}) Type:H")
            print("✅ MainSupervisor 로그를 확인하여 점수 획득을 체크하세요!")
            
        except Exception as e:
            print(f"⚠️ 전송 오류: {e}")
            
        current_state = STATE_FINISH

    elif current_state == STATE_FINISH:
        pass