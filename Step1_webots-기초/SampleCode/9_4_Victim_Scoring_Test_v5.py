# Victim_Scoring_Test_v20.py - [Fix] 글자 영역 정밀 Crop & Normalize
# 검은 테두리를 제거하고 글자 알맹이만 타이트하게 잘라내어 분석 정확도 극대화

from controller import Robot, DistanceSensor, Camera, Emitter, GPS
import struct
import time
import math
import os

try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False
    print("⚠️ OpenCV 설치 필요")

# --- 디버그 설정 ---
DEBUG_SAVE_IMAGES = True
DEBUG_DIR = "debug_v20_tight"

if DEBUG_SAVE_IMAGES:
    if not os.path.exists(DEBUG_DIR): os.makedirs(DEBUG_DIR)
    else:
        for f in os.listdir(DEBUG_DIR):
            try: os.remove(os.path.join(DEBUG_DIR, f))
            except: pass

# --- 로봇 설정 ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice('camera_centre')
if camera is None: camera = robot.getDevice('camera')
camera.enable(timestep)

gps = robot.getDevice('gps')
gps.enable(timestep)

ps0 = robot.getDevice('ps0')
ps7 = robot.getDevice('ps7')
ps0.enable(timestep)
ps7.enable(timestep)

emitter = robot.getDevice('emitter')

left_motor = robot.getDevice('wheel1 motor')
right_motor = robot.getDevice('wheel2 motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# --- 상태 상수 ---
STATE_SEARCH = 0    
STATE_APPROACH = 1  
STATE_STOP = 2      
STATE_REPORT = 3    
STATE_FINISH = 4    

current_state = STATE_SEARCH
stop_start_time = 0.0
STOP_DISTANCE = 0.12
WAIT_TIME = 1.5 
last_save_time = 0

def save_debug_image(step_name, img):
    if not DEBUG_SAVE_IMAGES: return
    timestamp = int(robot.getTime() * 1000)
    cv2.imwrite(f"{DEBUG_DIR}/{timestamp}_{step_name}.png", img)

# --- 1단계: 감지 ---
def detect_blob(camera):
    if not OPENCV_AVAILABLE: return False
    img_data = camera.getImage()
    w, h = camera.getWidth(), camera.getHeight()
    img = np.frombuffer(img_data, np.uint8).reshape((h, w, 4))
    img_bgr = img[:, :, :3]
    
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    # 단순 이진화가 아닌 적응형 이진화로 글자 획을 찾음
    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if 200 < cv2.contourArea(cnt) < 30000: return True
    return False

# --- ★ 2단계: 정밀 식별 (Tight Crop & TMB Analysis) ★ ---
def identify_victim_tight_crop(camera):
    global last_save_time
    if not OPENCV_AVAILABLE: return None

    current_time = robot.getTime()
    do_save = False
    if current_time - last_save_time > 0.5:
        do_save = True
        last_save_time = current_time

    img_data = camera.getImage()
    w, h = camera.getWidth(), camera.getHeight()
    img = np.frombuffer(img_data, np.uint8).reshape((h, w, 4))
    img_bgr = img[:, :, :3]
    
    # [Hazmat Check] (중앙부만)
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    roi_h = hsv[int(h*0.3):int(h*0.7), int(w*0.3):int(w*0.7)]
    mask_red = cv2.inRange(roi_h, np.array([0, 70, 50]), np.array([10, 255, 255]))
    mask_yellow = cv2.inRange(roi_h, np.array([20, 70, 50]), np.array([35, 255, 255]))
    
    if cv2.countNonZero(mask_red) > 50:
        return b'O' if cv2.countNonZero(mask_yellow) > 20 else b'F'

    # [Letter Check]
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    
    # 1. 이진화 (검은 글자 -> 흰색 덩어리)
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    if do_save: save_debug_image("full_thresh", thresh)

    # 2. 모든 윤곽선 찾기 (계층 구조 포함)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    best_char = None
    
    # 가장 적절한 '글자' 윤곽선을 찾기 위한 변수
    candidates = []

    for i, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        if area < 300: continue # 너무 작은 노이즈 무시
        
        x, y, bw, bh = cv2.boundingRect(cnt)
        
        # 화면 가장자리에 붙은 건 무시 (노이즈일 확률 높음)
        if x < 2 or y < 2 or (x+bw) > w-2 or (y+bh) > h-2: continue

        aspect = float(bw)/bh
        # 너무 길쭉하거나 납작한 건 글자가 아님
        if aspect < 0.3 or aspect > 2.0: continue
        
        # 글자 후보로 등록 (면적, 바운딩박스, 이미지)
        candidates.append((area, x, y, bw, bh))

    # 후보 중 '중앙에 가깝고' '적당히 큰' 것을 선택
    candidates.sort(key=lambda c: c[0], reverse=True) # 면적 큰 순서 정렬

    if not candidates: return None
    
    # 가장 큰 덩어리 선택 (보통 벽 토큰 배경이 잡힐 수 있음)
    # 만약 벽 배경(흰색 네모)이 잡혔다면, 그 안의 '검은 글자'는 반전되어 구멍으로 나옵니다.
    # 따라서 우리는 '흰색 픽셀(글자 획)'이 꽉 찬 영역을 찾아야 합니다.
    
    # 안전하게 가장 큰 후보를 선택하여 Crop 수행
    target = candidates[0] 
    tx, ty, tbw, tbh = target[1], target[2], target[3], target[4]
    
    # --- ★ 핵심: 정밀 Crop & Resize ★ ---
    # 1. 바운딩 박스만큼 잘라냄
    char_crop = thresh[ty:ty+tbh, tx:tx+tbw]
    
    # 2. 내부 여백 제거 (글자 알맹이만 남기기)
    # 잘라낸 이미지 안에서 다시 한번 윤곽선을 찾아 꽉 차게 줄임
    sub_contours, _ = cv2.findContours(char_crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if sub_contours:
        largest_sub = max(sub_contours, key=cv2.contourArea)
        sx, sy, sbw, sbh = cv2.boundingRect(largest_sub)
        # 진짜 글자 영역만 다시 자름
        char_final = char_crop[sy:sy+sbh, sx:sx+sbw]
    else:
        char_final = char_crop

    # 3. 60x60 정규화 (비율 유지하지 않고 꽉 채움 -> 특징 극대화)
    # S나 U나 60x60 박스에 꽉 차게 늘어납니다.
    normalized = cv2.resize(char_final, (60, 60))
    
    # --- ★ 정밀 분석 (TMB Logic) ★ ---
    # 정중앙 12~18px (너비 10%) 영역 검사
    col_s, col_e = 24, 36 # 60px의 중앙 20% (12px)
    
    roi_top = normalized[5:15, col_s:col_e]
    roi_mid = normalized[25:35, col_s:col_e]
    roi_bot = normalized[45:55, col_s:col_e]
    
    d_top = cv2.countNonZero(roi_top) / roi_top.size
    d_mid = cv2.countNonZero(roi_mid) / roi_mid.size
    d_bot = cv2.countNonZero(roi_bot) / roi_bot.size
    
    is_top_solid = d_top > 0.5
    is_mid_solid = d_mid > 0.5
    is_bot_solid = d_bot > 0.5
    
    result_text = "?"
    
    if is_mid_solid:
        # 중간이 막힘 -> S 아니면 H
        if is_top_solid and is_bot_solid:
            best_char = b'S'
            result_text = "S (All Solid)"
        elif not is_top_solid and not is_bot_solid:
            best_char = b'H'
            result_text = "H (Mid Only)"
        else:
            # 하이브리드 S (보통 대각선 때문에 위아래 중 하나는 막힘)
            best_char = b'S'
            result_text = "S (Hybrid)"
    else:
        # 중간이 뚫림 -> U
        if is_bot_solid:
            best_char = b'U'
            result_text = "U (Bot Solid)"
        else:
            # 예외: H의 기둥 사이일 수도 있으나 U 우선
            best_char = b'U' 
            result_text = "U (Default)"

    if do_save:
        state_str = f"T:{d_top:.2f} M:{d_mid:.2f} B:{d_bot:.2f}"
        print(f"📊 [Tight분석] {state_str} -> {result_text}")
        
        debug_view = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
        color_s = (0, 255, 0)
        color_h = (0, 0, 255)
        
        cv2.rectangle(debug_view, (col_s, 5), (col_e, 15), color_s if is_top_solid else color_h, 1)
        cv2.rectangle(debug_view, (col_s, 25), (col_e, 35), color_s if is_mid_solid else color_h, 1)
        cv2.rectangle(debug_view, (col_s, 45), (col_e, 55), color_s if is_bot_solid else color_h, 1)
        
        save_debug_image(f"final_crop_{result_text.split()[0]}", char_final) # 잘린 원본 확인
        save_debug_image(f"normalized_{result_text.split()[0]}", debug_view) # 분석된 이미지 확인

    return best_char

# --- 메인 루프 ---
print("🚀 [v20] 타이트한 Crop & Resize 모드 시작!")
detected_type = None

while robot.step(timestep) != -1:
    dist_front = (ps0.getValue() + ps7.getValue()) / 2
    gps_vals = gps.getValues()
    current_x_cm = int(gps_vals[0] * 100)
    current_z_cm = int(gps_vals[2] * 100)

    # 1. 탐색
    if current_state == STATE_SEARCH:
        if detect_blob(camera):
            print("👀 발견! 접근 모드")
            current_state = STATE_APPROACH
        else:
            left_motor.setVelocity(3.0)
            right_motor.setVelocity(3.0)
            
        if dist_front < 0.05: 
             left_motor.setVelocity(0)
             right_motor.setVelocity(0)

    # 2. 접근
    elif current_state == STATE_APPROACH:
        if not detect_blob(camera): pass 
        if dist_front < STOP_DISTANCE:
            if dist_front > 0.04:
                print(f"🛑 정밀 분석 위치({dist_front:.2f}).")
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                stop_start_time = robot.getTime()
                current_state = STATE_STOP
            else:
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                current_state = STATE_FINISH 
        else:
            left_motor.setVelocity(2.0)
            right_motor.setVelocity(2.0)

    # 3. 정밀 분석
    elif current_state == STATE_STOP:
        confirm_type = identify_victim_tight_crop(camera)
        
        if confirm_type:
            detected_type = confirm_type
            
        if robot.getTime() - stop_start_time > WAIT_TIME:
            if detected_type:
                print(f"🎉 최종 식별: {detected_type.decode()}")
                current_state = STATE_REPORT
            else:
                print("❓ 식별 실패. 재탐색")
                current_state = STATE_SEARCH

    # 4. 신고
    elif current_state == STATE_REPORT:
        try:
            msg_type = detected_type if detected_type else b'H'
            message = struct.pack('i i c', current_x_cm, current_z_cm, msg_type)
            emitter.send(message)
            print(f"📡 신고 전송: {msg_type.decode()} at ({current_x_cm}, {current_z_cm})")
        except Exception as e:
            print(f"⚠️ 오류: {e}")
        current_state = STATE_FINISH

    elif current_state == STATE_FINISH:
        pass