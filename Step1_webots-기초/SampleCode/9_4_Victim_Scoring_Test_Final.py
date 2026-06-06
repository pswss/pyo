# Victim_Scoring_Test_v43.py - [Debug Enhanced]
# 로직은 v42와 동일하지만, 각 단계별 이미지 저장과 상세 로그 출력을 추가하여 원인 파악을 돕습니다.

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
DEBUG_DIR = "debug_v43_debug"

if DEBUG_SAVE_IMAGES:
    if not os.path.exists(DEBUG_DIR): os.makedirs(DEBUG_DIR)
    else:
        for f in os.listdir(DEBUG_DIR):
            try: os.remove(os.path.join(DEBUG_DIR, f))
            except: pass

# --- 로봇 설정 ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())
camera = robot.getDevice('camera_centre') or robot.getDevice('camera')
camera.enable(timestep)
gps = robot.getDevice('gps'); gps.enable(timestep)
ps0 = robot.getDevice('ps0'); ps0.enable(timestep)
ps7 = robot.getDevice('ps7'); ps7.enable(timestep)
emitter = robot.getDevice('emitter')
left_motor = robot.getDevice('wheel1 motor')
right_motor = robot.getDevice('wheel2 motor')
left_motor.setPosition(float('inf')); right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0); right_motor.setVelocity(0.0)

# --- 상태 상수 ---
STATE_SEARCH = 0; STATE_APPROACH = 1; STATE_STOP = 2; STATE_REPORT = 3; STATE_FINISH = 4
current_state = STATE_SEARCH
stop_start_time = 0.0
STOP_DISTANCE = 0.12 # v20 Original
WAIT_TIME = 1.5 

def save_debug_image(step_name, img):
    if not DEBUG_SAVE_IMAGES: return
    timestamp = int(robot.getTime() * 1000)
    cv2.imwrite(f"{DEBUG_DIR}/{timestamp}_{step_name}.png", img)

# --- 1. 감지 (v20) ---
def detect_blob(camera):
    if not OPENCV_AVAILABLE: return False
    img_data = camera.getImage()
    w, h = camera.getWidth(), camera.getHeight()
    img = np.frombuffer(img_data, np.uint8).reshape((h, w, 4))
    img_bgr = img[:, :, :3]
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if 200 < cv2.contourArea(cnt) < 30000: return True
    return False

# =========================================================
# ★ Expert A: Letter 분석기 (v20 코드 100% 복원 + 로그) ★
# =========================================================
def solve_letter_v20_exact(raw_crop_img):
    print("   >> [Letter Logic] 진입")
    
    # [v20 Step 1] 내부 여백 제거 (Tight Crop)
    sub_contours, _ = cv2.findContours(raw_crop_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if sub_contours:
        largest_sub = max(sub_contours, key=cv2.contourArea)
        sx, sy, sbw, sbh = cv2.boundingRect(largest_sub)
        char_final = raw_crop_img[sy:sy+sbh, sx:sx+sbw]
    else:
        char_final = raw_crop_img

    if DEBUG_SAVE_IMAGES:
        save_debug_image("letter_tight_crop", char_final)

    # [v20 Step 2] 60x60 정규화
    normalized = cv2.resize(char_final, (60, 60))
    
    # [v20 Step 3] TMB Logic
    col_s, col_e = 24, 36 # 60px의 중앙 20% (12px)
    
    roi_top = normalized[5:15, col_s:col_e]
    roi_mid = normalized[25:35, col_s:col_e]
    roi_bot = normalized[45:55, col_s:col_e]
    
    d_top = cv2.countNonZero(roi_top) / roi_top.size
    d_mid = cv2.countNonZero(roi_mid) / roi_mid.size
    d_bot = cv2.countNonZero(roi_bot) / roi_bot.size
    
    # 로그 출력: 실제 밀도 값 확인
    print(f"      TMB Density -> T:{d_top:.2f} M:{d_mid:.2f} B:{d_bot:.2f}")
    
    is_top_solid = d_top > 0.5
    is_mid_solid = d_mid > 0.5
    is_bot_solid = d_bot > 0.5
    
    result_text = "?"
    best_char = b'U' # Default
    
    if is_mid_solid:
        if is_top_solid and is_bot_solid:
            best_char = b'S'; result_text = "S (All Solid)"
        elif not is_top_solid and not is_bot_solid:
            best_char = b'H'; result_text = "H (Mid Only)"
        else:
            best_char = b'S'; result_text = "S (Hybrid)"
    else:
        if is_bot_solid:
            best_char = b'U'; result_text = "U (Bot Solid)"
        else:
            best_char = b'U'; result_text = "U (Default)"

    # 디버그 이미지 (분석 구역 표시)
    if DEBUG_SAVE_IMAGES:
        debug_view = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
        color_s = (0, 255, 0)
        color_h = (0, 0, 255)
        cv2.rectangle(debug_view, (col_s, 5), (col_e, 15), color_s if is_top_solid else color_h, 1)
        cv2.rectangle(debug_view, (col_s, 25), (col_e, 35), color_s if is_mid_solid else color_h, 1)
        cv2.rectangle(debug_view, (col_s, 45), (col_e, 55), color_s if is_bot_solid else color_h, 1)
        save_debug_image(f"letter_result_{result_text.split()[0]}", debug_view)
            
    return best_char, result_text

# =========================================================
# ★ Expert B: Hazmat 분석기 (Shape + 로그) ★
# =========================================================
def solve_hazmat_shape(normalized_img):
    print("   >> [Hazmat Logic] 진입")
    
    # 상하 밸런스 확인
    d_top = cv2.countNonZero(normalized_img[0:20, :])
    d_bot = cv2.countNonZero(normalized_img[40:60, :])
    
    ratio = d_top / d_bot if d_bot > 0 else 999.0
    print(f"      Top Ink: {d_top}, Bot Ink: {d_bot}, Ratio: {ratio:.2f}")
    
    # 1. Corrosive (C): 역삼각형 (위가 무거움)
    if d_bot == 0 or d_top > d_bot * 1.8:
        return b'C', "Corrosive (Top Heavy)"
        
    # 2. Poison (P): 다이아몬드 (기본값)
    return b'P', "Poison (Diamond Shape)"


# =========================================================
# ★ [NEW] Expert C: 타겟팅된 Hazmat 컬러 분석기 ★
# 입력: 타겟 영역만 오려낸 BGR 컬러 이미지 (크기 다양)
# 동작: 60x60으로 리사이즈 후 HSV 색상 정밀 분석
# =========================================================
def solve_hazmat_color_targeted(cropped_bgr):
    print("   >> [Color Hazmat Logic] 진입")
    
    # 1. 정규화 (60x60 Resize)
    # 크기를 통일해야 픽셀 수 임계값을 고정할 수 있습니다.
    resized_bgr = cv2.resize(cropped_bgr, (60, 60))
    
    if DEBUG_SAVE_IMAGES:
        save_debug_image("color_target_resized", resized_bgr)

    # 2. HSV 변환
    hsv = cv2.cvtColor(resized_bgr, cv2.COLOR_BGR2HSV)

    # 3. 색상 마스크 설정 (채도(S)와 명도(V) 기준을 높여 선명한 색만 검출)
    # 빨간색은 Hue 범위가 0 근처와 180 근처 두 군데입니다.
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # 노란색 범위
    lower_yel = np.array([20, 100, 100])
    upper_yel = np.array([35, 255, 255])
    mask_yel = cv2.inRange(hsv, lower_yel, upper_yel)

    # 4. 픽셀 수 계산 (전체 3600픽셀 중 얼마나 차지하는지)
    red_count = cv2.countNonZero(mask_red)
    yel_count = cv2.countNonZero(mask_yel)

    print(f"      Color Counts -> Red: {red_count}, Yellow: {yel_count}")

    if DEBUG_SAVE_IMAGES:
        save_debug_image("mask_red", mask_red)
        save_debug_image("mask_yellow", mask_yel)

    # 5. 판단 로직 (임계값은 테스트하며 조정 필요)
    # 60x60 이미지에서 유의미한 색상 영역 기준 (예: 150픽셀 이상)
    COLOR_THRESHOLD = 120

    if yel_count > COLOR_THRESHOLD: # 노랑 있으면 O
        if red_count > COLOR_THRESHOLD / 2: # 노랑도 어느 정도 있으면 O
            return b'O', f"Organic Peroxide (R:{red_count}, Y:{yel_count})" 

    if red_count > COLOR_THRESHOLD: # 빨간색이 압도적이면 F
        return b'F', f"Flammable Solid (R:{red_count}, Y:{yel_count})"
             
    return None, None


# =========================================================
# ★ MAIN CONTROLLER: Gateway Logic ★
# =========================================================
def identify_victim_v43(camera):
    if not OPENCV_AVAILABLE: return None
    
    print("\n" + "="*40)
    print("🔍 [DEBUG] 식별 프로세스 시작")
    
    img_data = camera.getImage()
    w, h = camera.getWidth(), camera.getHeight()
    img = np.frombuffer(img_data, np.uint8).reshape((h, w, 4))
    img_bgr = img[:, :, :3]
    if DEBUG_SAVE_IMAGES: save_debug_image("step0_img", img)
    
   # 1. 컨투어 찾기 (위치 파악을 위해 가장 먼저 수행)
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 300: continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        # ★ 중요: 정밀 분석 때는 Edge Filter를 끕니다. (큰 Hazmat이 잘리는 것 방지)
        # if x<2 or y<2 or x+bw>w-2 or y+bh>h-2: continue 
        aspect = float(bw)/bh
        if aspect < 0.3 or aspect > 2.5: continue # 비율 범위 약간 넓힘
        candidates.append((area, x, y, bw, bh))

    if not candidates: 
        print("   ❌ 인식 실패: 유효한 컨투어가 없습니다.")
        return None
        
    candidates.sort(key=lambda c: c[0], reverse=True)
    target = candidates[0]
    tx, ty, tbw, tbh = target[1], target[2], target[3], target[4]
    print(f"   -> Target Found: Area={target[0]}, Box=({tx},{ty},{tbw},{tbh})")

    # =========================================================
    # ★ [PRIORITY 1] 타겟팅된 컬러 Hazmat 검사 (O/F) ★
    # =========================================================
    # 찾은 타겟 영역만큼 원본 컬러 이미지에서 오려냅니다.
    color_crop = img_bgr[ty:ty+tbh, tx:tx+tbw]
    
    # 컬러 전문가 호출
    color_res, color_reason = solve_hazmat_color_targeted(color_crop)
    
    if color_res is not None:
        print(f"🎉 컬러 Hazmat 식별 성공: {color_res.decode()} ({color_reason})")
        return color_res # O 또는 F 발견 시 즉시 리턴

    print("   -> 컬러 Hazmat 아님. B/W 분석으로 진행.")

    # =========================================================
    # ★ [PRIORITY 2] B/W Gateway & Analysis (P/C/H/S/U) ★
    # =========================================================
    # 컬러가 아니면 기존의 흑백 분석 로직을 수행합니다.
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    if DEBUG_SAVE_IMAGES: save_debug_image("step1_thresh", thresh)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for i, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        if area < 250:
            print(f"   [탈락] 너무 작음: {area}")
            continue
            
        x, y, bw, bh = cv2.boundingRect(cnt)
        
        # Edge check
        if x<2 or y<2 or x+bw>w-2 or y+bh>h-2:
            print(f"   [탈락] 가장자리 닿음 (Edge)")
            continue
            
        aspect = float(bw)/bh
        if aspect < 0.3 or aspect > 2.0:
            print(f"   [탈락] 비율 이상: {aspect:.2f}")
            continue
            
        # 통과!
        candidates.append((area, x, y, bw, bh))

    print(f"   -> Valid Candidates: {len(candidates)}")
    if not candidates: 
        print("   ❌ 인식 실패: 유효한 컨투어가 없습니다.")
        return None
        
    candidates.sort(key=lambda c: c[0], reverse=True)
    target = candidates[0]
    tx, ty, tbw, tbh = target[1], target[2], target[3], target[4]
    
    print(f"   -> Target Selected: Area={target[0]}")
    
    # ★ RAW CROP (이것을 v20 로직에 그대로 넘길 예정) ★
    raw_crop = thresh[ty:ty+tbh, tx:tx+tbw]
    if DEBUG_SAVE_IMAGES: save_debug_image("step2_raw_crop", raw_crop)
    
    # -------------------------------------------------------------
    # [3] Gateway Determination (반전 로직 수정)
    # -------------------------------------------------------------
    
    # [수정된 부분] Safety Shave: 테두리 노이즈 제거
    rh, rw = raw_crop.shape
    # 이미지가 충분히 클 때만 2픽셀씩 깎아냄 (너무 작으면 원본 사용)
    if rh > 8 and rw > 8:
        # 상하좌우 2px 잘라낸 안쪽 이미지로 배경 판단
        shaved_crop = raw_crop[2:rh-2, 2:rw-2]
    else:
        shaved_crop = raw_crop

    # 판단용 리사이즈 (깎아낸 이미지 사용)
    temp_norm = cv2.resize(shaved_crop, (60, 60), interpolation=cv2.INTER_NEAREST)
    
    # 배경색 판단 (테두리 평균)
    border_px = np.concatenate([temp_norm[0:2,:].flatten(), temp_norm[-2:,:].flatten(), temp_norm[:,0:2].flatten(), temp_norm[:,-2:].flatten()])
    border_mean = np.mean(border_px)
    
    print(f"   -> Border Mean (Shaved): {border_mean:.2f}")

    if border_mean > 127:
        print("   💡 [Invert] 흰 배경 감지됨 -> 반전 수행")
        # 판단용 이미지 반전
        temp_norm = cv2.bitwise_not(temp_norm)
        # 중요: 원본 raw_crop도 반전시켜서 전문가에게 넘겨야 함
        raw_crop = cv2.bitwise_not(raw_crop)
    else:
        print("   -> [Keep] 검은 배경 유지")

    # [저장] 반전 처리 후 상태 확인
    if DEBUG_SAVE_IMAGES: save_debug_image("step2_5_processed_crop", raw_crop)
    
    # -------------------------------------------------------------
    # [3] Gateway: 8-Point Probe
    # -------------------------------------------------------------
    # Safety Shave & Resize
    rh, rw = raw_crop.shape
    shaved = raw_crop[2:rh-2, 2:rw-2] if rh>8 and rw>8 else raw_crop
    temp_norm = cv2.resize(shaved, (60, 60), interpolation=cv2.INTER_NEAREST)
    
    # Auto Invert
    border_px = np.concatenate([temp_norm[0:2,:].flatten(), temp_norm[-2:,:].flatten(), temp_norm[:,0:2].flatten(), temp_norm[:,-2:].flatten()])
    if np.mean(border_px) > 127:
        print("   💡 [Invert] 흰 배경 감지됨 -> 반전")
        temp_norm = cv2.bitwise_not(temp_norm)
        raw_crop = cv2.bitwise_not(raw_crop)
    
    # --- [Probe Definition] ---
    # 4 Corners (12x12)
    CS = 12
    f_tl = cv2.countNonZero(temp_norm[0:CS, 0:CS]) / (CS*CS)
    f_tr = cv2.countNonZero(temp_norm[0:CS, 60-CS:60]) / (CS*CS)
    f_bl = cv2.countNonZero(temp_norm[60-CS:60, 0:CS]) / (CS*CS)
    f_br = cv2.countNonZero(temp_norm[60-CS:60, 60-CS:60]) / (CS*CS)

    # 4 Edge Centers (10x10) - 찌르기 포인트
    # 상(Top), 하(Bot), 좌(Left), 우(Right) 중앙
    roi_tc = temp_norm[0:10, 25:35]        # Top Center
    roi_bc = temp_norm[50:60, 25:35]       # Bot Center
    roi_lc = temp_norm[25:35, 0:10]        # Left Center
    roi_rc = temp_norm[25:35, 50:60]       # Right Center
    
    f_tc = cv2.countNonZero(roi_tc) / 100.0
    f_bc = cv2.countNonZero(roi_bc) / 100.0
    f_lc = cv2.countNonZero(roi_lc) / 100.0
    f_rc = cv2.countNonZero(roi_rc) / 100.0

    print(f"   📐 Corners: TL{f_tl:.1f} TR{f_tr:.1f} BL{f_bl:.1f} BR{f_br:.1f}")
    print(f"   👉 Probes : TC{f_tc:.1f} BC{f_bc:.1f} LC{f_lc:.1f} RC{f_rc:.1f}")

    final_res = b'?'
    final_reason = ""
    
    # 디버그 뷰 생성 (8개 포인트 그리기)
    debug_view = cv2.cvtColor(temp_norm, cv2.COLOR_GRAY2BGR)
    red, green = (0,0,255), (0,255,0)
    # Corners
    cv2.rectangle(debug_view, (0,0), (CS,CS), green if f_tl>0.2 else red, 1)
    cv2.rectangle(debug_view, (60-CS,0), (60,CS), green if f_tr>0.2 else red, 1)
    cv2.rectangle(debug_view, (0,60-CS), (CS,60), green if f_bl>0.2 else red, 1)
    cv2.rectangle(debug_view, (60-CS,60-CS), (60,60), green if f_br>0.2 else red, 1)
    # Centers
    cv2.rectangle(debug_view, (25,0), (35,10), green if f_tc>0.4 else red, 1)
    cv2.rectangle(debug_view, (25,50), (35,60), green if f_bc>0.4 else red, 1)
    cv2.rectangle(debug_view, (0,25), (10,35), green if f_lc>0.4 else red, 1)
    cv2.rectangle(debug_view, (50,25), (60,35), green if f_rc>0.4 else red, 1)
    save_debug_image("step3_8point_probe", debug_view)

    # === [판단 로직] ===

    # [조건 1] 귀퉁이가 비어있다 (마름모 후보)
    # 네 귀퉁이가 모두 20% 미만으로 비어있음
    if f_tl < 0.2 and f_tr < 0.2 and f_bl < 0.2 and f_br < 0.2:
        print("   -> [1] Corners Empty -> Checking Probes...")
        
        # 4변의 중앙을 찔러본다.
        # 마름모(P)는 4군데(혹은 최소 3군데)가 꽉 차있음
        # 특히 ★Top Center★가 차 있어야 마름모임 (U는 TC가 빔)
        filled_centers = sum([f_tc > 0.4, f_bc > 0.4, f_lc > 0.4, f_rc > 0.4])
        
        if filled_centers >= 3 and f_tc > 0.4:
            print(f"      -> 3+ Probes Filled & Top Hit! (Count: {filled_centers}) -> Poison")
            final_res, final_reason = solve_hazmat_shape(temp_norm)
        else:
            print(f"      -> Probes Failed (Count: {filled_centers}, Top:{f_tc:.1f}) -> Letter (U/H)")
            final_res, final_reason = solve_letter_v20_exact(raw_crop)

    # [조건 2] 귀퉁이가 차있다 (사각형 후보)
    else:
        print("   -> [2] Corners Filled -> Checking Corrosive...")
        # 역삼각형(C) 특수 체크: 상단 귀퉁이는 찼는데, 하단 귀퉁이는 비었음
        if f_bl < 0.2 and f_br < 0.2 and f_tl > 0.2 and f_tr > 0.2:
            print("      -> Top Corners Filled & Bot Empty -> Corrosive")
            final_res, final_reason = solve_hazmat_shape(temp_norm)
        else:
            print("      -> All Corners Filled -> Letter")
            final_res, final_reason = solve_letter_v20_exact(raw_crop)

    print(f"⚖️ 최종 판결: {final_res.decode()} ({final_reason})")
    return final_res

# --- 메인 루프 ---
print("🚀 [v43] 디버깅 강화 모드")
detected_type = None

while robot.step(timestep) != -1:
    dist_front = (ps0.getValue() + ps7.getValue()) / 2
    gps_vals = gps.getValues()
    x_cm, z_cm = int(gps_vals[0]*100), int(gps_vals[2]*100)

    # 1. 탐색
    if current_state == STATE_SEARCH:
        if detect_blob(camera):
            current_state = STATE_APPROACH
        else:
            left_motor.setVelocity(3.0); right_motor.setVelocity(3.0)
        if dist_front < 0.05: left_motor.setVelocity(0); right_motor.setVelocity(0)

    # 2. 접근
    elif current_state == STATE_APPROACH:
        if not detect_blob(camera): pass 
        if dist_front < STOP_DISTANCE:
            if dist_front > 0.04:
                left_motor.setVelocity(0); right_motor.setVelocity(0)
                stop_start_time = robot.getTime()
                current_state = STATE_STOP
            else:
                left_motor.setVelocity(0); right_motor.setVelocity(0)
                current_state = STATE_FINISH 
        else:
            left_motor.setVelocity(2.0); right_motor.setVelocity(2.0)

    # 3. 식별
    elif current_state == STATE_STOP:
        confirm_type = identify_victim_v43(camera) # 디버깅 함수 호출
        if confirm_type: detected_type = confirm_type
        
        if robot.getTime() - stop_start_time > WAIT_TIME:
            if detected_type:
                print(f"🎉 확정: {detected_type.decode()}")
                current_state = STATE_REPORT
            else:
                print("❓ 식별 실패")
                current_state = STATE_SEARCH

    # 4. 신고
    elif current_state == STATE_REPORT:
        try:
            msg = struct.pack('i i c', x_cm, z_cm, detected_type if detected_type else b'H')
            emitter.send(msg)
            print(f"📡 전송: {msg}")
        except: pass
        current_state = STATE_FINISH
    elif current_state == STATE_FINISH: pass