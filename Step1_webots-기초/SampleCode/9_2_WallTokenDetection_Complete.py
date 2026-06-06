# WallTokenDetection_Fixed.py
from controller import Robot
import cv2
import numpy as np
import struct

timeStep = 32
robot = Robot()

# 초기화
camera = robot.getDevice("camera_centre")
camera.enable(timeStep)

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(timeStep)

left_motor = robot.getDevice("wheel1 motor")
right_motor = robot.getDevice("wheel2 motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# 거리 센서
front_sensor = robot.getDevice('ps0')
front_sensor.enable(timeStep)

def detect_wall_tokens(image_data, camera):
    """벽 토큰 감지 - 개선 버전"""
    token_list = []
    
    img = np.array(np.frombuffer(image_data, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)))
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    
    # 그레이스케일 변환
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 블러 적용
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # 이진화 (여러 방법 시도)
    # 방법 1: 밝은 영역 감지
    _, mask_bright = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)
    
    # 방법 2: Otsu 자동 임계값
    _, mask_otsu = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # 방법 3: 어두운 배경에서 밝은 객체 (역이진화)
    _, mask_inv = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)
    
    # 모든 마스크 결합
    mask = cv2.bitwise_or(mask_bright, mask_otsu)
    
    # HSV 추가 검사 (컬러 토큰)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # 흰색/밝은 색
    mask_white = cv2.inRange(hsv, np.array([0, 0, 150]), np.array([180, 80, 255]))
    
    # 빨간색
    mask_red1 = cv2.inRange(hsv, np.array([0, 50, 50]), np.array([10, 255, 255]))
    mask_red2 = cv2.inRange(hsv, np.array([170, 50, 50]), np.array([180, 255, 255]))
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    
    # 노란색
    mask_yellow = cv2.inRange(hsv, np.array([15, 50, 50]), np.array([45, 255, 255]))
    
    # 최종 마스크 (모두 결합)
    mask = cv2.bitwise_or(mask, mask_white)
    mask = cv2.bitwise_or(mask, mask_red)
    mask = cv2.bitwise_or(mask, mask_yellow)
    
    # 노이즈 제거
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # 윤곽선 찾기
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    print(f"  [감지] 전체 윤곽선: {len(contours)}개")
    
    for c in contours:
        area = cv2.contourArea(c)
        
        # 크기 필터링 (더 넓은 범위)
        if 100 < area < 3000:
            x, y, w, h = cv2.boundingRect(c)
            aspect_ratio = float(w) / h
            
            # 정사각형 확인 (더 관대한 범위)
            if 0.6 <= aspect_ratio <= 1.4:
                roi = img[y:y+h, x:x+w]
                center_x = x + w // 2
                center_y = y + h // 2
                
                token_list.append({
                    'x': x,
                    'y': y,
                    'center_x': center_x,
                    'center_y': center_y,
                    'width': w,
                    'height': h,
                    'area': area,
                    'aspect_ratio': aspect_ratio,
                    'roi': roi
                })
                
                print(f"  [토큰] 면적:{area:.0f}, 크기:{w}x{h}, 비율:{aspect_ratio:.2f}")
    
    return token_list

def classify_token(roi):
    """토큰 분류"""
    if roi.shape[0] < 10 or roi.shape[1] < 10:
        return None
    
    h, w = roi.shape[:2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
    avg_brightness = np.mean(gray)
    avg_sat = np.mean(hsv[:, :, 1])
    avg_hue = np.mean(hsv[:, :, 0])
    
    print(f"  [분석] 밝기:{avg_brightness:.1f}, 채도:{avg_sat:.1f}, 색조:{avg_hue:.1f}")
    
    # Letter Victim (흰 배경 + 검은 글자)
    if avg_brightness > 140 and avg_sat < 100:
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
        ratio = np.sum(binary == 255) / binary.size
        
        print(f"  [Letter] 검은픽셀 비율:{ratio:.2f}")
        
        if ratio > 0.35:
            return "H"
        elif ratio > 0.25:
            return "S"
        elif ratio > 0.12:
            return "U"
    
    # Hazmat Signs
    if avg_sat > 80:
        # F: 빨강
        if avg_hue < 15 or avg_hue > 165:
            print(f"  [Hazmat] F (빨강)")
            return "F"
        # O: 노랑 (빨강+노랑)
        elif 15 < avg_hue < 45:
            print(f"  [Hazmat] O (노랑)")
            return "O"
    
    # P: 흰색 배경
    if avg_brightness > 140 and avg_sat < 100:
        print(f"  [Hazmat] P (흰색)")
        return "P"
    
    # C: 검정
    if avg_brightness < 120:
        print(f"  [Hazmat] C (검정)")
        return "C"
    
    print(f"  [Unknown]")
    return None

def is_token_valid(token, camera_width, camera_height):
    """토큰이 유효한 위치에 있는지"""
    margin = 5
    
    if token['x'] < margin or token['y'] < margin:
        return False
    
    if token['x'] + token['width'] > camera_width - margin:
        return False
    
    if token['y'] + token['height'] > camera_height - margin:
        return False
    
    return True

print("=" * 60)
print("개선된 토큰 감지 시스템")
print("=" * 60)

# 상태
STATE_SEARCHING = 0
STATE_APPROACHING = 1
STATE_BACKING = 2
STATE_CENTERING = 3
STATE_IDENTIFYING = 4
STATE_STOPPED = 5
STATE_REPORTING = 6

state = STATE_SEARCHING
timer = 0
backing_timer = 0
current_token = None
identification_attempts = 0
reported_tokens = []

stats = {'H': 0, 'S': 0, 'U': 0, 'F': 0, 'P': 0, 'C': 0, 'O': 0}

lastRequestTime = robot.getTime()

# 거리 설정
TOO_CLOSE = 0.08
SAFE_DISTANCE = 0.12
APPROACH_DISTANCE = 0.20

while robot.step(timeStep) != -1:
    
    # 게임 정보
    if robot.getTime() - lastRequestTime > 1:
        message = struct.pack('c', 'G'.encode())
        emitter.send(message)
        lastRequestTime = robot.getTime()
    
    if receiver.getQueueLength() > 0:
        receivedData = receiver.getBytes()
        if len(receivedData) == 16:
            tup = struct.unpack('c f i i', receivedData)
            if tup[0].decode("utf-8") == 'G':
                print(f'[게임] 점수: {tup[1]:.1f} | 시간: {tup[2]}초')
        receiver.nextPacket()
    
    # 전방 거리
    front_dist = front_sensor.getValue()
    
    img = camera.getImage()
    
    if img:
        tokens = detect_wall_tokens(img, camera)
        
        # 유효한 토큰만 필터링
        camera_width = camera.getWidth()
        camera_height = camera.getHeight()
        valid_tokens = [t for t in tokens if is_token_valid(t, camera_width, camera_height)]
        
        if state == STATE_SEARCHING:
            left_motor.setVelocity(2.0)
            right_motor.setVelocity(2.0)
            
            if valid_tokens:
                token = valid_tokens[0]
                token_id = f"{token['center_x']}_{token['center_y']}"
                
                if token_id not in reported_tokens:
                    print(f"\n{'='*60}")
                    print(f"🔍 토큰 발견!")
                    print(f"   위치: ({token['x']}, {token['y']})")
                    print(f"   중심: ({token['center_x']}, {token['center_y']})")
                    print(f"   면적: {token['area']:.0f}")
                    print(f"   거리: {front_dist:.3f}m")
                    
                    current_token = token
                    reported_tokens.append(token_id)
                    
                    if front_dist < TOO_CLOSE:
                        print("→ 후진 필요")
                        state = STATE_BACKING
                        backing_timer = 0
                    else:
                        print("→ 접근 시작")
                        state = STATE_APPROACHING
        
        elif state == STATE_APPROACHING:
            if front_dist < TOO_CLOSE:
                print(f"⚠️ 너무 가까움 ({front_dist:.3f}m) → 후진")
                state = STATE_BACKING
                backing_timer = 0
                continue
            
            if valid_tokens:
                token = valid_tokens[0]
                current_token = token
                
                center_x = camera_width // 2
                error = token['center_x'] - center_x
                
                if TOO_CLOSE < front_dist < SAFE_DISTANCE:
                    print(f"📍 적정 거리 ({front_dist:.3f}m) → 정렬")
                    state = STATE_CENTERING
                    continue
                
                if front_dist > APPROACH_DISTANCE:
                    speed = 1.0
                elif front_dist > SAFE_DISTANCE:
                    speed = 0.5
                else:
                    speed = 0.3
                
                if abs(error) < 15:
                    left_motor.setVelocity(speed)
                    right_motor.setVelocity(speed)
                elif error > 0:
                    left_motor.setVelocity(speed)
                    right_motor.setVelocity(speed * 0.4)
                else:
                    left_motor.setVelocity(speed * 0.4)
                    right_motor.setVelocity(speed)
            else:
                if tokens:
                    left_motor.setVelocity(0.4)
                    right_motor.setVelocity(0.4)
                else:
                    print("⚠️ 토큰 손실")
                    state = STATE_SEARCHING
        
        elif state == STATE_BACKING:
            left_motor.setVelocity(-0.6)
            right_motor.setVelocity(-0.6)
            backing_timer += 1
            
            if front_dist > SAFE_DISTANCE or backing_timer >= 15:
                print(f"✅ 후진 완료 ({front_dist:.3f}m)")
                state = STATE_CENTERING
                backing_timer = 0
        
        elif state == STATE_CENTERING:
            if front_dist < TOO_CLOSE:
                print(f"⚠️ 다시 가까움 ({front_dist:.3f}m) → 후진")
                state = STATE_BACKING
                backing_timer = 0
                continue
            
            if valid_tokens:
                token = valid_tokens[0]
                current_token = token
                
                center_x = camera_width // 2
                error = token['center_x'] - center_x
                
                if abs(error) < 8:
                    print(f"🎯 정렬 완료 → 식별")
                    state = STATE_IDENTIFYING
                    identification_attempts = 0
                else:
                    if error > 0:
                        left_motor.setVelocity(0.2)
                        right_motor.setVelocity(-0.2)
                    else:
                        left_motor.setVelocity(-0.2)
                        right_motor.setVelocity(0.2)
            else:
                print("⚠️ 정렬 중 토큰 손실")
                state = STATE_SEARCHING
        
        elif state == STATE_IDENTIFYING:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            
            if valid_tokens:
                token = valid_tokens[0]
                token_type = classify_token(token['roi'])
                
                if token_type:
                    print(f"✅ 식별: {token_type}")
                    print(f"   크기: {token['width']}x{token['height']}")
                    print(f"   거리: {front_dist:.3f}m")
                    
                    cv2.imwrite(f"token_{token_type}_{int(robot.getTime())}.png", token['roi'])
                    
                    current_token['type'] = token_type
                    state = STATE_STOPPED
                    timer = 0
                else:
                    identification_attempts += 1
                    
                    if identification_attempts > 20:
                        print("❌ 식별 실패")
                        state = STATE_SEARCHING
                    else:
                        print(f"🔄 시도 {identification_attempts}/20")
            else:
                print("⚠️ 식별 중 토큰 손실")
                state = STATE_SEARCHING
        
        elif state == STATE_STOPPED:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            timer += 1
            
            if timer >= 32:
                state = STATE_REPORTING
        
        elif state == STATE_REPORTING:
            message = struct.pack('c', current_token['type'].encode())
            emitter.send(message)
            
            print(f"📡 리포트: {current_token['type']}")
            
            stats[current_token['type']] += 1
            
            print("✅ 완료!\n")
            
            left_motor.setVelocity(2.0)
            right_motor.setVelocity(2.0)
            
            state = STATE_SEARCHING
            current_token = None
            
            total = sum(stats.values())
            if total > 0 and total % 3 == 0:
                print(f"📊 통계: H={stats['H']} S={stats['S']} U={stats['U']} | F={stats['F']} P={stats['P']} C={stats['C']} O={stats['O']}\n")