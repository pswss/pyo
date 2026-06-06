# WallTokenDetection_Fixed.py
from controller import Robot
import cv2
import numpy as np

def detect_wall_tokens(image_data, camera):
    """벽 토큰 감지 - 수정 버전"""
    token_list = []
    
    # 이미지 변환
    img = np.array(np.frombuffer(image_data, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)))
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    
    # HSV 변환
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # 흰색/밝은 색 감지
    lower_white = np.array([0, 0, 180])
    upper_white = np.array([180, 50, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    
    # 빨간색 감지
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    
    # 분홍색 감지
    mask_pink = cv2.inRange(hsv, np.array([140, 50, 50]), np.array([170, 255, 255]))
    
    # 모든 마스크 합치기
    mask = cv2.bitwise_or(mask_red, mask_white)
    mask = cv2.bitwise_or(mask, mask_pink)
    
    # 윤곽선 찾기
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for c in contours:
        area = cv2.contourArea(c)
        
        # 크기 필터링
        if 200 < area < 2000:
            x, y, w, h = cv2.boundingRect(c)
            aspect_ratio = float(w) / h
            
            # 정사각형 확인
            if 0.8 <= aspect_ratio <= 1.2:
                token_info = {
                    'x': x,
                    'y': y,
                    'width': w,
                    'height': h,
                    'area': area
                }
                token_list.append(token_info)
    
    return token_list

# 로봇 초기화
robot = Robot()
timeStep = 32

camera = robot.getDevice("camera_centre")
camera.enable(timeStep)

# Emitter 초기화 (게임 매니저 통신용)
emitter = robot.getDevice("emitter")

print("벽 토큰 감지 시작...")

# 메인 루프 (계속 실행)
while robot.step(timeStep) != -1:
    img = camera.getImage()
    
    if img:
        tokens = detect_wall_tokens(img, camera)
        
        # 토큰 발견 시에만 출력
        if tokens:
            print(f"\n🎯 토큰 {len(tokens)}개 발견!")
            
            for i, token in enumerate(tokens):
                print(f"  {i+1}. 위치: ({token['x']}, {token['y']}) "
                      f"크기: {token['width']}x{token['height']} "
                      f"면적: {token['area']:.0f}")