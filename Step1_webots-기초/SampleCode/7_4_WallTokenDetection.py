# WallTokenDetection.py
from controller import Robot
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 카메라 초기화
camera = robot.getDevice('camera_centre')
camera.enable(timestep)
width = camera.getWidth()
height = camera.getHeight()

def detect_wall_token(image):
    """벽 토큰 감지 함수"""
    # 그레이스케일 변환
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # 블러 적용 (노이즈 제거)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # 이진화 (밝은 토큰 감지용)
    # THRESH_BINARY_INV: 밝은 부분을 검정으로, 어두운 부분을 흰색으로
    _, binary = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY_INV)
    
    # 윤곽선 찾기
    contours, _ = cv2.findContours(
        binary, 
        cv2.RETR_EXTERNAL, 
        cv2.CHAIN_APPROX_SIMPLE
    )
    
    detected_tokens = []
    
    for contour in contours:
        # 면적 계산
        area = cv2.contourArea(contour)
        
        # 크기 필터링 (토큰 크기 범위)
        # 0.12m × 0.12m 토큰을 픽셀로 환산
        if 200 < area < 2000:
            # Bounding Box
            x, y, w, h = cv2.boundingRect(contour)
            
            # 종횡비 계산
            aspect_ratio = float(w) / h
            
            # 정사각형 모양 확인
            if 0.8 <= aspect_ratio <= 1.2:
                # 토큰 영역 추출
                roi = image[y:y+h, x:x+w]
                
                # 토큰 정보 저장
                token_info = {
                    'position': (x, y),
                    'size': (w, h),
                    'area': area,
                    'aspect_ratio': aspect_ratio,
                    'roi': roi
                }
                detected_tokens.append(token_info)
    
    if detected_tokens:
        return True, detected_tokens
    else:
        return False, None

# 메인 루프
frame_count = 0

while robot.step(timestep) != -1:
    image = camera.getImage()
    
    if image:
        # 이미지 변환
        img = np.frombuffer(image, np.uint8)
        img = img.reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        
        # 토큰 감지
        found, tokens = detect_wall_token(img)
        
        # 결과 이미지
        result = img.copy()
        
        if found:
            print(f'\n=== 프레임 {frame_count} ===')
            print(f'🎯 토큰 발견: {len(tokens)}개')
            
            for i, token in enumerate(tokens):
                x, y = token['position']
                w, h = token['size']
                area = token['area']
                ratio = token['aspect_ratio']
                
                # 초록색 사각형으로 표시
                cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # 라벨 표시
                label = f'Token {i+1}'
                cv2.putText(result, label, (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # 토큰 영역 저장
                cv2.imwrite(f'token_{frame_count}_{i+1}.png', token['roi'])
                
                # 정보 출력
                print(f'\n토큰 {i+1}:')
                print(f'  위치: ({x}, {y})')
                print(f'  크기: {w}x{h}')
                print(f'  면적: {area:.0f}')
                print(f'  종횡비: {ratio:.2f}')
        else:
            print(f'프레임 {frame_count}: 토큰 없음')