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

def detect_blue(img):
    """파란색 감지 함수"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # 파란색 범위
    # lower_blue = np.array([100, 100, 100])
    # upper_blue = np.array([130, 255, 255])
    
    
    # 짙은 파란색 범위 - 채도와 명도 최솟값을 높임
    # lower_blue = np.array([100, 150, 50])   # 채도 150 이상, 명도 50 이상
    # upper_blue = np.array([130, 255, 200])  # 명도 200 이하 (너무 밝은 하늘색 제외)
    
    lower_blue = np.array([110, 200, 180])  # 선명하고 밝은 파란색
    upper_blue = np.array([125, 255, 255])
    
    # 마스크 생성
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    return mask

def find_blue_object(mask):
    """파란색 객체의 중심과 크기 찾기"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                   cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # 가장 큰 파란색 객체 선택
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        # 중심점 계산
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy), area
    
    return None, 0

# 메인 루프
frame_count = 0
while robot.step(timestep) != -1:
    image = camera.getImage()
    
    if image:
        # 이미지 변환
        img = np.frombuffer(image, np.uint8)
        img = img.reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        
        # 파란색 감지
        blue_mask = detect_blue(img)
        center, area = find_blue_object(blue_mask)
        
        if center:
            cx, cy = center
            print(f'프레임 {frame_count}: 파란색 발견! 위치=({cx}, {cy}), 크기={area}')
            
            # 화면 중앙 기준으로 방향 판단
            center_x = width // 2
            if cx < center_x - 20:
                print('  → 왼쪽에 있음')
            elif cx > center_x + 20:
                print('  → 오른쪽에 있음')
            else:
                print('  → 중앙에 있음')
        else:
            print(f'프레임 {frame_count}: 파란색 객체 없음')
        
        frame_count += 1
        
        # 100프레임마다 결과 이미지 저장
        if frame_count % 100 == 0:
            cv2.imwrite(f'blue_detection_{frame_count}.png', blue_mask)