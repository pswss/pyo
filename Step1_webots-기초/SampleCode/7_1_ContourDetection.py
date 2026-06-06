# ContourDetection.py
from controller import Robot
import cv2
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice('camera_centre')
camera.enable(timestep)

width = camera.getWidth()
height = camera.getHeight()

while robot.step(timestep) != -1:
    image = camera.getImage()
    
    if image:
        # 이미지 변환
        img = np.frombuffer(image, np.uint8)
        img = img.reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        
        # 그레이스케일 변환
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # 블러 적용
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 이진화
        _, binary = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
        
        # 윤곽선 찾기
        contours, _ = cv2.findContours(
            binary,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        print(f'윤곽선 개수: {len(contours)}')
        
        # 윤곽선 그리기
        result = img.copy()
        cv2.drawContours(result, contours, -1, (0, 255, 0), 2)
        
        # 저장
        cv2.imwrite('contours.png', result)
        break
