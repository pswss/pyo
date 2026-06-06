# ShapeDetection_Minimal.py
from controller import Robot
import numpy as np
import cv2

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
        img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        
        # 전처리
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        
        # 윤곽선 찾기
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 도형 분류
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100: continue
            
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            vertices = len(approx)
            
            if vertices == 3:
                shape = 'Triangle'
            elif vertices == 4:
                shape = 'Rectangle'
            elif vertices > 8:
                shape = 'Circle'
            else:
                shape = 'Polygon'
            
            print(f'{shape}: {vertices}개 꼭짓점, 면적: {area:.0f}')
        
        break