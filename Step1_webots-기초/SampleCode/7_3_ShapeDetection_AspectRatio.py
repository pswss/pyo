# ShapeDetection_AspectRatio_Minimal.py
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
        img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100: continue
            
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            vertices = len(approx)
            
            # Bounding Box 구하기
            x, y, w, h = cv2.boundingRect(contour)
            
            # 종횡비
            aspect_ratio = float(w) / h
            
            # 도형 판별
            if vertices == 3:
                shape = 'Triangle'
            elif vertices == 4:
                # 정사각형 판별
                if 0.95 <= aspect_ratio <= 1.05:
                    shape = 'Square'
                else:
                    shape = 'Rectangle'
            elif vertices > 8:
                shape = 'Circle'
            else:
                shape = 'Polygon'
            
            print(f'{shape}: 종횡비={aspect_ratio:.2f}, 크기={w}x{h}')
        
        break