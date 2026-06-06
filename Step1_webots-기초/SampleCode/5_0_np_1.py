from controller import Robot
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice('camera_centre')
camera.enable(timestep)
width = camera.getWidth()   # 예: 64
height = camera.getHeight() # 예: 64

while robot.step(timestep) != -1:
    image = camera.getImage()
    
    if image:
        # numpy 배열로 변환
        img = np.frombuffer(image, np.uint8)
        print("1차원 배열:", img.shape)  # (16384,) = 64*64*4
        
        # 3차원으로 재구성
        img = img.reshape((height, width, 4))
        print("3차원 배열:", img.shape)  # (64, 64, 4)
        
        # 색상 변환
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        print("BGR 배열:", img.shape)    # (64, 64, 3)
        
        # 특정 픽셀 값 확인
        pixel = img[32, 32]  # 중앙 픽셀
        print(f"중앙 픽셀 색상 (BGR): {pixel}")
        
        break