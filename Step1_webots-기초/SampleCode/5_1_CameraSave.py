# CameraSave.py
from controller import Robot
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Camera 활성화
camera = robot.getDevice('camera_centre')
camera.enable(timestep)

width = camera.getWidth()
height = camera.getHeight()

saved = False

while robot.step(timestep) != -1 and not saved:
    # 이미지 획득
    image = camera.getImage()
    
    if image:
        # BGRA → RGB 변환  numpy 배열로 변환
        img = np.frombuffer(image, np.uint8)
        # 3차원으로 재구성
        img = img.reshape((height, width, 4))
        # 색상 변환
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        
        # 저장 (C:\Webots\Erebus-v24_1_0\game\controllers\robot0Controller)
        cv2.imwrite('captured.png', img)
        print('이미지 저장 완료!')
        saved = True
