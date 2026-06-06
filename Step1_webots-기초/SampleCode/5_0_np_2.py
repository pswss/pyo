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
        # numpy 배열로 변환
        img = np.frombuffer(image, np.uint8)
        img = img.reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        
        # 이미지 분석
        print("=== 이미지 분석 ===")
        print(f"크기: {img.shape}")  # (높이, 너비, 채널)
        print(f"데이터 타입: {img.dtype}")  # uint8 (0~255)
        print(f"전체 픽셀 수: {img.shape[0] * img.shape[1]}")
        
        # 색상별 평균값
        blue_avg = np.mean(img[:, :, 0])   # Blue 채널
        green_avg = np.mean(img[:, :, 1])  # Green 채널
        red_avg = np.mean(img[:, :, 2])    # Red 채널
        
        print(f"\n평균 색상 값:")
        print(f"  Blue: {blue_avg:.1f}")
        print(f"  Green: {green_avg:.1f}")
        print(f"  Red: {red_avg:.1f}")
        
        # 가장 많은 색상 판단
        if blue_avg > green_avg and blue_avg > red_avg:
            print("→ 이미지가 전체적으로 파란색이에요!")
        elif green_avg > red_avg:
            print("→ 이미지가 전체적으로 초록색이에요!")
        else:
            print("→ 이미지가 전체적으로 빨간색이에요!")
        
        break