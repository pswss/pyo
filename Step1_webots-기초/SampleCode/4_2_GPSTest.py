# GPSTest.py
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# GPS 활성화
gps = robot.getDevice('gps')
gps.enable(timestep)

while robot.step(timestep) != -1:
    # 위치 읽기
    pos = gps.getValues()
    x, y, z = pos[0], pos[1], pos[2]
    
    print(f'위치: X={x:.3f}, Y={y:.3f}, Z={z:.3f}')
