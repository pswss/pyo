"""
[예제 03] GPS로 위치 읽기
-------------------------
로봇이 움직이는 동안 현재 위치(x, y)를 실시간으로 출력합니다.
이동한 총 거리도 계산해서 보여줍니다.

실행 방법: Erebus에서 이 파일을 컨트롤러로 로드
"""

import math
from controller import Robot

robot = Robot()
time_step = 32
MAX_SPEED  = 6.28

left_motor  = robot.getDevice("wheel1 motor")
right_motor = robot.getDevice("wheel2 motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

gps = robot.getDevice("gps")
gps.enable(time_step)

def move(left_ratio, right_ratio):
    left_motor.setVelocity(left_ratio * MAX_SPEED)
    right_motor.setVelocity(right_ratio * MAX_SPEED)

def get_position():
    """GPS에서 (x, y) 위치를 읽어온다."""
    values = gps.getValues()
    # Webots GPS: [x, y, z] → 우리는 x, z를 2D 위치로 사용
    return values[0], values[2]

def distance(x1, y1, x2, y2):
    """두 점 사이의 거리 (피타고라스 정리)"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# ── 메인 루프 ─────────────────────────────────────────────────
step = 0
total_distance = 0.0
prev_x, prev_y = None, None

print("=" * 50)
print("  스텝  |    X     |    Y     | 이동거리(누적)")
print("=" * 50)

while robot.step(time_step) != -1:
    step += 1

    # 현재 위치 읽기
    cur_x, cur_y = get_position()

    # 누적 이동 거리 계산
    if prev_x is not None:
        d = distance(prev_x, prev_y, cur_x, cur_y)
        total_distance += d

    prev_x, prev_y = cur_x, cur_y

    # 10스텝마다 출력 (너무 많이 출력되지 않도록)
    if step % 10 == 0:
        print(f"  {step:4d}  | {cur_x:+.4f} | {cur_y:+.4f} | {total_distance:.4f}m")

    # 0~3초: 직진
    if step <= 100:
        move(1.0, 1.0)
    # 3~5초: 오른쪽 회전
    elif step <= 150:
        move(1.0, -1.0)
    # 5~8초: 직진
    elif step <= 250:
        move(1.0, 1.0)
    # 이후: 정지
    else:
        move(0, 0)
        if step == 251:
            print("=" * 50)
            print(f"최종 위치: ({cur_x:.4f}, {cur_y:.4f})")
            print(f"총 이동 거리: {total_distance:.4f}m")
