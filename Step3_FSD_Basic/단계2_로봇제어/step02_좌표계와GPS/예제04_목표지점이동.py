"""
[예제 04] 목표 좌표로 이동하기
-------------------------------
GPS로 현재 위치를 확인하면서 목표 좌표까지 이동합니다.

핵심 개념:
  1. 현재 위치와 목표 위치의 방향각 계산 (atan2)
  2. 로봇이 그 방향을 바라보도록 회전
  3. 목표에 가까워지면 정지

[학생 과제]
  TARGETS 리스트에 목표 좌표를 추가해보세요!
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

compass = robot.getDevice("compass")
compass.enable(time_step)

# ── 설정값 ────────────────────────────────────────────────────
# 방문할 목표 좌표 목록 (x, y) — 직접 바꿔보세요!
TARGETS = [
    (0.3,  0.0),
    (0.3,  0.3),
    (0.0,  0.3),
    (0.0,  0.0),   # 출발점으로 복귀
]

ARRIVAL_THRESHOLD = 0.05   # 5cm 이내면 도착으로 판정
TURN_SPEED        = 0.5    # 회전 속도
MOVE_SPEED        = 0.8    # 직진 속도
ANGLE_THRESHOLD   = 0.1    # 방향 오차 허용 범위 (라디안)

# ── 헬퍼 함수 ─────────────────────────────────────────────────
def get_position():
    v = gps.getValues()
    return v[0], v[2]

def get_heading():
    """나침반으로 현재 방향각(라디안) 반환. 0 = 북쪽(+y)"""
    north = compass.getValues()
    return math.atan2(north[0], north[2])

def angle_to_target(cur_x, cur_y, tgt_x, tgt_y):
    """현재 위치에서 목표 방향의 각도(라디안) 계산"""
    return math.atan2(tgt_x - cur_x, tgt_y - cur_y)

def angle_diff(a, b):
    """두 각도의 차이 (-π ~ π 범위로 정규화)"""
    diff = a - b
    while diff >  math.pi: diff -= 2 * math.pi
    while diff < -math.pi: diff += 2 * math.pi
    return diff

def dist(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

def move(left_ratio, right_ratio):
    left_motor.setVelocity(left_ratio * MAX_SPEED)
    right_motor.setVelocity(right_ratio * MAX_SPEED)

# ── 메인 루프 ─────────────────────────────────────────────────
target_index = 0

print(f"목표 좌표: {TARGETS}")
print(f"첫 번째 목표: {TARGETS[0]}")

while robot.step(time_step) != -1:
    if target_index >= len(TARGETS):
        move(0, 0)
        print("모든 목표 도착 완료!")
        break

    cur_x, cur_y   = get_position()
    tgt_x, tgt_y   = TARGETS[target_index]
    heading         = get_heading()
    target_angle    = angle_to_target(cur_x, cur_y, tgt_x, tgt_y)
    d               = dist(cur_x, cur_y, tgt_x, tgt_y)
    err             = angle_diff(target_angle, heading)

    # 도착 판정
    if d < ARRIVAL_THRESHOLD:
        print(f"✓ 목표 {target_index+1} 도착! 위치=({cur_x:.3f}, {cur_y:.3f})")
        target_index += 1
        move(0, 0)
        continue

    # 방향이 맞으면 직진, 아니면 회전
    if abs(err) < ANGLE_THRESHOLD:
        # 방향 맞음 → 직진
        move(MOVE_SPEED, MOVE_SPEED)
    elif err > 0:
        # 왼쪽으로 돌아야 함
        move(-TURN_SPEED, TURN_SPEED)
    else:
        # 오른쪽으로 돌아야 함
        move(TURN_SPEED, -TURN_SPEED)

# ── 도전 과제 ─────────────────────────────────────────────────
# 1. TARGETS에 좌표를 더 추가해서 복잡한 경로 만들기
# 2. 목표에 가까울수록 속도를 줄이는 코드 추가하기
#    힌트: speed = min(MOVE_SPEED, d * 10)
# 3. ARRIVAL_THRESHOLD를 0.02로 줄이면 어떻게 되나요?
