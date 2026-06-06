"""
[예제 01] 바퀴 기초 제어
-----------------------
이 파일을 Erebus 컨트롤러로 로드하면 로봇이 순서대로 움직입니다.

학습 목표:
  - move_wheels(left, right) 로 로봇을 직접 조종한다
  - 타임스텝(time_step)과 루프의 관계를 이해한다
"""

from controller import Robot

# ── 로봇 초기화 ──────────────────────────────────────────────
robot = Robot()
time_step = 32  # 시뮬레이션 한 스텝 = 32ms

# 바퀴 장치 가져오기 (Erebus 로봇의 바퀴 이름)
left_motor  = robot.getDevice("wheel1 motor")
right_motor = robot.getDevice("wheel2 motor")

# 위치 제어 → 속도 제어 모드로 전환
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

MAX_SPEED = 6.28  # 최대 속도 (라디안/초)

def move(left_ratio, right_ratio):
    """비율(-1 ~ 1)로 바퀴를 움직인다."""
    left_motor.setVelocity(left_ratio * MAX_SPEED)
    right_motor.setVelocity(right_ratio * MAX_SPEED)

def stop():
    move(0, 0)

# ── 동작 시퀀스 ───────────────────────────────────────────────
# (스텝 수) × (time_step ms) = 실제 시간
# 예: 50스텝 × 32ms = 1.6초

step = 0

while robot.step(time_step) != -1:
    step += 1

    # 0 ~ 1.6초: 직진
    if step <= 50:
        print(f"[{step:3d}] 직진 중...")
        move(1.0, 1.0)

    # 1.6 ~ 3.2초: 정지
    elif step <= 100:
        print(f"[{step:3d}] 정지")
        stop()

    # 3.2 ~ 4.8초: 후진
    elif step <= 150:
        print(f"[{step:3d}] 후진 중...")
        move(-1.0, -1.0)

    # 4.8 ~ 6.4초: 정지
    elif step <= 200:
        stop()

    # 6.4 ~ 8.0초: 제자리 오른쪽 회전
    elif step <= 250:
        print(f"[{step:3d}] 오른쪽 회전...")
        move(1.0, -1.0)

    # 8.0 ~ 9.6초: 정지
    elif step <= 300:
        stop()

    # 9.6 ~ 11.2초: 왼쪽으로 커브 (부드러운 회전)
    elif step <= 350:
        print(f"[{step:3d}] 왼쪽 커브...")
        move(0.5, 1.0)  # 오른쪽이 더 빠름 → 왼쪽으로 휨

    # 이후: 완전 정지
    else:
        stop()
        print("완료!")
        break
