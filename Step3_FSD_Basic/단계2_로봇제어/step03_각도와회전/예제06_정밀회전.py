"""
[예제 06] 자이로스코프로 정밀 회전
-----------------------------------
자이로스코프(각속도 센서)를 이용해 정확히 원하는 각도만큼 회전합니다.
타이머 기반(예제02)보다 훨씬 정확합니다!

핵심 개념:
  - 자이로스코프: 매 순간 얼마나 빠르게 회전하는지 측정
  - 각속도 × 시간 = 회전한 각도 (적분)
  - P 제어: 오차가 클수록 빠르게, 작을수록 천천히 회전

[학생 과제]
  TARGET_ANGLE_DEG 값을 바꿔서 다양한 각도로 회전해보세요!
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

gyro = robot.getDevice("gyro")
gyro.enable(time_step)

# ── 설정값 ────────────────────────────────────────────────────
TARGET_ANGLE_DEG = 90.0   # 목표 회전 각도 (도 단위) — 바꿔보세요!
TARGET_ANGLE_RAD = math.radians(TARGET_ANGLE_DEG)

KP = 1.5   # 비례 제어 계수 (크면 빠르지만 오버슈트 발생)
MIN_SPEED = 0.2   # 최소 회전 속도 (너무 느리면 안 돌아감)
MAX_TURN  = 1.0   # 최대 회전 속도

DONE_THRESHOLD = math.radians(2.0)  # 2도 이내면 완료

# ── 헬퍼 함수 ─────────────────────────────────────────────────
def move(left_ratio, right_ratio):
    left_motor.setVelocity(left_ratio * MAX_SPEED)
    right_motor.setVelocity(right_ratio * MAX_SPEED)

def get_angular_velocity():
    """자이로스코프에서 y축 각속도(라디안/초) 반환"""
    return gyro.getValues()[1]

# ── 메인 루프 ─────────────────────────────────────────────────
accumulated_angle = 0.0   # 지금까지 회전한 각도 누적
dt = time_step / 1000.0   # 시간 간격 (초 단위)

print(f"목표: {TARGET_ANGLE_DEG}도 회전")
print(f"시작!")

phase = "rotate"

while robot.step(time_step) != -1:

    if phase == "rotate":
        omega = get_angular_velocity()          # 현재 각속도 (라디안/초)
        accumulated_angle += omega * dt         # 적분: 각도 누적

        remaining = TARGET_ANGLE_RAD - accumulated_angle  # 남은 각도

        # 완료 체크
        if abs(remaining) < DONE_THRESHOLD:
            move(0, 0)
            phase = "done"
            print(f"✓ 완료! 실제 회전: {math.degrees(accumulated_angle):.2f}도 "
                  f"(목표: {TARGET_ANGLE_DEG}도, "
                  f"오차: {abs(math.degrees(remaining)):.2f}도)")
            continue

        # P 제어: 남은 각도에 비례해서 속도 결정
        speed = KP * abs(remaining)
        speed = max(MIN_SPEED, min(MAX_TURN, speed))  # 범위 제한

        if remaining > 0:
            move(-speed, speed)   # 반시계 회전
        else:
            move(speed, -speed)   # 시계 회전

        # 매 10스텝마다 진행상황 출력
        if int(accumulated_angle * 100) % 50 == 0:
            print(f"  현재: {math.degrees(accumulated_angle):6.1f}도 | "
                  f"남은: {math.degrees(remaining):6.1f}도 | "
                  f"속도: {speed:.2f}")

    elif phase == "done":
        move(0, 0)

# ── 도전 과제 ─────────────────────────────────────────────────
# 1. KP 값을 0.5, 1.0, 2.0으로 바꿔보고 차이를 관찰하세요
# 2. 45도 → 90도 → 180도 → 360도 순서로 회전하도록 수정하세요
# 3. 예제02의 타이머 방식과 정확도를 비교해보세요
