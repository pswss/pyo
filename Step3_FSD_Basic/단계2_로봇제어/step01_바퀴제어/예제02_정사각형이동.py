"""
[예제 02] 정사각형 이동 — 미니 프로젝트
----------------------------------------
로봇이 정사각형 경로로 한 바퀴 돌도록 완성하세요.

정사각형 = 직진 → 90도 회전 → 직진 → 90도 회전 → (×4 반복)

[학생 과제]
  TURN_STEPS 값을 바꿔가며 정확히 90도 회전하는 값을 찾아보세요!
  힌트: 너무 작으면 덜 돌고, 너무 크면 더 돌아갑니다.
"""

from controller import Robot

robot = Robot()
time_step = 32
MAX_SPEED = 6.28

left_motor  = robot.getDevice("wheel1 motor")
right_motor = robot.getDevice("wheel2 motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

def move(left_ratio, right_ratio):
    left_motor.setVelocity(left_ratio * MAX_SPEED)
    right_motor.setVelocity(right_ratio * MAX_SPEED)

def stop():
    move(0, 0)

# ── 여기를 수정해보세요! ──────────────────────────────────────
FORWARD_STEPS = 80   # 직진 시간 (스텝 수) — 얼마나 앞으로 갈지
TURN_STEPS    = 30   # 회전 시간 (스텝 수) — 여기를 수정해서 90도 맞추기!
PAUSE_STEPS   = 15   # 동작 사이 잠깐 멈춤
# ────────────────────────────────────────────────────────────

# 동작 순서 정의: (동작이름, 스텝수, left비율, right비율)
sequence = []
for i in range(4):  # 4번 반복 = 정사각형 4변
    sequence.append(("직진",   FORWARD_STEPS,  1.0,  1.0))
    sequence.append(("정지",   PAUSE_STEPS,    0.0,  0.0))
    sequence.append(("우회전", TURN_STEPS,     1.0, -1.0))  # 오른쪽 회전
    sequence.append(("정지",   PAUSE_STEPS,    0.0,  0.0))

sequence.append(("완료", 1, 0.0, 0.0))

# ── 메인 루프 ─────────────────────────────────────────────────
action_index = 0
action_step  = 0

while robot.step(time_step) != -1:
    if action_index >= len(sequence):
        stop()
        break

    name, total_steps, left, right = sequence[action_index]
    move(left, right)

    action_step += 1
    if action_step == 1:
        print(f"▶ {name} 시작 (총 {total_steps}스텝)")

    if action_step >= total_steps:
        action_index += 1
        action_step = 0

# ── 도전 과제 ─────────────────────────────────────────────────
# 1. TURN_STEPS를 조정해서 정확히 90도 회전하게 만들기
# 2. 정사각형이 아닌 삼각형(120도 회전 × 3)으로 바꿔보기
# 3. 속도를 절반으로 줄이면 TURN_STEPS는 어떻게 바꿔야 할까?
