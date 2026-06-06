"""
[예제 16] 상태 기계 직접 만들기
----------------------------------
터미널에서 실행: python 예제16_상태기계실습.py

실제 프로젝트의 StateMachine 클래스를 단순화해서
로봇 행동 시뮬레이션을 만들어봅니다.

[학생 과제]
  새로운 상태("recharging")를 추가하고 적절한 전환 규칙을 만들어보세요.
"""

import random
import time

# ── 실제 프로젝트 StateMachine 단순화 버전 ───────────────────
class StateMachine:
    def __init__(self, initial_state):
        self.state            = initial_state
        self.state_functions  = {}
        self.allowed_changes  = {}
        self.possible_states  = set()

    def create_state(self, name, function, possible_changes=None):
        if name in self.possible_states:
            raise ValueError(f"상태 '{name}'가 이미 존재합니다!")
        self.possible_states.add(name)
        self.state_functions[name]  = function
        self.allowed_changes[name]  = possible_changes or set()

    def change_state(self, new_state):
        if new_state not in self.possible_states:
            print(f"  ⚠ 오류: '{new_state}'는 존재하지 않는 상태입니다!")
            return
        if new_state in self.allowed_changes[self.state]:
            print(f"  → 상태 전환: [{self.state}] ──→ [{new_state}]")
            self.state = new_state
        else:
            print(f"  ✗ 전환 불가: [{self.state}]에서 [{new_state}]로 전환 금지!")

    def run(self):
        return self.state_functions[self.state](self.change_state)

# ── 로봇 가상 환경 ────────────────────────────────────────────
class SimRobot:
    def __init__(self):
        self.position      = (0, 0)
        self.victim_found  = False
        self.victim_type   = None
        self.at_start      = False
        self.is_stuck      = False
        self.step_count    = 0

    def update_environment(self):
        """매 스텝 환경 변화 시뮬레이션"""
        self.step_count += 1
        # 랜덤으로 이벤트 발생
        if self.step_count == 5:
            self.victim_found = True
            self.victim_type  = random.choice(["H", "S", "U"])
            print(f"\n  [환경] 피해자 발견! 타입: {self.victim_type}")
        if self.step_count == 12:
            self.is_stuck = True
            print("\n  [환경] 로봇이 갇혔습니다!")
        if self.step_count == 18:
            self.is_stuck = False
            print("\n  [환경] 탈출 성공!")
        if self.step_count >= 25:
            self.at_start = True

# ── 상태별 실행 함수들 ────────────────────────────────────────
robot = SimRobot()
reported_victim = False
stuck_timer     = 0
report_timer    = 0
return_timer    = 0

def state_explore(change_state):
    """탐색 상태: A*로 경로 따라 이동하며 피해자 감지"""
    print(f"  [탐색] 미로 탐색 중... (스텝 {robot.step_count})")

    if robot.is_stuck:
        change_state("stuck")
        return

    if robot.victim_found:
        change_state("report_fixture")
        return

    if robot.step_count >= 20:
        change_state("return_to_start")

def state_report_fixture(change_state):
    """피해자 보고 상태: 심판에게 피해자 위치 전송"""
    global report_timer, reported_victim
    report_timer += 1
    print(f"  [보고] 피해자 보고 중... 타입={robot.victim_type} ({report_timer}/3)")

    if report_timer >= 3:
        print(f"  [보고] ✓ 보고 완료!")
        report_timer = 0
        robot.victim_found = False
        reported_victim = True
        change_state("explore")

def state_stuck(change_state):
    """갇힘 상태: 탈출 동작 실행"""
    global stuck_timer
    stuck_timer += 1
    print(f"  [갇힘] 탈출 시도 중... ({stuck_timer})")

    if not robot.is_stuck:
        stuck_timer = 0
        change_state("explore")
    elif stuck_timer > 5:
        print("  [갇힘] 탈출 실패, 강제 복귀!")
        stuck_timer = 0
        change_state("return_to_start")

def state_return_to_start(change_state):
    """복귀 상태: 출발점으로 돌아가기"""
    global return_timer
    return_timer += 1
    print(f"  [복귀] 출발점으로 이동 중... ({return_timer})")

    if robot.at_start:
        print("  [복귀] ✓ 출발점 도착! 임무 완료!")
        change_state("done")

def state_done(change_state):
    """완료 상태: 임무 종료"""
    print("  [완료] 모든 임무 완료!")
    return "DONE"

# ── 상태 기계 구성 ────────────────────────────────────────────
sm = StateMachine("explore")

sm.create_state("explore",
    function        = state_explore,
    possible_changes = {"report_fixture", "stuck", "return_to_start"})

sm.create_state("report_fixture",
    function        = state_report_fixture,
    possible_changes = {"explore"})

sm.create_state("stuck",
    function        = state_stuck,
    possible_changes = {"explore", "return_to_start"})

sm.create_state("return_to_start",
    function        = state_return_to_start,
    possible_changes = {"done"})

sm.create_state("done",
    function        = state_done,
    possible_changes = set())

# ── 메인 루프 (Webots의 while robot.do_loop() 역할) ──────────
print("=" * 55)
print("  로봇 자율 임무 시뮬레이션")
print("=" * 55)
print(f"\n초기 상태: [{sm.state}]")
print("-" * 55)

for _ in range(35):
    robot.update_environment()
    result = sm.run()
    time.sleep(0.3)  # 시각화를 위해 약간 대기

    if result == "DONE":
        break

print("\n" + "=" * 55)
print("  시뮬레이션 완료!")
print(f"  총 스텝: {robot.step_count}")
print(f"  피해자 보고: {'완료' if reported_victim else '없음'}")

# ── 도전 과제 ─────────────────────────────────────────────────
print("\n[도전 과제]")
print("  1. 배터리 부족 상태('low_battery')를 추가하고")
print("     배터리가 일정 이하면 복귀하도록 만들어보세요")
print("  2. 여러 피해자를 순서대로 보고하는 로직을 추가해보세요")
print("  3. 실제 StateMachine 코드와 비교해보세요:")
print("     src/flow_control/state_machine.py")
