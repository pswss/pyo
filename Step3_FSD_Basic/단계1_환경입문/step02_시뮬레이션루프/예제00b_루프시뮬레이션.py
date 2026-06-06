"""
[예제 00b] 시뮬레이션 루프 모델
----------------------------------
터미널에서 실행: python 예제00b_루프시뮬레이션.py

Webots의 while do_loop() 구조를 순수 Python으로 모방합니다.
상태기계 + Sequencer의 동작 원리를 눈으로 확인합니다.
"""

import time

TIME_STEP_MS = 32          # 실제 Webots와 동일
TIME_STEP_S  = TIME_STEP_MS / 1000.0
MAX_STEPS    = 200         # 최대 200 스텝 (약 6.4초)

print("=" * 60)
print("  시뮬레이션 루프 동작 원리")
print("=" * 60)

# ── 1. 기본 루프 구조 ─────────────────────────────────────────
print("\n[1] 기본 루프 구조 - while do_loop()")
print("-" * 60)
print("  Webots 실제 코드:")
print("    while self.robot.do_loop():")
print("        self.robot.update()")
print("        self.state_machine.run()")
print()
print("  우리 시뮬레이션:")

step = 0
sim_time = 0.0

def do_loop():
    """Webots do_loop() 모방 - 최대 스텝 수에 도달하면 False"""
    global step, sim_time
    step     += 1
    sim_time += TIME_STEP_S
    return step <= 10   # 10 스텝만 보여주기

while do_loop():
    print(f"    스텝 {step:>3} | 경과 시간: {sim_time:.3f}초 | update() + run() 실행")

print(f"\n  -> {step}스텝 완료. 실제 경기는 약 15,000스텝 (8분)")

# ── 2. 상태기계 동작 시뮬레이션 ──────────────────────────────
print("\n[2] 상태기계 동작 시뮬레이션")
print("-" * 60)

class SimpleStateMachine:
    def __init__(self, initial):
        self.state = initial
        self._states = {}

    def add_state(self, name, fn, transitions=None):
        self._states[name] = (fn, transitions or set())

    def change(self, new_state):
        _, allowed = self._states[self.state]
        if new_state in allowed:
            print(f"    -> 상태 전환: [{self.state}] ──-> [{new_state}]")
            self.state = new_state
        else:
            print(f"    ✗ 전환 불가: {self.state} -> {new_state}")

    def run(self):
        fn, _ = self._states[self.state]
        fn(self.change)


# 가상 환경
env = {
    "step":          0,
    "victim_found":  False,
    "reported":      False,
    "near_start":    False,
}

report_step_count = 0

def state_init(change):
    if env["step"] == 1:
        print(f"    [{env['step']:>3}] [init] GPS 보정 중...")
    elif env["step"] == 2:
        print(f"    [{env['step']:>3}] [init] 360° 스캔...")
        change("explore")

def state_explore(change):
    print(f"    [{env['step']:>3}] [explore] 미로 탐색 중...")
    if env["victim_found"] and not env["reported"]:
        change("report_fixture")
    elif env["near_start"]:
        change("end")

def state_report(change):
    global report_step_count
    report_step_count += 1
    print(f"    [{env['step']:>3}] [report_fixture] 피해자 신고... ({report_step_count}/3)")
    if report_step_count >= 3:
        env["reported"]      = True
        env["victim_found"]  = False
        report_step_count    = 0
        change("explore")

def state_end(change):
    print(f"    [{env['step']:>3}] [end] 지도 전송 & 임무 완료!")

sm = SimpleStateMachine("init")
sm.add_state("init",           state_init,   {"explore"})
sm.add_state("explore",        state_explore, {"report_fixture", "end"})
sm.add_state("report_fixture", state_report,  {"explore"})
sm.add_state("end",            state_end,     set())

for _ in range(15):
    env["step"] += 1
    if env["step"] == 6:
        print(f"\n    [환경 이벤트] 피해자 발견!\n")
        env["victim_found"] = True
    if env["step"] == 13:
        print(f"\n    [환경 이벤트] 출발점 근처!\n")
        env["near_start"] = True

    sm.run()

    if sm.state == "end":
        break

# ── 3. Sequencer 원리 ─────────────────────────────────────────
print("\n[3] Sequencer 동작 원리 - 순차 실행")
print("-" * 60)
print("  매 스텝마다 호출되지만 내부적으로 '어디까지 했나' 기억")
print()

class MiniSequencer:
    def __init__(self):
        self._step = 0
        self._call = 0

    def start(self):
        self._call = 0

    def event(self, fn=None, *args, done_condition=None):
        """현재 스텝이 이 이벤트 차례면 실행, 아니면 건너뜀"""
        if self._call == self._step:
            if done_condition is None:
                if fn:
                    fn(*args)
                self._step += 1
            else:
                if done_condition():
                    self._step += 1
        self._call += 1

    def reset(self):
        self._step = 0


seq = MiniSequencer()
rotate_timer  = 0
delay_timer   = 0
rotate_done   = False
delay_done    = False

def do_rotate():
    global rotate_timer, rotate_done
    if rotate_timer < 3:
        rotate_timer += 1
        print(f"      회전 중... ({rotate_timer}/3)")
        return False
    rotate_done = True
    print("      회전 완료!")
    return True

def do_delay():
    global delay_timer, delay_done
    if delay_timer < 2:
        delay_timer += 1
        print(f"      대기 중... ({delay_timer}/2)")
        return False
    delay_done = True
    print("      대기 완료!")
    return True

def send_signal():
    print("      피해자 신호 전송!")

print("  report_fixture 상태 실행 (총 10 스텝 시뮬레이션):")
print()

for i in range(10):
    print(f"  스텝 {i+1}:")
    seq.start()
    seq.event(print, "      바퀴 정지")
    seq.event(done_condition=do_rotate)
    seq.event(done_condition=do_delay)
    seq.event(send_signal)
    if seq._step >= 4:
        print("      -> 모든 단계 완료! 탐색으로 복귀")
        seq.reset()
        rotate_timer = delay_timer = 0
        break

# ── 4. 타이밍 분석 ───────────────────────────────────────────
print("\n[4] 시간 계산 - 실제 경기 타이밍")
print("-" * 60)

analysis = [
    ("경기 시간",      8 * 60,    "초"),
    ("time_step",     32,         "ms"),
    ("총 스텝 수",    8*60*1000//32, "번"),
    ("지도 전송 시점", (8*60-2)*1000//32, "번째 스텝 (종료 2초 전)"),
    ("A* 재계산 주기", 1,         "스텝마다"),
    ("라이다 갱신",    6,         "스텝마다 (192ms)"),
    ("카메라 갱신",    3,         "스텝마다 (96ms)"),
]

print()
for label, value, unit in analysis:
    print(f"  {label:<20}: {value:>8,} {unit}")

print()
print("  -> executor.py 75번 줄:")
print("    self.max_time_in_run = 8 * 60  # 480초")
print("    check_map_sending(): mapper.time > 480 - 2 -> send_map 상태 전환")
