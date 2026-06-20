"""LoP(끼임) 자동 탈출 배선 테스트.

버그: Executor가 stuck_detector.is_stuck()를 한 번도 호출하지 않아
'stuck' 상태로 전환되는 트리거가 존재하지 않았다(LoP -5점 자동 탈출 미작동).

이 테스트는 무거운 Webots 의존성을 피하기 위해 Executor.__new__로 인스턴스를
만들고, 실제 StateMachine / StuckDetector / Sequencer를 수동 배선하여
check_stuck()의 동작만 격리 검증한다.
"""
import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

# Webots 전용 'controller' 모듈을 stub으로 대체 (시뮬레이터 밖에서 import 가능하게)
_fake_controller = types.ModuleType("controller")
class _Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, n): return _Any()
_fake_controller.Robot = _Any
_fake_controller.__getattr__ = lambda name: _Any
sys.modules["controller"] = _fake_controller

from executor.executor import Executor
from executor.stuck_detector import StuckDetector
from flow_control.state_machine import StateMachine
from flow_control.sequencer import Sequencer
from data_structures.vectors import Position2D


def make_executor(initial_state):
    """__init__을 우회하고 check_stuck이 의존하는 협력자만 실제 객체로 배선한다."""
    ex = Executor.__new__(Executor)
    sm = StateMachine(initial_state)
    sm.create_state("explore", lambda *a: None, {"stuck"})
    sm.create_state("stuck", lambda *a: None, {"explore"})
    ex.state_machine = sm
    ex.stuck_detector = StuckDetector()
    ex.sequencer = Sequencer(reset_function=lambda: None)
    ex.consecutive_stuck_escapes = 0
    ex.max_stuck_escapes = 3
    ex.last_stuck_position = None
    ex.stuck_same_spot_radius = 0.15
    # LoP 신고 호출을 기록하는 가짜 comunicator
    lop = types.SimpleNamespace(calls=0)
    def _send_lop():
        lop.calls += 1
    # check_stuck의 로그/통신이 참조하는 robot 협력자 (position은 거리계산 위해 실제 Position2D)
    ex.robot = types.SimpleNamespace(
        position=Position2D(0.0, 0.0), time=0.0,
        comunicator=types.SimpleNamespace(send_lack_of_progress=_send_lop))
    ex._lop = lop  # 테스트 접근용
    return ex


def test_stuck_triggers_from_explore():
    ex = make_executor("explore")
    ex.stuck_detector.stuck_counter = 100  # is_stuck() True
    ex.check_stuck()
    assert ex.state_machine.state == "stuck", \
        f"explore에서 끼임 시 stuck으로 전환돼야 함, 실제={ex.state_machine.state}"


def test_no_transition_when_not_stuck():
    ex = make_executor("explore")
    ex.stuck_detector.stuck_counter = 0  # is_stuck() False
    ex.check_stuck()
    assert ex.state_machine.state == "explore", \
        f"끼이지 않았으면 explore 유지해야 함, 실제={ex.state_machine.state}"


def test_ignored_outside_explore():
    # 'stuck'은 explore에서만 진입 가능하므로 다른 상태에서는 발동하면 안 됨
    ex = make_executor("stuck")
    ex.stuck_detector.stuck_counter = 100  # is_stuck() True
    ex.check_stuck()
    assert ex.state_machine.state == "stuck", \
        f"explore가 아닌 상태에서는 전환 시도하지 않아야 함, 실제={ex.state_machine.state}"


def test_lop_request_after_repeated_wiggle_failures():
    # wiggle을 max_stuck_escapes회 반복해도 끼임이 안 풀리면 LoP 신고(순간이동)
    ex = make_executor("explore")
    ex.stuck_detector.stuck_counter = 100  # 계속 끼인 상태 유지(탈출 실패 모사)

    # 1, 2회차: wiggle 탈출 시도 → stuck 전환, LoP 신고 없음
    # (stuck 전환 시 감지기 reset되므로, 복귀 후 '여전히 끼임'은 재감지로 모사)
    for attempt in (1, 2):
        ex.check_stuck()
        assert ex.state_machine.state == "stuck", f"{attempt}회차는 wiggle(stuck)이어야 함"
        assert ex._lop.calls == 0, f"{attempt}회차에 LoP 신고하면 안 됨"
        ex.state_machine.state = "explore"  # 탈출 시퀀스 종료 후 explore 복귀 모사
        ex.stuck_detector.stuck_counter = 100  # 탈출 실패: 끼임 재감지 모사

    # 3회차: 임계 도달 → LoP 신고, 상태 전환 없음(게임매니저가 순간이동)
    ex.check_stuck()
    assert ex._lop.calls == 1, f"3회 실패 후 LoP 신고 1회 발생해야 함, 실제={ex._lop.calls}"
    assert ex.state_machine.state == "explore", "LoP 신고 시 stuck 전환은 하지 않아야 함"
    assert ex.consecutive_stuck_escapes == 0, "LoP 신고 후 카운터 리셋돼야 함"


def test_far_stuck_starts_fresh_count():
    # 한 곳에서 끼였다 나온 뒤 '멀리 떨어진' 새 위치에서 끼이면 연속 카운트는 1로 새로
    # 시작해야 한다(서로 다른 deadlock이 합산돼 조기 LoP 되는 것 방지). wiggle은 거의
    # 원위치라 '이동했으니 리셋'은 같은 자리 한계루프를 못 끊으므로, 리셋 기준은 '거리'다.
    ex = make_executor("explore")
    ex.stuck_detector.stuck_counter = 100
    ex.check_stuck()                       # A(0,0)에서 끼임 → consecutive 1, stuck
    assert ex.consecutive_stuck_escapes == 1
    ex.state_machine.state = "explore"     # 복귀
    ex.robot.position = Position2D(1.0, 1.0)  # 멀리 이동 (반경 0.15m 초과)
    ex.stuck_detector.stuck_counter = 100  # 새 위치 B에서 끼임
    ex.check_stuck()
    assert ex.consecutive_stuck_escapes == 1, "멀리서 새로 끼이면 카운트 1로 시작해야 함"
    assert ex._lop.calls == 0, "서로 다른 위치 끼임은 합산되지 않아 LoP 없어야 함"


def test_same_spot_repeats_escalate_to_lop():
    # 같은 자리에서 wiggle이 계속 실패(원위치 복귀 반복)하면 3회째 LoP로 탈출
    ex = make_executor("explore")
    for _ in (1, 2):
        ex.stuck_detector.stuck_counter = 100
        ex.check_stuck()
        assert ex.state_machine.state == "stuck"
        assert ex._lop.calls == 0
        ex.state_machine.state = "explore"   # 같은 위치(0,0) 유지 → 같은 자리 재끼임
    ex.stuck_detector.stuck_counter = 100
    ex.check_stuck()
    assert ex._lop.calls == 1, "같은 자리 3회 실패 → LoP 신고"


if __name__ == "__main__":
    test_stuck_triggers_from_explore()
    test_no_transition_when_not_stuck()
    test_ignored_outside_explore()
    test_lop_request_after_repeated_wiggle_failures()
    test_far_stuck_starts_fresh_count()
    test_same_spot_repeats_escalate_to_lop()
    print("ALL PASS")
