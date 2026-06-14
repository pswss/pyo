"""주기적 능동 heading 재캘리브레이션 트리거 테스트.

근거(실런 계측): passive GPS 보정은 목표 노드가 1~3cm 앞이라 직진이 ≤4cm로 짧아
baseline(8cm) 영영 미달 → 4600프레임 동안 보정 적용 0회 → 자이로 드리프트 누적 → 맵 휘어짐.
해결: explore 중 interval마다 멈추고 초기 캘리브 기동(후진/전진+GPS robust heading)으로
자이로 재동기화. 여기선 트리거 타이밍 로직만 격리 검증(기동 자체는 test_calibration_heading).
"""
import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

_fake = types.ModuleType("controller")
class _Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, n): return _Any()
_fake.Robot = _Any
_fake.__getattr__ = lambda n: _Any
sys.modules["controller"] = _fake

from executor.executor import Executor


def make_executor(interval=20.0):
    ex = Executor.__new__(Executor)
    ex.heading_recalib_interval = interval
    ex._Executor__last_recalib_time = None
    return ex


def test_first_call_initializes_and_does_not_trigger():
    ex = make_executor()
    assert ex.should_recalibrate_heading(5.0) is False   # 타이머 시작점 등록
    assert ex.should_recalibrate_heading(24.9) is False  # 5+20=25 미만


def test_triggers_after_interval():
    ex = make_executor()
    ex.should_recalibrate_heading(5.0)
    assert ex.should_recalibrate_heading(25.1) is True


def test_mark_resets_timer():
    ex = make_executor()
    ex.should_recalibrate_heading(0.0)
    assert ex.should_recalibrate_heading(21.0) is True
    ex.mark_heading_recalibrated(21.0)
    assert ex.should_recalibrate_heading(40.0) is False
    assert ex.should_recalibrate_heading(41.5) is True


if __name__ == "__main__":
    test_first_call_initializes_and_does_not_trigger()
    test_triggers_after_interval()
    test_mark_resets_timer()
    print("OK")
