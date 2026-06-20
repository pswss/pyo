"""초기 heading 캘리브레이션 로직 테스트 (원인: world별 시작방향 미보정 → 맵 90° 회전).

__calibrate_heading_from_gps 단위 검증:
- reverse=True  → 측정 heading + π 를 자이로에 세팅 (후진으로 측정)
- reverse=False → 측정 heading + 0 (전진=전방, world1/4 입구 이동)
- GPS heading None → 영점 미설정(__calibrated_heading False 유지)

전체 시퀀스(앞뒤 시도)는 Webots 필요 → 시뮬 검증. 여기선 측정→세팅 변환만 격리.
"""
import os
import sys
import types
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

_fake = types.ModuleType("controller")
class _Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, n): return _Any()
_fake.Robot = _Any
_fake.__getattr__ = lambda n: _Any
sys.modules["controller"] = _fake

from executor.executor import Executor
from data_structures.angle import Angle


class FakeGyro:
    def __init__(self):
        self.set_to = None
    def set_orientation(self, angle):
        self.set_to = angle


class FakeGps:
    def __init__(self, heading):
        self._h = heading
    def get_orientation_robust(self):
        return self._h
    def get_orientation(self):
        return self._h


def make_executor(heading):
    ex = Executor.__new__(Executor)
    ex._Executor__calibrated_heading = False
    gyro = FakeGyro()
    ex.robot = types.SimpleNamespace(gps=FakeGps(heading), gyroscope=gyro)
    return ex, gyro


def test_reverse_adds_pi():
    # 동쪽(예: 90°)으로 후진 측정 → 전방은 +π
    measured = Angle(math.radians(90))
    ex, gyro = make_executor(measured)
    ex._Executor__calibrate_heading_from_gps(reverse=True)
    assert ex._Executor__calibrated_heading is True
    expected = Angle(math.radians(90) + math.pi); expected.normalize()
    assert gyro.set_to.get_absolute_distance_to(expected).degrees < 0.5


def test_forward_no_offset():
    # 전진 측정 → 측정방향이 곧 전방 (world1/4 입구 이동 케이스)
    measured = Angle(math.radians(90))
    ex, gyro = make_executor(measured)
    ex._Executor__calibrate_heading_from_gps(reverse=False)
    assert ex._Executor__calibrated_heading is True
    assert gyro.set_to.get_absolute_distance_to(measured).degrees < 0.5


def test_none_does_not_calibrate():
    ex, gyro = make_executor(None)
    ex._Executor__calibrate_heading_from_gps(reverse=True)
    assert ex._Executor__calibrated_heading is False, "측정 실패 시 보정 안 됨"
    assert gyro.set_to is None, "자이로 미세팅"


if __name__ == "__main__":
    test_reverse_adds_pi()
    test_forward_no_offset()
    test_none_does_not_calibrate()
    print("ALL PASS")
