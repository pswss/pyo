"""벽 거리유지 가드 테스트 (벽에 박는 문제 / 픽스처 촬영 시 벽 붙음 방지).

SmoothMovementToCoordinatesManager는 목표를 향해 전진하다 벽에 박는다.
front_blocked(전방 라이다 근접)일 때 전진 모드면 전진을 금지(정렬됐으면 정지,
약간 틀어졌으면 제자리 회전으로 방향만)하는지 검증. 제자리 회전(>45°)은 원래 안전.
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

from robot.drive_base import SmoothMovementToCoordinatesManager
from data_structures.vectors import Position2D


class FakeWheel:
    def __init__(self):
        self.last = None
    def move(self, v):
        self.last = v


def make_manager(front_blocked):
    l, r = FakeWheel(), FakeWheel()
    m = SmoothMovementToCoordinatesManager(l, r)
    m.current_position = Position2D(0, 0)
    m.front_blocked = front_blocked
    return m, l, r


def test_blocked_and_aligned_stops_no_forward():
    # 목표 정면 + 정렬 + 전방 막힘 → 전진 금지(정지). 안 막혔으면 풀전진이던 자리.
    target = Position2D(0, 1)
    m, l, r = make_manager(front_blocked=True)
    m.current_angle = Position2D(0, 0).get_angle_to(target)
    m.move_to_position(target)
    assert not (l.last > 0 and r.last > 0), "전방 막힘+정렬이면 양 바퀴 전진 금지(벽에 박지 말 것)"
    assert l.last == 0 and r.last == 0, "정렬+막힘은 정지(거리 유지)"


def test_not_blocked_aligned_drives_forward():
    # 안 막혔으면 정상 직진(회귀 방지).
    target = Position2D(0, 1)
    m, l, r = make_manager(front_blocked=False)
    m.current_angle = Position2D(0, 0).get_angle_to(target)
    m.move_to_position(target)
    assert l.last > 0 and r.last > 0, "안 막혔으면 정렬 시 전진해야 함"


if __name__ == "__main__":
    test_blocked_and_aligned_stops_no_forward()
    test_not_blocked_aligned_drives_forward()
    print("ALL PASS")
