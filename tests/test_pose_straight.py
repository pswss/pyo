"""robot_is_going_straight 비대칭 버그 회귀 테스트.

버그(매핑 깨짐 원인 A): get_wheel_velocity_difference()는 right-left라 부호가 있다.
우회전 시 음수 → `wheel_velocity_difference < threshold`가 항상 True → 우회전을
'직진'으로 오판 → 회전 중 GPS chord 방향을 heading으로 읽어 자이로 손상 → 맵 회전/깨짐.
수정: abs()로 좌/우 대칭 처리.
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

from robot.pose_manager import PoseManager
from data_structures.angle import Angle


class FakeGyro:
    def get_angular_velocity(self):
        return Angle(0)  # 회전 없음 (각속도 0)


def make_pm():
    return PoseManager(gps=object(), gyroscope=FakeGyro())


def test_straight_is_straight():
    pm = make_pm()
    assert pm.robot_is_going_straight(6.0, 0.1) is True, "양 바퀴 거의 같으면 직진"


def test_right_turn_not_straight():
    # 우회전: right < left → diff = right-left < 0. abs 없으면 < threshold라 직진 오판됨.
    pm = make_pm()
    assert pm.robot_is_going_straight(6.0, -3.0) is False, \
        "우회전(음수 diff)은 직진이 아니어야 함 (비대칭 버그)"


def test_left_turn_not_straight():
    pm = make_pm()
    assert pm.robot_is_going_straight(6.0, 3.0) is False, "좌회전(양수 diff)은 직진이 아니어야 함"


if __name__ == "__main__":
    test_straight_is_straight()
    test_right_turn_not_straight()
    test_left_turn_not_straight()
    print("ALL PASS")
