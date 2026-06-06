"""GPS baseline 누적-회전 리셋 테스트 (area 넘어가며 누적 heading 드리프트 → 구역 회전).

완만한 곡선이 매-스텝 직진판정을 통과하면 GPS baseline이 곡선을 가로질러 chord 오측정.
decide_orientation_sensor가 baseline 이후 누적 회전 > 8°면 baseline을 리셋하는지 검증.
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

from robot.pose_manager import PoseManager
from data_structures.angle import Angle


class FakeGyro:
    def __init__(self, deg):
        self._o = Angle(math.radians(deg))
    def get_orientation(self):
        return self._o
    def get_angular_velocity(self):
        return Angle(0)  # 매-스텝 회전 없음 → robot_is_going_straight의 각속도 조건 통과


class FakeGps:
    def __init__(self):
        self.reset_count = 0
    def reset_orientation_baseline(self):
        self.reset_count += 1


def make_pm(gyro_deg):
    return PoseManager(gps=FakeGps(), gyroscope=FakeGyro(gyro_deg))


def test_resets_baseline_on_large_cumulative_turn():
    # baseline(0°) 이후 자이로가 20° 회전 → 8° 초과 → 직진 중이어도 baseline 리셋
    pm = make_pm(gyro_deg=20)
    pm.decide_orientation_sensor(6.0, 0.1)  # 직진 조건
    assert pm.gps.reset_count >= 1, "누적 회전 20° > 8°면 baseline 리셋해야 함"


def test_no_reset_on_small_cumulative_turn():
    # 3° < 8° + 직진 지속 → 리셋 없음. 단 GPS는 연속 직진 임계 이상일 때만 선택됨.
    pm = make_pm(gyro_deg=3)
    for _ in range(pm.min_straight_frames_for_gps):
        pm.decide_orientation_sensor(6.0, 0.1)  # 직진 조건 반복
    assert pm.gps.reset_count == 0, "작은 누적 회전 + 직진이면 baseline 유지"
    assert pm.orientation_sensor == PoseManager.GPS


if __name__ == "__main__":
    test_resets_baseline_on_large_cumulative_turn()
    test_no_reset_on_small_cumulative_turn()
    print("ALL PASS")
