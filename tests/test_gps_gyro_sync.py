"""GPS↔자이로 동기화 가드 테스트 (왼쪽하단 45° 갑작스런 회전 원인).

곡선 구간이 직진 판정을 통과하면 GPS가 측정한 chord 방향(실제 heading과 큰 차이)이
자이로에 동기화되어 갑작스런 점프가 생긴다. calculate_orientation이 GPS와 자이로 차이가
임계(30°) 이상이면 동기화를 거부하고 자이로를 유지하는지 검증.
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
        self.synced_to = None
    def get_orientation(self):
        return self._o
    def get_angular_velocity(self):
        return Angle(0)
    def set_orientation(self, angle):
        self.synced_to = angle
        self._o = angle


class FakeGps:
    def __init__(self, deg):
        self._h = Angle(math.radians(deg))
    def get_orientation(self):
        return self._h


def make_pm(gps_deg, gyro_deg):
    pm = PoseManager(gps=FakeGps(gps_deg), gyroscope=FakeGyro(gyro_deg))
    pm.orientation_sensor = PoseManager.GPS  # GPS 사용 분기
    return pm


def test_large_disagreement_rejected():
    # GPS 45°, 자이로 0° → 45° 차이 ≥ 30° → 동기화 거부, 자이로(0°) 유지
    pm = make_pm(gps_deg=45, gyro_deg=0)
    pm.calculate_orientation()
    assert pm.gyroscope.synced_to is None, "큰 불일치 시 자이로 동기화 안 해야 함"
    assert pm.orientation.get_absolute_distance_to(Angle(0)).degrees < 0.5, "자이로 값 유지"


def test_deadband_disagreement_ignored():
    # GPS 3°, 자이로 0° → 3° < 데드밴드 5° → 노이즈로 무시, 보정 안 함, 자이로 유지
    pm = make_pm(gps_deg=3, gyro_deg=0)
    pm.calculate_orientation()
    assert pm.gyroscope.synced_to is None, "데드밴드 이하 차이는 보정 안 해야 함"
    assert pm.orientation.get_absolute_distance_to(Angle(0)).degrees < 0.5, "자이로 값 유지"


def test_in_band_partial_correction():
    # GPS 10°, 자이로 0° → 5°<10°<30° → 게인(0.05)만큼만 부분 보정 = 0.5°
    pm = make_pm(gps_deg=10, gyro_deg=0)
    pm.calculate_orientation()
    assert pm.gyroscope.synced_to is not None, "밴드 내 차이는 부분 보정해야 함"
    expected = pm.gps_correction_gain * 10.0  # 0.5°
    assert pm.orientation.get_absolute_distance_to(Angle(math.radians(expected))).degrees < 0.2, \
        f"부분 보정은 게인×차이(={expected:.2f}°)만큼만 이동해야 함"


if __name__ == "__main__":
    test_large_disagreement_rejected()
    test_deadband_disagreement_ignored()
    test_in_band_partial_correction()
    print("ALL PASS")
