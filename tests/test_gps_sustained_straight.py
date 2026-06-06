"""연속 직진 게이트 테스트 (핵심: report_fixture 짧은 접근에서 GPS chord 오염 차단).

로그 확인: report_fixture의 짧은 접근 이동(수 프레임)에서 GPS chord가 노이즈에 지배돼
5~22° 잘못된 heading을 자이로에 박았다(원인 가). decide_orientation_sensor가 연속 직진
프레임이 임계(min_straight_frames_for_gps) 이상일 때만 GPS를 선택하는지 검증.
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
    def __init__(self, deg=0):
        self._o = Angle(math.radians(deg))
    def get_orientation(self):
        return self._o
    def get_angular_velocity(self):
        return Angle(0)  # 매-스텝 회전 없음 → 직진 각속도 조건 통과


class FakeGps:
    def __init__(self):
        self.reset_count = 0
    def reset_orientation_baseline(self):
        self.reset_count += 1


def make_pm():
    return PoseManager(gps=FakeGps(), gyroscope=FakeGyro())


def test_short_straight_burst_stays_gyro():
    # report_fixture 접근처럼 직진이 임계 미만(예: 5프레임)만 지속 → GPS 선택 안 함(자이로 유지).
    pm = make_pm()
    burst = pm.min_straight_frames_for_gps - 1
    for _ in range(burst):
        pm.decide_orientation_sensor(6.0, 0.1)
    assert pm.orientation_sensor == PoseManager.GYROSCOPE, \
        "짧은 직진 버스트(report_fixture 접근)는 GPS heading을 신뢰하면 안 됨"


def test_sustained_straight_uses_gps():
    # 긴 복도처럼 직진이 임계 이상 지속 → GPS 선택.
    pm = make_pm()
    for _ in range(pm.min_straight_frames_for_gps):
        pm.decide_orientation_sensor(6.0, 0.1)
    assert pm.orientation_sensor == PoseManager.GPS, "지속 직진이면 GPS heading 사용"


def test_non_straight_frame_resets_counter():
    # 직진 누적 중 비직진 프레임 한 번 → 카운터 리셋 → 다시 임계까지 직진해야 GPS.
    pm = make_pm()
    for _ in range(pm.min_straight_frames_for_gps - 1):
        pm.decide_orientation_sensor(6.0, 0.1)
    pm.decide_orientation_sensor(6.0, 5.0)  # 큰 좌우 속도차 → 비직진
    assert pm.consecutive_straight_frames == 0, "비직진 프레임은 연속 직진 카운터를 리셋"
    assert pm.orientation_sensor == PoseManager.GYROSCOPE


if __name__ == "__main__":
    test_short_straight_burst_stays_gyro()
    test_sustained_straight_uses_gps()
    test_non_straight_frame_resets_counter()
    print("ALL PASS")
