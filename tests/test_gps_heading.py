"""GPS 강건 heading(get_orientation_robust) 테스트.

원인 D 수정: 초기 캘리브레이션이 단일 GPS 측정으로 전체 런 heading 기준을 박아
v26 노이즈에 취약 → 맵 고정 회전. centroid 평균 방식이 단발보다 노이즈에 강건함을 검증.

Webots 'controller' 의존을 피하려고 Gps.__new__로 인스턴스 생성 후 위치 이력을 수동 주입.
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

from robot.devices.gps import Gps
from data_structures.vectors import Position2D


def make_gps(history):
    g = Gps.__new__(Gps)
    g._Gps__position_history = list(history)   # name-mangled private
    g.min_baseline_distance = 0.04
    g.min_calibration_baseline = 0.02   # 캘리브 전용 게이트(주행 baseline과 분리)
    g.position = history[-1]
    return g


# 결정적 톱니 노이즈(±2.5mm) — 무작위 없이 재현 가능
def noisy(i):
    return 0.0025 if i % 2 == 0 else -0.0025


def test_robust_recovers_straight_heading_under_noise():
    # +x 방향으로 0~5cm 직진 (12프레임), y에 ±2.5mm 노이즈
    hist = [Position2D(0.005 * i, noisy(i)) for i in range(12)]
    g = make_gps(hist)

    true = Position2D(0, 0).get_angle_to(Position2D(1, 0))  # +x 방향의 관례상 각도
    robust = g.get_orientation_robust()
    assert robust is not None, "충분히 이동했으면 None 아니어야 함"
    err = robust.get_absolute_distance_to(true).degrees
    assert err < 5.0, f"강건 heading 오차 {err:.1f}° (<5° 기대)"


def test_returns_none_when_not_moved():
    # 제자리(노이즈만) → 이동 부족 → None
    hist = [Position2D(noisy(i), noisy(i + 1)) for i in range(12)]
    g = make_gps(hist)
    assert g.get_orientation_robust() is None, "이동 없으면 None 반환해야 함"


def test_robust_beats_single_read_on_average():
    # 동일 경로에서 단발 측정 대비 강건 측정이 평균적으로 참값에 더 가까움
    hist = [Position2D(0.005 * i, noisy(i)) for i in range(12)]
    g = make_gps(hist)
    true = Position2D(0, 0).get_angle_to(Position2D(1, 0))
    robust_err = g.get_orientation_robust().get_absolute_distance_to(true).degrees
    single_err = g.get_orientation().get_absolute_distance_to(true).degrees
    assert robust_err <= single_err + 1e-6, \
        f"강건({robust_err:.2f}°)이 단발({single_err:.2f}°)보다 나빠선 안 됨"


if __name__ == "__main__":
    test_robust_recovers_straight_heading_under_noise()
    test_returns_none_when_not_moved()
    test_robust_beats_single_read_on_average()
    print("ALL PASS")
