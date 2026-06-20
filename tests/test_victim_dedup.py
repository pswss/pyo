"""피해자 맵핑 디덥 테스트 (원인2: 한 피해자가 여러 인접 셀에 중복 마킹되는 번짐).

map_fixtures가 매 프레임 victim 셀을 영구 마킹하지만 디덥/클러스터링이 없어,
한 피해자를 여러 프레임·여러 위치에서 보면 인접 셀 여러 개가 True가 됨(sim 이미지: 세로 번짐).
_mark_victim이 dedup 반경 내 기존 마크가 있으면 새 마크를 건너뛰는지 검증.
"""
import os
import sys
import types

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

_fake = types.ModuleType("controller")
class _Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, n): return _Any()
_fake.Robot = _Any
_fake.__getattr__ = lambda n: _Any
sys.modules["controller"] = _fake

from fixture_detection.fixture_detection import FixtureDetector


class FakeGrid:
    def __init__(self, shape=(100, 100)):
        self.array_shape = shape
        self.arrays = {
            "victims":       np.zeros(shape, dtype=np.bool_),
            "victim_angles": np.zeros(shape, dtype=np.float32),
        }


def make_detector():
    d = FixtureDetector.__new__(FixtureDetector)
    d.pixel_grid = FakeGrid()
    d.victim_dedup_radius = 5  # 픽셀; 같은 피해자 번짐을 한 마크로 합침
    return d


def test_nearby_marks_deduped():
    # 한 피해자를 인접 셀에서 여러 번 보고 → 첫 마크만 남고 나머지는 스킵.
    d = make_detector()
    assert d._mark_victim((50, 50), 0.0) is True
    assert d._mark_victim((52, 50), 0.1) is False   # 반경 내 → 스킵
    assert d._mark_victim((50, 53), 0.1) is False   # 반경 내 → 스킵
    assert int(d.pixel_grid.arrays["victims"].sum()) == 1, "한 피해자는 셀 하나만 차지해야 함"


def test_far_mark_kept():
    # 다른 피해자(반경 밖)는 별도 마크로 남음.
    d = make_detector()
    assert d._mark_victim((50, 50), 0.0) is True
    assert d._mark_victim((50, 80), 0.0) is True
    assert int(d.pixel_grid.arrays["victims"].sum()) == 2, "멀리 떨어진 피해자는 별도 마크 유지"


def test_first_mark_angle_recorded():
    # 첫 마크 시 각도도 기록되어야 함(기존 동작 보존).
    d = make_detector()
    d._mark_victim((50, 50), 1.23)
    assert abs(float(d.pixel_grid.arrays["victim_angles"][50, 50]) - 1.23) < 1e-5


if __name__ == "__main__":
    test_nearby_marks_deduped()
    test_far_mark_kept()
    test_first_mark_angle_recorded()
    print("ALL PASS")
