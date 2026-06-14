"""늪 근처 왕복 진동 수정 테스트.

D1: follow_walls가 공유 fixture_distance_margin 레이어를 in-place 훼손(늪/traversable
    후보 제거 + dither를 원본에 기록). 15스텝 주기 재생성과 맞물려 후보 집합이 출렁
    → 늪 경계에서 목표 플립 → 왕복 진동. 수정: 복사본에서 연산, 공유 레이어 불변.
D2: check_swamp_proximity가 늪 2cm 경계에서 매 프레임 센서 모드 토글(채터링).
    수정: 복귀에 디바운스(연속 N프레임 늪 비근접 시에만 auto 복귀).
"""
import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np

_fake = types.ModuleType("controller")
class _Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, n): return _Any()
_fake.Robot = _Any
_fake.__getattr__ = lambda n: _Any
sys.modules["controller"] = _fake

from mapping.mapper import Mapper
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from agent.subagents.follow_walls.follow_walls_position_finder import PositionFinder
from executor.executor import Executor


def make_mapper():
    m = Mapper.__new__(Mapper)
    m.pixel_grid = CompoundExpandablePixelGrid((100, 100), pixel_per_m=10 / 0.06,
                                               robot_radius_m=0.037)
    m.robot_diameter = 0.074
    m.robot_grid_index = m.pixel_grid.coordinates_to_grid_index(np.array([0.05, 0.05]))
    return m


def test_shared_margin_layer_not_mutated():
    """update() 후 공유 fixture_distance_margin 원본 불변이어야 함."""
    m = make_mapper()
    margin = m.pixel_grid.arrays["fixture_distance_margin"]
    margin[40:60, 40:60] = True
    m.pixel_grid.arrays["swamps"][45:55, 45:55] = True
    before = margin.copy()
    f = PositionFinder(m)
    f.update(force_calculation=True)
    assert np.array_equal(m.pixel_grid.arrays["fixture_distance_margin"], before), \
        "공유 마진 레이어가 in-place 변형됨 (늪/dither/고립제거가 원본에 기록)"


def make_executor(close_seq):
    ex = Executor.__new__(Executor)
    seq = iter(close_seq)
    ex.mapper = types.SimpleNamespace(is_close_to_swamp=lambda: next(seq))
    ex.robot = types.SimpleNamespace(auto_decide_orientation_sensor=True,
                                     orientation_sensor=None, GYROSCOPE="GYRO")
    ex._Executor__swamp_clear_frames = 0
    return ex


def test_swamp_sensor_no_chatter():
    """경계에서 근접/비근접 교차해도 auto 복귀는 디바운스 후에만."""
    pattern = [True, False, True, False, True, False]
    ex = make_executor(pattern)
    for _ in pattern:
        ex.check_swamp_proximity()
    assert ex.robot.auto_decide_orientation_sensor is False, \
        "경계 교차 중엔 자이로 고정 유지(채터링 금지)"


def test_swamp_sensor_returns_after_debounce():
    """늪에서 충분히 벗어나면(연속 비근접) auto 복귀."""
    pattern = [True] + [False] * 40
    ex = make_executor(pattern)
    for _ in pattern:
        ex.check_swamp_proximity()
    assert ex.robot.auto_decide_orientation_sensor is True, \
        "연속 비근접이면 auto 복귀해야 함"


if __name__ == "__main__":
    test_shared_margin_layer_not_mutated()
    test_swamp_sensor_no_chatter()
    test_swamp_sensor_returns_after_debounce()
    print("OK")
