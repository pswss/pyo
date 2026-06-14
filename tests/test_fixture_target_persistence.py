"""미보고 토큰 목표 유지 테스트 — "맵핑만 하고 안 찍음" 수정.

원인 2개:
1. 후보 제거 기준이 robot_center_traversed → 벽 따라 '지나가기만 해도' victim 후보
   전멸, 보고 없이 영구 스킵. 올바른 기준 = robot_detected_fixture_from(보고한 곳).
2. target_age >= 60(≈2초) 블랙리스트 → 2초 내 도착 못 하면 victim 포기. 토큰까지
   보통 10초+ → 사실상 전부 포기. 한도 대폭 상향.
체크포인트는 '방문 = 완료'가 맞으므로 traversed 필터 유지.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np

from mapping.mapper import Mapper
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from agent.subagents.go_to_fixtures.go_to_fixtures_position_finder import PositionFinder


def make_finder():
    m = Mapper.__new__(Mapper)
    m.pixel_grid = CompoundExpandablePixelGrid((100, 100), pixel_per_m=10 / 0.06,
                                               robot_radius_m=0.037)
    m.robot_diameter = 0.074
    m.robot_grid_index = m.pixel_grid.coordinates_to_grid_index(np.array([0.05, 0.05]))
    # 로봇 주변 traversed (reachability BFS 시작 영역)
    ai = m.pixel_grid.grid_index_to_array_index(m.robot_grid_index)
    m.pixel_grid.arrays["traversed"][ai[0]-15:ai[0]+15, ai[1]-15:ai[1]+15] = True
    return m, PositionFinder(m)


def put_victim_zone(m, x, y):
    idx = m.pixel_grid.coordinates_to_array_index(np.array([x, y]))
    m.pixel_grid.arrays["victims"][idx[0], idx[1]] = True
    m.pixel_grid.arrays["fixture_distance_margin"][idx[0]-8:idx[0]+9, idx[1]-8:idx[1]+9] = True
    return idx


def test_traversed_zone_still_targeted():
    """지나가기만 한(미보고) victim 존 → 목표 유지돼야 함 (기존: 스킵)."""
    m, f = make_finder()
    idx = put_victim_zone(m, 0.08, 0.08)
    # 존 전체를 로봇 중심이 지나감 (보고는 안 함)
    m.pixel_grid.arrays["robot_center_traversed"][idx[0]-10:idx[0]+11, idx[1]-10:idx[1]+11] = True
    f.update(force_calculation=True)
    assert f.target_position_exists(), "미보고 victim은 지나갔어도 목표여야 함"


def test_reported_zone_not_targeted():
    """보고 완료(robot_detected_fixture_from) 존 → 목표 제외."""
    m, f = make_finder()
    idx = put_victim_zone(m, 0.08, 0.08)
    m.pixel_grid.arrays["robot_detected_fixture_from"][idx[0]-10:idx[0]+11, idx[1]-10:idx[1]+11] = True
    f.update(force_calculation=True)
    assert not f.target_position_exists(), "보고한 victim은 목표에서 빠져야 함"


def test_target_age_limit_generous():
    """목표 유지 한도 ≥ 300프레임 (2초 컷으로 포기하지 않게)."""
    m, f = make_finder()
    put_victim_zone(m, 0.08, 0.08)
    f.update(force_calculation=True)
    assert f.target_position_exists()
    for _ in range(299):
        f.update()
    assert f.target_position_exists(), "300프레임 미만에 목표 포기하면 안 됨"


if __name__ == "__main__":
    test_traversed_zone_still_targeted()
    test_reported_zone_not_targeted()
    test_target_age_limit_generous()
    print("OK")
