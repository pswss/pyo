"""보고 완료 fixture 웨이포인트 고정 해제 테스트.

실런: 보고 성공 후에도 웨이포인트가 그 fixture에 고정. 원인 = detected_from
스탬프가 '로봇 위치' 중심 10cm 원만이라, victim 중심 zone(반경 ~9px)의 반대편
초승달이 스탬프 밖에 남아 GoToFixtures 후보로 살아있음. 수정 = 보고 시 근처
victim/hazmat 포인트 주변까지 스탬프 → 한 번 보고로 zone 후보 전부 소멸.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from data_structures.vectors import Position2D
from mapping.fixture_mapper import FixtureMapper


def setup():
    grid = CompoundExpandablePixelGrid((140, 140), pixel_per_m=10 / 0.06, robot_radius_m=0.037)
    fm = FixtureMapper(grid, tile_size=0.12)
    return grid, fm


def test_far_side_of_victim_zone_cleared():
    grid, fm = setup()
    victim_idx = (60, 60)
    grid.arrays["victims"][victim_idx] = True
    # 로봇은 victim에서 ~8cm(13px) 떨어진 곳에서 보고.
    # 인덱스 규약: idx=[y*res+off, x*res+off], shape(140)→off=70
    robot_pos = Position2D((47 - 70) * 0.006, (60 - 70) * 0.006)
    robot_idx = grid.coordinates_to_array_index(np.array([robot_pos.x, robot_pos.y]))
    assert abs(int(robot_idx[0]) - 60) <= 1 and abs(int(robot_idx[1]) - 47) <= 1, \
        f"테스트 좌표 셋업 오류: {robot_idx}"
    fm.map_detected_fixture(robot_pos)
    far_side = (60, 68)   # victim 너머 8px — zone(9px) 안, 로봇 10cm 원 밖
    assert grid.arrays["robot_detected_fixture_from"][far_side], \
        "보고 후 victim zone 반대편도 detected_from으로 덮여야 함 (웨이포인트 고정 방지)"


def test_unrelated_far_victim_untouched():
    grid, fm = setup()
    grid.arrays["victims"][(60, 60)] = True
    grid.arrays["victims"][(110, 110)] = True   # 다른 미보고 victim (멀리)
    robot_pos = Position2D((47 - 70) * 0.006, (60 - 70) * 0.006)
    fm.map_detected_fixture(robot_pos)
    assert not grid.arrays["robot_detected_fixture_from"][(110, 110)], \
        "멀리 있는 미보고 victim은 건드리면 안 됨"


if __name__ == "__main__":
    test_far_side_of_victim_zone_cleared()
    test_unrelated_far_victim_untouched()
    print("OK")
