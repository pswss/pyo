"""근접 벽 등록 테스트 — traversed 거부 범위 축소.

증상: 벽에 가까이 가면 그 벽이 영영 안 찍힘.
원인: occupy_point가 traversed(로봇 풋프린트 직경 7.4cm 스탬프) 셀에 벽 등록을 거부.
포즈 오차 1~2cm로 풋프린트가 1cm(1.7px) 두께 벽 라인을 덮으면 복구 불가.
수정: 거부 기준을 robot_center_traversed(중심 ~2cm)로 축소 — 로봇 '중심'이 지난
자리는 진짜 벽일 수 없지만, 풋프린트 가장자리는 포즈 오차 내에서 벽일 수 있다.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from mapping.wall_mapper import WallMapper


def make_mapper():
    grid = CompoundExpandablePixelGrid((100, 100), pixel_per_m=10 / 0.06, robot_radius_m=0.037)
    wm = WallMapper(grid, robot_diameter=0.074)
    return grid, wm


def confirm_wall(wm, idx):
    """임계 초과까지 같은 셀 반복 타격."""
    for _ in range(wm.to_boolean_threshold + 2):
        wm.occupy_point(np.array(idx))


def test_footprint_sustained_evidence_registers():
    """풋프린트(traversed) 셀: dp 상한까지 지속 스캔되면 벽 등록 (근접 벽 복구)."""
    grid, wm = make_mapper()
    idx = (50, 50)
    grid.arrays["traversed"][idx] = True
    for _ in range(wm.max_detected_points + 2):
        wm.occupy_point(np.array(idx))
    assert grid.arrays["walls_raw"][idx], "지속 증거(상한 도달)면 풋프린트여도 벽 등록"


def test_footprint_transient_noise_blocked():
    """풋프린트 셀: 임계만 넘긴 일시 노이즈는 차단 (궤적 주변 얼룩 방지)."""
    grid, wm = make_mapper()
    idx = (50, 50)
    grid.arrays["traversed"][idx] = True
    confirm_wall(wm, idx)   # threshold+2 < 상한
    assert not grid.arrays["walls_raw"][idx], "일시 노이즈가 풋프린트에 벽 찍으면 안 됨"


def test_center_traversed_still_blocks_wall():
    """로봇 중심이 지난 셀 → 여전히 벽 등록 거부 (그 자리에 벽 있을 수 없음)."""
    grid, wm = make_mapper()
    idx = (50, 50)
    grid.arrays["robot_center_traversed"][idx] = True
    confirm_wall(wm, idx)
    assert not grid.arrays["walls_raw"][idx], "중심이 지난 자리는 벽일 수 없음"


def test_normal_cell_registers():
    grid, wm = make_mapper()
    idx = (40, 40)
    confirm_wall(wm, idx)
    assert grid.arrays["walls_raw"][idx]


if __name__ == "__main__":
    test_footprint_sustained_evidence_registers()
    test_footprint_transient_noise_blocked()
    test_center_traversed_still_blocks_wall()
    test_normal_cell_registers()
    print("OK")
