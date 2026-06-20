"""최종 맵 행렬에 토큰(victim/hazmat) 글자가 들어가는지 검증.

기존 버그: final_matrix_creator가 victim/hazmat을 최종 행렬에 전혀 안 넣음
(보고는 되지만 제출 맵엔 토큰 누락). 수정: 보고한 토큰을 그 위치의 벽 칸에 글자로 배치.
이 테스트는 배치 로직(__place_fixtures)이 토큰 글자를 벽('1') 칸에 기록하는지 확인한다.
(시뮬 채점 정밀도는 검증 못 하지만, '토큰이 행렬에 들어간다'는 핵심은 검증.)
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from data_structures.vectors import Position2D
from final_matrix_creation.final_matrix_creator import FinalMatrixCreator


def _setup():
    res = 10 / 0.06
    grid = CompoundExpandablePixelGrid(initial_shape=np.array([100, 100]),
                                       pixel_per_m=res, robot_radius_m=0.03)
    fmc = FinalMatrixCreator(tile_size=0.12, resolution=res)
    return grid, fmc


def _cell_for(grid, fmc, pos, offsets):
    """__place_fixtures와 동일하게 위치 → 텍스트 격자 셀 계산."""
    sq = fmc._FinalMatrixCreator__square_size_px
    arr = grid.coordinates_to_array_index(np.array([pos.x, pos.y])) - offsets
    half = arr // sq
    return int(half[0]) * 2, int(half[1]) * 2


def test_token_letter_lands_on_wall_cell():
    grid, fmc = _setup()
    offsets = np.array([0, 0])
    pos = Position2D(0.12, 0.18)
    tr, tc = _cell_for(grid, fmc, pos, offsets)

    # 충분히 큰 텍스트 격자에 그 위치를 벽('1')으로
    H = W = max(tr, tc) + 10
    text_grid = [["0"] * W for _ in range(H)]
    text_grid[tr][tc] = "1"

    fmc._FinalMatrixCreator__place_fixtures(text_grid, grid, offsets, [(pos, "H")])
    assert text_grid[tr][tc] == "H", f"토큰 글자가 벽 칸에 안 들어감: ({tr},{tc})={text_grid[tr][tc]}"


def test_hazmat_and_empty_letter():
    grid, fmc = _setup()
    offsets = np.array([0, 0])
    pos = Position2D(0.24, 0.12)
    tr, tc = _cell_for(grid, fmc, pos, offsets)
    H = W = max(tr, tc) + 10
    text_grid = [["0"] * W for _ in range(H)]
    text_grid[tr][tc] = "1"

    # 빈 글자/None은 무시, 유효 hazmat 'F'는 배치
    fmc._FinalMatrixCreator__place_fixtures(text_grid, grid, offsets,
                                            [(pos, ""), (pos, None), (pos, "F")])
    assert text_grid[tr][tc] == "F", f"hazmat 글자 미배치: ({tr},{tc})={text_grid[tr][tc]}"


if __name__ == "__main__":
    test_token_letter_lands_on_wall_cell()
    test_hazmat_and_empty_letter()
    print("OK")
