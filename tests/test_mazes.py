"""미로 정의 + 주행 경로 생성기 테스트."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from harness.webots_stub import install_stub
install_stub()

import numpy as np
from harness.mazes import demo_maze, traversal_path

TILE = 0.12


def test_demo_maze_shapes():
    maze = demo_maze()
    assert len(maze["walls"]) > 10            # 세그먼트 여러 개
    assert len(maze["free_cells"]) >= 12      # 자유 타일 여러 개
    # 곡선(폴리라인) 포함 확인: 길이 0.04m 미만의 짧은 세그먼트 존재
    short = [w for w in maze["walls"]
             if np.hypot(w[1][0] - w[0][0], w[1][1] - w[0][1]) < 0.04]
    assert len(short) >= 4, "곡선 폴리라인 없음"


def test_traversal_visits_all_cells_with_small_steps():
    maze = demo_maze()
    path = traversal_path(maze, step=0.004)
    pts = np.array([p[0] for p in path])
    headings = np.array([p[1] for p in path])
    # 스텝 크기 ≤ 4mm
    deltas = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert deltas.max() < 0.0045, deltas.max()
    # 모든 자유 셀 중심 4cm 이내 통과
    for cell in maze["free_cells"]:
        c = np.array(cell, dtype=float)
        assert np.min(np.linalg.norm(pts - c, axis=1)) < 0.04, cell
    # heading은 이동 방향과 일치 (스텝 벡터와 각도 차 < 0.01rad, 정지 스텝 제외)
    for i in range(1, len(path)):
        d = pts[i] - pts[i - 1]
        n = np.linalg.norm(d)
        if n > 1e-9:
            expect = np.arctan2(d[0], d[1])   # heading 0=+y 규약
            diff = (headings[i] - expect + np.pi) % (2 * np.pi) - np.pi
            assert abs(diff) < 0.01


if __name__ == "__main__":
    test_demo_maze_shapes()
    test_traversal_visits_all_cells_with_small_steps()
    print("ALL PASS")
