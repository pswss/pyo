"""가상 라이다(레이-세그먼트 교차) 단위 테스트."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from harness.webots_stub import install_stub
install_stub()

import numpy as np
from harness.virtual_lidar import cast_rays


def test_single_wall_straight_ahead():
    # 로봇 (0,0), 벽: y=0.10 에 가로 세그먼트. 위쪽(+y) 레이는 0.10m에서 끊겨야 함.
    walls = [((-1.0, 0.10), (1.0, 0.10))]
    in_pts, out_pts = cast_rays(walls, np.array([0.0, 0.0]), heading=0.0,
                                n_rays=8, max_d=0.48, min_d=0.036)
    # 8방향 중 +y 정방향 레이 1개: 거리 0.10
    dists = [np.hypot(p[0], p[1]) for p in in_pts]
    assert any(abs(d - 0.10) < 1e-6 for d in dists), dists
    # 아래쪽(-y) 레이는 벽 없음 → out_of_bounds (max_d)
    assert any(abs(p[1] + 0.48) < 1e-6 for p in out_pts), out_pts


def test_min_distance_filtered():
    # 너무 가까운 벽(0.02m < min_d)은 in에도 out에도 없어야 함
    walls = [((-1.0, 0.02), (1.0, 0.02))]
    in_pts, out_pts = cast_rays(walls, np.array([0.0, 0.0]), heading=0.0,
                                n_rays=4, max_d=0.48, min_d=0.036)
    dists = [np.hypot(p[0], p[1]) for p in in_pts]
    assert not any(d < 0.036 for d in dists), dists


def test_nearest_of_two_walls():
    walls = [((-1.0, 0.10), (1.0, 0.10)), ((-1.0, 0.20), (1.0, 0.20))]
    in_pts, _ = cast_rays(walls, np.array([0.0, 0.0]), heading=0.0,
                          n_rays=8, max_d=0.48, min_d=0.036)
    dists = sorted(np.hypot(p[0], p[1]) for p in in_pts)
    assert abs(dists[0] - 0.10) < 1e-6  # 가까운 벽이 이김


def test_heading_rotates_rays():
    # heading π/2 회전 시 레이 패턴이 회전해도 같은 벽을 같은 거리에서 맞음
    walls = [((-1.0, 0.10), (1.0, 0.10))]
    in_a, _ = cast_rays(walls, np.array([0.0, 0.0]), heading=0.0, n_rays=64)
    in_b, _ = cast_rays(walls, np.array([0.0, 0.0]), heading=np.pi / 2, n_rays=64)
    # 거리 분포가 동일해야 함 (레이 패턴 회전 ≠ 벽 거리 변화)
    da = sorted(round(np.hypot(p[0], p[1]), 6) for p in in_a)
    db = sorted(round(np.hypot(p[0], p[1]), 6) for p in in_b)
    assert da == db


def test_empty_walls_all_out_of_bounds():
    in_pts, out_pts = cast_rays([], np.array([0.0, 0.0]), heading=0.0, n_rays=8)
    assert in_pts == []
    assert len(out_pts) == 8
    assert all(abs(np.hypot(p[0], p[1]) - 0.48) < 1e-9 for p in out_pts)


if __name__ == "__main__":
    test_single_wall_straight_ahead()
    test_min_distance_filtered()
    test_nearest_of_two_walls()
    test_heading_rotates_rays()
    test_empty_walls_all_out_of_bounds()
    print("ALL PASS")
