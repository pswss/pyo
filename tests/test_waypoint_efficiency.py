"""웨이포인트 효율 + 구멍 기동 조건 테스트.

증상(실런): ① 목표를 로봇 코앞(2~3cm)에 연발 설정 → 조향만 하다 시간 낭비 + 직진
구간 소멸(GPS 보정 불가). ② 바로 앞 미탐지 두고 먼 목표 유지 — GoToNonDiscovered
재계산 조건이 '목표가 벽이 됐을 때'뿐이라, 가는 도중 목표가 discovered 돼도(갈 이유
소멸) 끝까지 감. ③ 구멍 회피 180°가 웨이포인트와 구멍이 안 겹쳐도 무조건 발동.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np

from mapping.mapper import Mapper
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from data_structures.vectors import Position2D
from agent.subagents.follow_walls.follow_walls_position_finder import PositionFinder as FWFinder
from agent.subagents.go_to_non_discovered.go_to_non_discovered_position_finder import PositionFinder as NDFinder


def make_mapper():
    m = Mapper.__new__(Mapper)
    m.pixel_grid = CompoundExpandablePixelGrid((120, 120), pixel_per_m=10 / 0.06,
                                               robot_radius_m=0.037)
    m.robot_diameter = 0.074
    m.robot_position = Position2D(0.1, 0.1)
    m.robot_grid_index = m.pixel_grid.coordinates_to_grid_index(np.array([0.1, 0.1]))
    ai = m.pixel_grid.grid_index_to_array_index(m.robot_grid_index)
    m.pixel_grid.arrays["traversed"][ai[0]-15:ai[0]+15, ai[1]-15:ai[1]+15] = True
    return m


def idx_of(m, x, y):
    return m.pixel_grid.coordinates_to_array_index(np.array([x, y]))


def dist_to(m, t):
    """파인더 반환형(Position2D 또는 ndarray) 모두 처리."""
    tx = float(t.x) if hasattr(t, "x") else float(t[0])
    ty = float(t.y) if hasattr(t, "y") else float(t[1])
    return m.robot_position.get_distance_to(Position2D(tx, ty))


# ---------- ① FollowWalls 코앞 목표 금지 ----------

def test_follow_walls_skips_micro_targets():
    m = make_mapper()
    margin = m.pixel_grid.arrays["fixture_distance_margin"]
    near = idx_of(m, 0.12, 0.1)    # 로봇에서 2cm
    far = idx_of(m, 0.2, 0.1)      # 로봇에서 10cm
    margin[near[0]-2:near[0]+3, near[1]-2:near[1]+3] = True
    margin[far[0]-2:far[0]+3, far[1]-2:far[1]+3] = True
    f = FWFinder(m)
    f.update(force_calculation=True)
    assert f.target_position_exists(), "10cm 후보가 있으니 목표 존재해야 함"
    t = f.get_target_position()
    assert dist_to(m, t) > 0.045, \
        f"코앞(<4.5cm) 마이크로 목표 금지, 실제 {dist_to(m, t):.3f}m"


# ---------- ② GoToNonDiscovered 목표 discovered 시 재계산 ----------

def test_non_discovered_retargets_when_objective_discovered():
    m = make_mapper()
    disc = m.pixel_grid.arrays["discovered"]
    disc[:, :] = True
    far = idx_of(m, 0.3, 0.1)
    disc[far[0]-2:far[0]+3, far[1]-2:far[1]+3] = False   # 미탐지 = 먼 포켓만
    f = NDFinder(m)
    f.update(force_calculation=True)
    assert f.target_position_exists()
    # 가는 도중 그 포켓이 시야에 들어옴 + 가까운 새 미탐지 등장
    disc[:, :] = True
    near = idx_of(m, 0.16, 0.1)
    disc[near[0]-2:near[0]+3, near[1]-2:near[1]+3] = False
    f.update()   # force 없이 — 기존 코드는 먼 (이미 discovered된) 목표 유지
    t = f.get_target_position()
    assert t is not None
    assert dist_to(m, t) < 0.12, \
        f"discovered된 목표는 버리고 가까운 미탐지로 재설정해야 함, 실제 {dist_to(m, t):.3f}m"


# ---------- ②-2 주기 재평가: 도중에 가까운 미탐지 등장 시 갈아타기 ----------

def test_non_discovered_periodic_retarget_to_nearer():
    """먼 목표로 가는 도중 가까운 미탐지가 '새로 생기면'(둘 다 미탐지 유지)
    주기 재평가로 가까운 쪽으로 전환해야 함 — 실런: 맵 반 바퀴 동선."""
    m = make_mapper()
    disc = m.pixel_grid.arrays["discovered"]
    disc[:, :] = True
    far = idx_of(m, 0.34, 0.1)
    disc[far[0]-2:far[0]+3, far[1]-2:far[1]+3] = False
    f = NDFinder(m)
    f.update(force_calculation=True)
    assert dist_to(m, f.get_target_position()) > 0.2
    near = idx_of(m, 0.16, 0.1)
    disc[near[0]-2:near[0]+3, near[1]-2:near[1]+3] = False   # 가까운 프런티어 등장 (먼 것도 유지)
    for _ in range(95):                                       # 주기(90프레임) 경과
        f.update()
    assert dist_to(m, f.get_target_position()) < 0.12, \
        f"주기 재평가로 가까운 미탐지로 전환해야 함, 실제 {dist_to(m, f.get_target_position()):.3f}m"


# ---------- ③ 구멍-웨이포인트 겹침 판정 ----------

def test_hole_between_detects_overlap():
    m = make_mapper()
    hole = idx_of(m, 0.16, 0.1)
    m.pixel_grid.arrays["holes"][hole[0]-2:hole[0]+3, hole[1]-2:hole[1]+3] = True
    assert m.is_hole_between(Position2D(0.1, 0.1), Position2D(0.22, 0.1)), \
        "직선 경로가 구멍을 관통하면 True"
    assert not m.is_hole_between(Position2D(0.1, 0.1), Position2D(0.1, 0.22)), \
        "다른 방향 경로는 False"


if __name__ == "__main__":
    test_follow_walls_skips_micro_targets()
    test_non_discovered_retargets_when_objective_discovered()
    test_non_discovered_periodic_retarget_to_nearer()
    test_hole_between_detects_overlap()
    print("OK")
