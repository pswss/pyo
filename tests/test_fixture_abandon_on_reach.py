"""옵션 A: 토큰 최적 보고위치 도달했는데 보고 안 되면 토큰 전체 포기 (orbit 방지).

실런 증상: GoToFixtures가 최우선이라, 보고 못 한 토큰 주변에 후보가 남아있는 한
무한히 맴돔(reach→recalc→옆 crescent→reach…). robot_detected_fixture_from은
'보고'로만 찍히는데, 카메라가 그 위치에서 분류 실패/already_detected/오탐이면
보고가 안 돼 영영 안 빠져나옴 = "무작정 달려들면 계속 토큰 옆에서만 있음".

핵심: 보고 *가능한* 토큰은 접근 도중 report_fixture가 self-clear 한다. 따라서
'BFS 최적 보고위치(타겟)까지 실제 도달했는데도 미보고' = 그 위치에선 보고 불가
→ 토큰 zone 전체 포기하고 떠난다 (진짜 보고가능 피해자는 안 잃음).
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
    ai = m.pixel_grid.grid_index_to_array_index(m.robot_grid_index)
    m.pixel_grid.arrays["traversed"][ai[0] - 15:ai[0] + 15, ai[1] - 15:ai[1] + 15] = True
    return m, PositionFinder(m)


def put_victim_zone(m, x, y):
    idx = m.pixel_grid.coordinates_to_array_index(np.array([x, y]))
    m.pixel_grid.arrays["victims"][idx[0], idx[1]] = True
    m.pixel_grid.arrays["fixture_distance_margin"][idx[0] - 8:idx[0] + 9, idx[1] - 8:idx[1] + 9] = True
    return idx


def _target_gi(f):
    return f._PositionFinder__target


def test_reached_unreported_fixture_is_abandoned():
    """최적 보고위치(타겟)에 로봇 중심이 도달 → 미보고면 토큰 전체 포기."""
    m, f = make_finder()
    put_victim_zone(m, 0.08, 0.08)
    f.update(force_calculation=True)
    assert f.target_position_exists(), "초기엔 목표가 있어야 함"

    tai = m.pixel_grid.grid_index_to_array_index(_target_gi(f))
    # 로봇 중심이 그 타겟(최적 보고위치)에 도달했다고 표시
    m.pixel_grid.arrays["robot_center_traversed"][int(tai[0]), int(tai[1])] = True

    f.update()  # 도달 감지 → 토큰 포기 후 재계산
    assert not f.target_position_exists(), \
        "최적 보고위치 도달했는데 미보고면 토큰 전체를 포기해야 함 (orbit 방지)"


def test_partial_graze_keeps_target():
    """타겟이 아닌 zone 픽셀만 지나감(스쳐감) → 여전히 목표 유지(미보고 토큰 스킵 금지)."""
    m, f = make_finder()
    idx = put_victim_zone(m, 0.08, 0.08)
    f.update(force_calculation=True)
    tai = m.pixel_grid.grid_index_to_array_index(_target_gi(f))

    z = m.pixel_grid.arrays["robot_center_traversed"]
    z[idx[0] - 8:idx[0] + 9, idx[1] - 8:idx[1] + 9] = True
    z[int(tai[0]), int(tai[1])] = False  # 정작 타겟 픽셀은 미도달

    f.update()
    assert f.target_position_exists(), \
        "최적 보고위치엔 아직 도달 안 했으면 목표 유지해야 함 (스쳐 지나간 것만으로 스킵 금지)"


def test_abandon_with_multiple_victims_no_crash():
    """포기 윈도우에 victim 픽셀이 2개 이상이어도 크래시 없이 등록.

    회귀: 포기 목록을 numpy 배열로 보관하면 두 번째 등록 시 `gi not in list`가
    element-wise 비교 → "truth value of an array ... is ambiguous" ValueError로
    실런이 죽음. (단위테스트가 victim 1개라 빈 리스트만 거쳐 못 잡았던 갭.)
    """
    m, f = make_finder()
    idx = put_victim_zone(m, 0.08, 0.08)
    # 같은 zone 안에 victim 픽셀 하나 더 → 포기 윈도우에 2개 → dedup 비교 경로 유발
    m.pixel_grid.arrays["victims"][idx[0] + 2, idx[1] + 1] = True
    f.update(force_calculation=True)
    tai = m.pixel_grid.grid_index_to_array_index(_target_gi(f))
    m.pixel_grid.arrays["robot_center_traversed"][int(tai[0]), int(tai[1])] = True

    f.update()  # victim 2개 등록 — numpy 'in' 버그면 여기서 ValueError
    assert not f.target_position_exists(), \
        "다중 victim 포기 후에도 토큰 전체 제외돼야 함"


if __name__ == "__main__":
    test_reached_unreported_fixture_is_abandoned()
    test_partial_graze_keeps_target()
    test_abandon_with_multiple_victims_no_crash()
    print("OK")
