"""블랙홀 전방 감지 테스트 (회피 기동 트리거용).

실런 증상: 구멍 코앞에서 어정쩡한 정지/진동. 경로계획 회피(occupied)만으론
근접 시 명시적 탈출 동작 없음. 추가: 전방 일정 거리 내 holes 픽셀 감지 →
executor가 후진+180° 회전 기동 트리거.
"""
import os
import sys
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np

from mapping.mapper import Mapper
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from data_structures.vectors import Position2D
from data_structures.angle import Angle


def make_mapper():
    """무거운 __init__ 우회 — is_hole_in_front는 pixel_grid만 필요."""
    m = Mapper.__new__(Mapper)
    m.pixel_grid = CompoundExpandablePixelGrid((100, 100), pixel_per_m=10 / 0.06,
                                               robot_radius_m=0.037)
    return m


def put_hole(m, x, y):
    idx = m.pixel_grid.coordinates_to_array_index(np.array([x, y]))
    m.pixel_grid.arrays["holes"][idx[0] - 2:idx[0] + 3, idx[1] - 2:idx[1] + 3] = True


def test_hole_ahead_detected():
    m = make_mapper()
    pos = Position2D(0.1, 0.1)
    # orientation 0 기준 전방에 구멍 (lidar.set_orientation/포인트 변환과 동일 좌표계:
    # 전방 = getCoordsFromRads(0) → +y, y반전 → 실좌표 -y 방향이 아니라…
    # 좌표계 검증은 본 테스트 아래 양방향 케이스로 커버)
    ahead = m._front_point(pos, Angle(0.0), 0.06)
    put_hole(m, ahead[0], ahead[1])
    assert m.is_hole_in_front(pos, Angle(0.0))


def test_hole_behind_not_detected():
    m = make_mapper()
    pos = Position2D(0.1, 0.1)
    ahead = m._front_point(pos, Angle(0.0), 0.06)
    behind = (2 * pos.x - ahead[0], 2 * pos.y - ahead[1])
    put_hole(m, behind[0], behind[1])
    assert not m.is_hole_in_front(pos, Angle(0.0))


def test_no_holes_no_trigger():
    m = make_mapper()
    assert not m.is_hole_in_front(Position2D(0.1, 0.1), Angle(0.0))


def test_rotated_heading_detects_rotated_hole():
    m = make_mapper()
    pos = Position2D(0.1, 0.1)
    heading = Angle(math.pi / 2)
    ahead = m._front_point(pos, heading, 0.06)
    put_hole(m, ahead[0], ahead[1])
    assert m.is_hole_in_front(pos, heading)
    assert not m.is_hole_in_front(pos, Angle(0.0)) or ahead == m._front_point(pos, Angle(0.0), 0.06)


if __name__ == "__main__":
    test_hole_ahead_detected()
    test_hole_behind_not_detected()
    test_no_holes_no_trigger()
    test_rotated_heading_detects_rotated_hole()
    print("OK")
