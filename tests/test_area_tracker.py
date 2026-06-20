"""Area 추적기 테스트 — 맵 일치율 46%의 진범(Area4 '*' 미기록) 수정 1단계.

정답 행렬(MapAnswer.py)은 room==4 타일을 5×5 '*'로 채움 → 제출에 '*' 없으면
world2 기준 ~200셀 불일치. 통로 횡단으로 현재 구역을 추적해 Area4 체류 위치를 기록.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from mapping.area_tracker import AreaTracker
from data_structures.vectors import Position2D


def walk(tr, pts, letter):
    for (x, y) in pts:
        tr.step(letter, Position2D(x, y))


def test_crossing_g_passage_enters_area4():
    tr = AreaTracker()
    walk(tr, [(0.0, 0.0)], None)                        # Area1 일반 타일
    walk(tr, [(0.10, 0.0), (0.16, 0.0)], "g")           # 통로 진입+내부 (12cm 횡단)
    walk(tr, [(0.22, 0.0)], None)                       # 반대편으로 나옴
    assert tr.current_area == 4
    assert len(tr.area4_positions) >= 1                 # Area4 위치 기록 시작


def test_retreat_same_side_keeps_area():
    tr = AreaTracker()
    walk(tr, [(0.0, 0.0)], None)
    walk(tr, [(0.10, 0.0)], "g")                        # 통로에 살짝 발 들임
    walk(tr, [(0.09, 0.0)], None)                       # 같은 쪽으로 후퇴 (변위 1cm)
    assert tr.current_area == 1, "되돌아 나가면 구역 유지"
    assert len(tr.area4_positions) == 0


def test_round_trip_returns_to_area1():
    tr = AreaTracker()
    walk(tr, [(0.0, 0.0)], None)
    walk(tr, [(0.10, 0.0), (0.16, 0.0)], "g")
    walk(tr, [(0.22, 0.0)], None)                       # → Area4
    walk(tr, [(0.16, 0.0), (0.10, 0.0)], "g")           # 다시 통로
    walk(tr, [(0.02, 0.0)], None)                       # → Area1 복귀
    assert tr.current_area == 1


def test_b_passage_goes_area2_not_4():
    tr = AreaTracker()
    walk(tr, [(0.0, 0.0)], None)
    walk(tr, [(0.10, 0.0), (0.18, 0.0)], "b")
    walk(tr, [(0.24, 0.0)], None)
    assert tr.current_area == 2
    assert len(tr.area4_positions) == 0


if __name__ == "__main__":
    test_crossing_g_passage_enters_area4()
    test_retreat_same_side_keeps_area()
    test_round_trip_returns_to_area1()
    test_b_passage_goes_area2_not_4()
    print("OK")
