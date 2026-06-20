"""경로 추종 lookahead 테스트 — 이동 끊김(뚝뚝) 수정.

기존: 노드 전진 임계 3px + 노드 간격 2px → 목표가 항상 1.2~2.4cm 앞 →
2cm마다 도착·재조준 → 움찔움찔 + 직진 구간 전무(GPS 보정 불가).
수정: lookahead 8px(≈5cm) 안의 노드는 통과 처리, 그 너머 첫 노드를 목표로.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np
import types

from agent.pathfinding.pathfinder import PathFinder
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from data_structures.vectors import Position2D


def make_pf(path_nodes, robot_xy):
    pf = PathFinder.__new__(PathFinder)
    grid = CompoundExpandablePixelGrid((100, 100), pixel_per_m=10 / 0.06, robot_radius_m=0.037)
    mapper = types.SimpleNamespace(pixel_grid=grid, robot_position=Position2D(*robot_xy))
    pf._PathFinder__mapper = mapper
    pf._PathFinder__a_star_path = path_nodes
    pf._PathFinder__smooth_astar_path = path_nodes
    pf._PathFinder__a_star_index = 0
    return pf


def test_lookahead_skips_near_nodes():
    grid_tmp = CompoundExpandablePixelGrid((100, 100), pixel_per_m=10 / 0.06, robot_radius_m=0.037)
    robot_xy = (0.1, 0.1)
    rgi = grid_tmp.coordinates_to_grid_index(np.array(robot_xy))
    # 로봇 위치에서 +2px 간격 직선 노드 10개
    nodes = [np.array([int(rgi[0]) + 2 * i, int(rgi[1])]) for i in range(1, 11)]
    pf = make_pf(nodes, robot_xy)
    pf._PathFinder__calculate_path_index()
    # 8px 미만(2,4,6px) 노드 3개는 통과, 인덱스는 8px 노드(=index 3)
    assert pf._PathFinder__a_star_index == 3,         f"lookahead 8px 너머 첫 노드를 가리켜야 함, 실제 index={pf._PathFinder__a_star_index}"


def test_last_node_clamped():
    grid_tmp = CompoundExpandablePixelGrid((100, 100), pixel_per_m=10 / 0.06, robot_radius_m=0.037)
    robot_xy = (0.1, 0.1)
    rgi = grid_tmp.coordinates_to_grid_index(np.array(robot_xy))
    nodes = [np.array([int(rgi[0]) + 2, int(rgi[1])]), np.array([int(rgi[0]) + 4, int(rgi[1])])]
    pf = make_pf(nodes, robot_xy)
    pf._PathFinder__calculate_path_index()
    assert pf._PathFinder__a_star_index == len(nodes) - 1, "마지막 노드에서 멈춰야 함(목표 보존)"


if __name__ == "__main__":
    test_lookahead_skips_near_nodes()
    test_last_node_clamped()
    print("OK")
