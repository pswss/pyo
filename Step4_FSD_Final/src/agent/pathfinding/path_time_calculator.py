import numpy as np
import cv2 as cv

from data_structures.vectors import Position2D
from mapping.mapper import Mapper

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from algorithms.np_bool_array.efficient_a_star import aStarAlgorithm
from algorithms.np_bool_array.bfs import NavigatingBFSAlgorithm

from flags import SHOW_PATHFINDING_DEBUG, SHOW_GRANULAR_NAVIGATION_GRID

class PathTimeCalculator():
    """
    주어진 목표 위치까지 A*로 경로 길이를 계산하고,
    예상 이동 시간을 추정하는 유틸리티 클래스입니다.

    비선형 비용 함수 사용: time = n * factor + n^exponent
    이를 통해 짧은 경로보다 긴 경로의 비용이 비례 이상으로 높아집니다.
    에이전트 간 우선순위 결정에 활용될 수 있습니다.
    """
    def __init__(self, mapper: Mapper, factor: float, exponent: float):
        self.__a_star = aStarAlgorithm()
        # 시작점/목표점이 장애물 위에 있을 때 가장 가까운 빈 공간으로 보정하는 BFS
        self.__closest_free_point_finder = NavigatingBFSAlgorithm(lambda x : x == 0, lambda x: True)

        self.__mapper = mapper

        self.factor = factor       # 선형 거리 비용 계수
        self.exponent = exponent   # 거리의 지수 비용 계수 (>1이면 비선형 증가)


    def calculate(self, target_position: np.ndarray):
        """
        목표 위치까지의 경로 길이 n을 계산하여
        예상 비용(n * factor + n^exponent)을 반환합니다.
        """
        n = self.__calculate_path_lenght(target_position)
        return n * self.factor + n ** self.exponent

    def __calculate_path_lenght(self, target_position):
        """
        A*로 현재 위치에서 목표 위치까지의 경로를 계산하고 노드 수를 반환합니다.
        시작점/목표점이 장애물 위면 가장 가까운 빈 공간으로 보정합니다.
        """
        # 목표 그리드 인덱스로 픽셀 그리드 확장
        target_grid_index = self.__mapper.pixel_grid.coordinates_to_grid_index(target_position)
        self.__mapper.pixel_grid.expand_to_grid_index(target_grid_index)

        # 시작점: 로봇 현재 위치 (장애물 위면 가장 가까운 빈 공간으로 이동)
        start_array_index = self.__mapper.pixel_grid.coordinates_to_array_index(self.__mapper.robot_position)
        start_array_index = self.__get_closest_traversable_array_index(start_array_index)

        # 목표점: 좌표 → 배열 인덱스 변환 (장애물 위면 보정)
        target_array_index = self.__mapper.pixel_grid.coordinates_to_array_index(target_position)
        target_array_index = self.__get_closest_traversable_array_index(target_array_index)

        # A* 경로 계산: navigation_preference 레이어로 벽 근처 경로 회피
        a_star_path = self.__a_star.a_star(self.__mapper.pixel_grid.arrays["traversable"],
                                        start_array_index,
                                        target_array_index,
                                        self.__mapper.pixel_grid.arrays["navigation_preference"])


        return len(a_star_path)


    def __get_closest_traversable_array_index(self, array_index):
        """
        주어진 배열 인덱스가 장애물(traversable=True) 위에 있으면
        BFS로 가장 가까운 빈 공간을 반환합니다.
        """
        if self.__mapper.pixel_grid.arrays["traversable"][array_index[0], array_index[1]]:
            return  self.__closest_free_point_finder.bfs(found_array=self.__mapper.pixel_grid.arrays["traversable"],
                                                         traversable_array=self.__mapper.pixel_grid.arrays["traversable"],
                                                         start_node=array_index)[0]
        else:
            return array_index
