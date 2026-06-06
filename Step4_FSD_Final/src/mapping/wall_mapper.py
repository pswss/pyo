import numpy as np
import cv2 as cv
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid

from data_structures.vectors import Position2D

import skimage

class WallMapper:
    """
    라이다 포인트클라우드를 받아 픽셀 그리드에 벽/장애물을 등록하는 클래스입니다.

    처리 흐름:
    1. 범위 내 포인트(in_bounds) → 해당 픽셀의 detected_points 카운터 증가
       → 임계값(3회) 초과 시 walls 레이어에 True 등록
    2. 범위 초과 포인트(out_of_bounds) → seen_by_lidar 레이어에 빔 궤적 등록
    3. 카메라가 본 영역과 라이다 벽을 교차하여 walls_seen_by_camera 갱신
    4. 벽 주변에 로봇 크기만큼 통과 불가 영역(traversable) 생성
    5. 벽 근처에 낮은 선호도(navigation_preference) 부여하여 경로 탐색 시 벽 회피 유도
    """
    def __init__(self, compound_grid: CompoundExpandablePixelGrid, robot_diameter: float) -> None:
        self.grid = compound_grid

        compensation = 0
        # 로봇 직경을 픽셀 단위로 변환
        self.robot_diameter = int(robot_diameter * self.grid.resolution) + compensation * 2
        self.robot_radius = int(robot_diameter / 2 * self.grid.resolution) + compensation

        self.to_boolean_threshold = 3   # 이 횟수 이상 감지된 포인트만 벽으로 확정
        self.delete_threshold = 1       # 이 횟수 이하의 감지 포인트는 노이즈로 제거

        # 로봇 크기 원형 마스크 (traversable 영역 생성에 사용)
        self.robot_diameter_template = np.zeros((self.robot_diameter, self.robot_diameter), dtype=np.uint8)
        self.robot_diameter_template = cv.circle(self.robot_diameter_template,
                                                  (self.robot_radius, self.robot_radius),
                                                  self.robot_radius, 255, -1)
        self.robot_diameter_template = self.robot_diameter_template.astype(np.bool_)

        # 경로 탐색 선호도 그라디언트 템플릿 (벽 근처일수록 높은 값 → 회피)
        self.preference_template = self.__generate_quadratic_circle_gradient(
            self.robot_radius, self.robot_radius * 1.7)

    def load_point_cloud(self, in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position):
        """
        라이다 데이터를 받아 seen_by_lidar 초기화 후 벽/여유 공간을 등록합니다.
        매 타임스텝마다 호출됩니다.
        """
        robot_position_as_array = np.array(robot_position, dtype=float)

        self.__reset_seen_by_lidar()  # 이전 프레임의 라이다 시야 초기화

        self.load_in_bounds_point_cloud(in_bounds_point_cloud, robot_position_as_array)
        self.load_out_of_bounds_point_cloud(out_of_bounds_point_cloud, robot_position_as_array)

    def load_in_bounds_point_cloud(self, point_cloud, robot_position):
        """
        감지 범위 내 포인트들을 처리합니다:
        - 그리드 확장
        - 포인트 카운터 증가 및 벽 등록
        - 라이다 빔 궤적 기록
        - 노이즈 필터링
        - 통과 불가 영역 생성
        """
        for p in point_cloud:
            point = np.array(p, dtype=float) + robot_position

            point_grid_index = self.grid.coordinates_to_grid_index(point)
            self.grid.expand_to_grid_index(point_grid_index)

            robot_array_index = self.grid.coordinates_to_array_index(robot_position)
            point_array_index = self.grid.grid_index_to_array_index(point_grid_index)

            self.occupy_point(point_array_index)                                      # 벽 카운터 증가
            self.mark_point_as_seen_by_lidar(robot_array_index, point_array_index)   # 빔 궤적 기록

        self.filter_out_noise()            # 저빈도 포인트 제거
        self.generate_navigation_margins() # traversable + navigation_preference 계산

    def load_out_of_bounds_point_cloud(self, point_cloud, robot_position):
        """
        감지 범위 초과 방향의 포인트들을 처리합니다:
        - 열린 공간 방향 라이다 빔 궤적 기록
        - 카메라에서 본 벽 영역 계산 갱신
        """
        for p in point_cloud:
            point = np.array(p, dtype=float) + robot_position

            point_grid_index = self.grid.coordinates_to_grid_index(point)
            self.grid.expand_to_grid_index(point_grid_index)

            robot_array_index = self.grid.coordinates_to_array_index(robot_position)
            point_array_index = self.grid.grid_index_to_array_index(point_grid_index)

            self.mark_point_as_seen_by_lidar(robot_array_index, point_array_index)

        self.calculate_seen_walls()

    def calculate_seen_walls(self):
        """카메라 시야와 라이다 벽 감지를 교차하여 카메라로 본 벽/못 본 벽을 분류합니다."""
        self.grid.arrays["walls_seen_by_camera"] = (
            self.grid.arrays["seen_by_camera"] * self.grid.arrays["walls"])
        self.grid.arrays["walls_not_seen_by_camera"] = np.logical_xor(
            self.grid.arrays["walls"],
            self.grid.arrays["walls_seen_by_camera"])

    def generate_navigation_margins(self):
        """
        벽 주변으로 로봇 크기만큼의 통과 불가 영역(traversable)과
        경로 탐색 선호도 지도(navigation_preference)를 생성합니다.
        """
        occupied_as_int = self.grid.arrays["occupied"].astype(np.uint8)
        diameter_template_as_int = self.robot_diameter_template.astype(np.uint8)

        # 각 벽 픽셀 주변 로봇 직경 범위를 traversable(통과 불가)로 표시
        self.grid.arrays["traversable"] = np.zeros_like(self.grid.arrays["traversable"])
        self.grid.arrays["traversable"] = cv.filter2D(occupied_as_int, -1, diameter_template_as_int)
        self.grid.arrays["traversable"] = self.grid.arrays["traversable"].astype(np.bool_)

        # 벽 근처 픽셀에 높은 선호도 값 부여 (경로 탐색 시 회피 유도)
        self.grid.arrays["navigation_preference"] = cv.filter2D(occupied_as_int, -1, self.preference_template)
        self.grid.arrays["navigation_preference"][self.grid.arrays["swamps"]] = 150  # 늪지대도 회피

    def filter_out_noise(self):
        """감지 횟수가 delete_threshold 이하인 포인트를 노이즈로 제거합니다."""
        self.grid.arrays["detected_points"] = (
            self.grid.arrays["detected_points"] *
            (self.grid.arrays["detected_points"] > self.delete_threshold))

    def __generate_quadratic_circle_gradient(self, min_radius, max_radius):
        """벽 근처일수록 값이 커지는 2차 원형 그라디언트 템플릿을 생성합니다."""
        min_radius = round(min_radius)
        max_radius = round(max_radius)
        template = np.zeros((max_radius * 2 + 1, max_radius * 2 + 1), dtype=np.float32)
        for i in range(max_radius, min_radius, -1):
            template = cv.circle(template, (max_radius, max_radius), i,
                                 max_radius ** 2 - i ** 2, -1)
        return template * 0.1

    def __generate_linear_circle_gradient(self, min_radius, max_radius):
        """선형 원형 그라디언트 템플릿 (현재 미사용)."""
        min_radius = round(min_radius)
        max_radius = round(max_radius)
        template = np.zeros((max_radius * 2 + 1, max_radius * 2 + 1), dtype=np.float32)
        for i in range(max_radius, min_radius, -1):
            template = cv.circle(template, (max_radius, max_radius), i, max_radius - i, -1)
        return template * 0.5

    def occupy_point(self, point_array_index):
        """
        해당 위치의 감지 카운터를 증가시키고, 임계값 초과 시 벽으로 확정합니다.
        단, 이미 로봇이 지나간 곳(traversed)은 벽으로 등록하지 않습니다.
        """
        if not self.grid.arrays["walls"][point_array_index[0], point_array_index[1]]:
            self.grid.arrays["detected_points"][point_array_index[0], point_array_index[1]] += 1

            if self.grid.arrays["detected_points"][point_array_index[0], point_array_index[1]] > self.to_boolean_threshold:
                if not self.grid.arrays["traversed"][point_array_index[0], point_array_index[1]]:
                    self.grid.arrays["walls"][point_array_index[0], point_array_index[1]] = True

    def mark_point_as_seen_by_lidar(self, robot_array_index, point_array_index):
        """로봇 위치에서 포인트까지의 선을 seen_by_lidar 레이어에 그립니다."""
        self.grid.arrays["seen_by_lidar"] = self.__draw_bool_line(
            self.grid.arrays["seen_by_lidar"], robot_array_index, point_array_index)

    def __draw_bool_line(self, array, point1, point2):
        """Bresenham 직선 알고리즘으로 두 점 사이의 직선 경로를 True로 표시합니다."""
        indexes = skimage.draw.line(point1[0], point1[1], point2[0], point2[1])
        array[indexes[0][:-2], indexes[1][:-2]] = True  # 끝 2픽셀 제외 (포인트 자체는 제외)
        return array

    def __reset_seen_by_lidar(self):
        """매 프레임 시작 시 라이다 시야 레이어를 초기화합니다."""
        self.grid.arrays["seen_by_lidar"] = np.zeros_like(self.grid.arrays["seen_by_lidar"])
