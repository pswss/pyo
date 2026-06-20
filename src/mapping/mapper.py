import math
from copy import copy, deepcopy

import numpy as np
import cv2 as cv

from data_structures.vectors import Position2D
from data_structures.angle import Angle

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid

from mapping.wall_mapper import WallMapper
from mapping.floor_mapper import FloorMapper
from mapping.occupied_mapping import OccupiedMapper

from mapping.array_filtering import ArrayFilterer

from mapping.robot_mapper import RobotMapper
from mapping.fixture_mapper import FixtureMapper
from mapping.area_tracker import AreaTracker, classify_passage_patch

from fixture_detection.fixture_detection import FixtureDetector

from flags import SHOW_DEBUG, SHOW_GRANULAR_NAVIGATION_GRID, DO_WAIT_KEY, SHOW_LIVE_MAP


class Mapper:
    """
    로봇이 탐색하며 얻는 모든 센서 데이터를 통합하여 지도(맵)를 생성하는 핵심 클래스입니다.

    매 타임스텝 update()가 호출되면 다음 작업을 순서대로 수행합니다:
    1. 라이다 포인트클라우드 → 벽/장애물 등록 (WallMapper)
    2. 로봇 경로 기록 (RobotMapper)
    3. 조난자 감지 영역 생성 (FixtureMapper)
    4. 카메라 이미지 → 바닥 색상 분석 (FloorMapper)
    5. 카메라 이미지 → 조난자 위치 지도 등록 (FixtureDetector)
    6. 점유(occupied) 영역 계산 (OccupiedMapper)
    7. 노이즈 제거 (ArrayFilterer)
    """
    def __init__(self, tile_size, robot_diameter, camera_distance_from_center):
        self.tile_size = tile_size                          # 지도 한 타일의 크기(m), 예: 0.12m
        self.quarter_tile_size = tile_size / 2             # 반 타일 크기 (픽셀 해상도 계산용)
        self.robot_diameter = robot_diameter               # 로봇 직경(m)

        self.robot_position = None      # 현재 로봇 위치 (매 update 시 갱신)
        self.robot_orientation = None   # 현재 로봇 방향
        self.start_position = None      # 미션 시작 위치 (register_start로 등록)
        self.robot_grid_index = None    # 로봇 위치의 그리드 인덱스

        # Area 1~4 추적 (통로 횡단 감지). Area4 체류 위치는 최종 행렬 '*' 채움에 사용.
        self.area_tracker = AreaTracker()

        # -------- 픽셀 그리드 초기화 --------
        pixels_per_tile = 10           # 타일 한 칸당 픽셀 수
        self.pixel_grid = CompoundExpandablePixelGrid(
            initial_shape=np.array([1, 1]),
            pixel_per_m=pixels_per_tile / self.quarter_tile_size,
            robot_radius_m=(self.robot_diameter / 2) - 0.008)

        # -------- 각 역할별 맵퍼 초기화 --------
        self.wall_mapper = WallMapper(self.pixel_grid, robot_diameter)
        self.floor_mapper = FloorMapper(
            pixel_grid=self.pixel_grid,
            tile_resolution=pixels_per_tile * 2,
            tile_size=self.tile_size,
            camera_distance_from_center=camera_distance_from_center)

        self.occupied_mapper = OccupiedMapper(self.pixel_grid)
        self.filterer = ArrayFilterer()

        self.robot_mapper = RobotMapper(
            pixel_grid=self.pixel_grid,
            robot_diameter=self.robot_diameter,
            pixels_per_m=pixels_per_tile / self.quarter_tile_size)

        self.fixture_mapper = FixtureMapper(
            pixel_grid=self.pixel_grid,
            tile_size=self.tile_size)

        self.fixture_detector = FixtureDetector(self.pixel_grid)

        self.time = 0   # 현재 시뮬레이션 시간 (초)

        # 실시간 시각화 (SHOW_LIVE_MAP=1 시 MapVisualizer 인스턴스가 여기에 등록됨)
        self.visualizer = None
        if SHOW_LIVE_MAP:
            from map_visualizer import MapVisualizer
            self.visualizer = MapVisualizer(self)

    def update(self, in_bounds_point_cloud: list = None,
               out_of_bounds_point_cloud: list = None,
               lidar_detections: list = None,
               camera_images: list = None,
               robot_position: Position2D = None,
               robot_orientation: Angle = None,
               time=None):
        """
        매 타임스텝(또는 호출 시)마다 지도를 업데이트합니다.
        흔들릴 때는 라이다 없이 카메라 데이터만으로 호출되기도 합니다.
        """
        if time is not None:
            self.time = time

        if robot_position is None or robot_orientation is None:
            return

        self.robot_position = robot_position
        self.robot_orientation = robot_orientation
        self.robot_grid_index = self.pixel_grid.coordinates_to_grid_index(self.robot_position)

        # 1. 라이다 포인트클라우드로 벽 등록
        if in_bounds_point_cloud is not None and out_of_bounds_point_cloud is not None:
            self.wall_mapper.load_point_cloud(in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position)

        # 2. 로봇 위치 기록 (traversed, seen_by_camera, discovered 레이어 업데이트)
        self.robot_mapper.map_traversed_by_robot(self.robot_grid_index)
        self.robot_mapper.map_seen_by_camera(self.robot_grid_index, self.robot_orientation)
        self.robot_mapper.map_discovered_by_robot(self.robot_grid_index, self.robot_orientation)

        # 3. 조난자 도달 가능 마진 영역 갱신
        self.fixture_mapper.generate_detection_zone()
        self.fixture_mapper.clean_up_fixtures()

        # 4. 바닥 색상 분석 (구멍, 늪지대, 체크포인트 감지)
        if camera_images is not None:
            self.floor_mapper.map_floor(camera_images, self.pixel_grid.coordinates_to_grid_index(self.robot_position))

        # 5. 카메라에서 조난자 위치 추정 및 지도 등록
        if camera_images is not None and lidar_detections is not None:
            self.fixture_detector.map_fixtures(camera_images, self.robot_position)

        # 5-1. 구역(Area 1~4) 추적: 로봇 발밑 바닥색으로 통로 위 여부 판별
        robot_array_index = self.pixel_grid.grid_index_to_array_index(self.robot_grid_index)
        half = 4   # 발밑 8×8px(≈5cm) 패치 — 타일 경계 걸침 영향 최소화
        r0 = max(robot_array_index[0] - half, 0)
        c0 = max(robot_array_index[1] - half, 0)
        patch = self.pixel_grid.arrays["floor_color"][r0:r0 + half * 2, c0:c0 + half * 2]
        self.area_tracker.step(classify_passage_patch(patch), self.robot_position)

        # 6. 점유 영역(벽 OR 구멍) 계산
        self.occupied_mapper.map_occupied()

        # 7. 라이다 노이즈(고립 포인트) 제거
        self.filterer.remove_isolated_points(self.pixel_grid)

        if self.visualizer is not None:
            self.visualizer.update()

        if DO_WAIT_KEY:
            cv.waitKey(1)

    def register_start(self, robot_position):
        """미션 시작 위치를 기록합니다. 나중에 복귀 경로 계산에 사용됩니다."""
        self.start_position = deepcopy(robot_position)
        print(f"[맵퍼:mapper.register_start] 시작 위치 등록: ({self.start_position.x:.4f}, {self.start_position.y:.4f})m, 그리드={self.pixel_grid.coordinates_to_grid_index(self.start_position)}")

    def has_detected_victim_from_position(self):
        """현재 로봇 위치에서 이미 조난자를 보고한 적 있는지 확인합니다 (중복 보고 방지)."""
        robot_array_index = self.pixel_grid.grid_index_to_array_index(self.robot_grid_index)
        return self.pixel_grid.arrays["robot_detected_fixture_from"][robot_array_index[0], robot_array_index[1]]

    # 전방 구멍(블랙홀) 검사 파라미터. 검사점 = 전방 6cm, 반경 2.5cm. ★시뮬 튜닝
    hole_check_distance = 0.06
    hole_check_radius = 0.025

    def _front_point(self, position, orientation, distance):
        """로봇 전방 distance(m) 지점의 절대좌표. (전방 = (sinθ, cosθ), get_angle_to 규약)"""
        return (position.x + distance * math.sin(orientation.radians),
                position.y + distance * math.cos(orientation.radians))

    def is_hole_in_front(self, robot_position, robot_orientation) -> bool:
        """전방 검사점 주변에 구멍(holes) 픽셀이 있으면 True — 회피 기동 트리거용."""
        fx, fy = self._front_point(robot_position, robot_orientation, self.hole_check_distance)
        idx = self.pixel_grid.coordinates_to_array_index(np.array([fx, fy]))
        r = round(self.hole_check_radius * self.pixel_grid.resolution)
        holes = self.pixel_grid.arrays["holes"]
        min_x = max(idx[0] - r, 0)
        max_x = min(idx[0] + r + 1, holes.shape[0])
        min_y = max(idx[1] - r, 0)
        max_y = min(idx[1] + r + 1, holes.shape[1])
        if min_x >= max_x or min_y >= max_y:
            return False
        return bool(np.any(holes[min_x:max_x, min_y:max_y]))

    def is_hole_between(self, pos_a, pos_b) -> bool:
        """a→b 직선 경로가 구멍 픽셀을 지나는지 — 웨이포인트·구멍 겹침 판정용.
        겹칠 때만 회피 기동(후진+180°)을 발동하고, 아니면 경로계획 우회에 맡긴다."""
        dist = pos_a.get_distance_to(pos_b)
        if dist < 1e-6:
            return False
        steps = max(int(dist / 0.01), 1)
        holes = self.pixel_grid.arrays["holes"]
        for i in range(steps + 1):
            t = i / steps
            x = pos_a.x + (pos_b.x - pos_a.x) * t
            y = pos_a.y + (pos_b.y - pos_a.y) * t
            idx = self.pixel_grid.coordinates_to_array_index(np.array([x, y]))
            if (0 <= idx[0] < holes.shape[0] and 0 <= idx[1] < holes.shape[1]
                    and holes[idx[0], idx[1]]):
                return True
        return False

    def is_close_to_swamp(self):
        """
        로봇 근처(약 2cm 반경)에 늪지대가 있는지 확인합니다.
        늪지대 근처에서는 GPS 오차가 커지므로 PoseManager가 자이로만 사용하도록 전환합니다.
        """
        if self.robot_grid_index is None:
            return False

        swamp_check_area = 0.02
        swamp_check_area_px = round(swamp_check_area * self.pixel_grid.resolution)

        robot_array_index = self.pixel_grid.grid_index_to_array_index(self.robot_grid_index)

        min_x = max(robot_array_index[0] - swamp_check_area_px, 0)
        max_x = min(robot_array_index[0] + swamp_check_area_px, self.pixel_grid.array_shape[0])
        min_y = max(robot_array_index[1] - swamp_check_area_px, 0)
        max_y = min(robot_array_index[1] + swamp_check_area_px, self.pixel_grid.array_shape[1])

        return np.any(self.pixel_grid.arrays["swamps"][min_x:max_x, min_y:max_y])
