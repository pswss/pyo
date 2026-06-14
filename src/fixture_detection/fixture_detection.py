from data_structures.vectors import Position2D, Vector2D
from data_structures.angle import Angle
from typing import List
from robot.devices.camera import CameraImage
from fixture_detection.color_filter import ColorFilter
from raster_drawing import line_indices

import copy

import math

import numpy as np
import cv2 as cv

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from flags import SHOW_FIXTURE_DEBUG

class FixtureDetector:
    """
    카메라 이미지에서 2026 wall token(letter victim/cognitive target)의 위치를 픽셀 그리드에 매핑하는 클래스입니다.

    동작 흐름:
    1. get_fixture_positions_in_image(): 이미지에서 fixture 후보의 이미지 내 위치(픽셀 좌표) 추출
    2. get_fixture_positions_and_angles(): 이미지 내 위치 → 실제 공간 좌표 + 수평 각도 변환
       - 카메라의 수평 시야각(FOV)을 이용해 이미지 x좌표 → 각도 변환
       - 픽셀 선을 따라 벽까지 레이 캐스팅하여 실제 위치 추정
    3. map_fixtures(): 각 카메라 이미지에서 fixtures를 감지하여 victims 레이어에 기록
    4. mark_reported_fixture(): 보고된 위치를 fixture_detection 레이어에 원으로 표시
    """
    def __init__(self, pixel_grid: CompoundExpandablePixelGrid) -> None:
        self.pixel_grid = pixel_grid

        # fixture 색상 필터: 2026 letter victim + cognitive target(K/R/Y/G/B)
        self.colors = ("black", "white", "yellow", "red", "green", "blue")
        self.passive_map_colors = ("yellow", "red", "green", "blue")
        self.color_filters = {
            "black":  ColorFilter(lower_hsv=(0, 0, 0),      upper_hsv=(179, 80, 120)),
            "white":  ColorFilter(lower_hsv=(0, 0, 170),    upper_hsv=(179, 80, 255)),
            "yellow": ColorFilter(lower_hsv=(18, 80, 80),   upper_hsv=(40, 255, 255)),
            "red":    ColorFilter(lower_hsv=(160, 80, 80),  upper_hsv=(179, 255, 255)),
            "green":  ColorFilter(lower_hsv=(45, 80, 80),   upper_hsv=(85, 255, 255)),
            "blue":   ColorFilter(lower_hsv=(95, 80, 80),   upper_hsv=(130, 255, 255)),
        }
        self.red_low_filter = ColorFilter(lower_hsv=(0, 80, 80), upper_hsv=(10, 255, 255))

        # 벽 색상 필터 (벽 내부 영역 마스킹에 사용)
        self.wall_color_filter = ColorFilter((90, 44,  0), (95, 213, 158))

        # fixture 감지 최대 거리: 0.12m * 5 = 0.6m
        self.max_detection_distance = 0.12 * 5
        self.max_fixtures_per_camera = 1
        self.min_candidate_area = 50
        self.required_candidate_hits = 4
        self.candidate_bin_size = 3
        self.candidate_hits = {}

    def get_wall_mask(self, image: np.ndarray):
        """
        이미지에서 벽 내부를 채운 마스크를 반환합니다.
        fixture는 벽 안에 있으므로 벽 내부 영역만 fixture 후보로 인정합니다.
        """
        margin = 1
        raw_wall = self.wall_color_filter.filter(image)

        # 좌우 여백 추가 (이미지 가장자리의 벽 처리)
        wall = np.ones(shape=(raw_wall.shape[0], raw_wall.shape[1] + margin * 2), dtype=np.uint8) * 255

        wall[:, margin: -margin] = raw_wall

        conts, _ = cv.findContours(wall, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        debug = np.copy(image)

        filled_wall = np.zeros_like(wall, dtype=np.bool_)

        # 각 벽 윤곽선 내부 채움
        for c in conts:
            this_cont = np.zeros_like(wall, dtype=np.uint8)
            cv.fillPoly(this_cont, [c,], 255)
            filled_wall += this_cont > 0

        filled_wall = filled_wall[:, margin:-margin]

        return filled_wall

    def get_fixture_positions_and_angles(self, robot_position: Position2D, camera_image: CameraImage) -> list:
        """
        카메라 이미지 내의 fixture 위치를 실제 공간 좌표와 각도로 변환합니다.

        동작:
        1. 이미지를 좌우 반전하여 fixture 위치(픽셀 좌표) 추출
        2. 이미지 x좌표 → 카메라 수평 FOV 기반 수평 각도 변환
        3. 카메라 위치에서 fixture 방향으로 레이 캐스팅
        4. 레이가 벽(walls 레이어)에 처음 닿는 위치의 이전 픽셀을 fixture 위치로 기록
        """
        # 이미지 좌우 반전 후 fixture 픽셀 위치 추출
        positions_in_image = self.get_fixture_positions_in_image(np.flip(camera_image.image, axis=1))

        fixture_positions = []
        fixture_angles = []
        for position in positions_in_image:
            # 이미지 내 x좌표 → 카메라 FOV 기반 상대 수평 각도 계산
            relative_horizontal_angle = Angle(position[1] * (camera_image.data.horizontal_fov.radians / camera_image.data.width))

            # 로봇 전역 좌표계에서의 fixture 수평 방향 각도
            fixture_horizontal_angle = (relative_horizontal_angle - camera_image.data.horizontal_fov / 2) + camera_image.data.horizontal_orientation

            fixture_horizontal_angle.normalize()

            # 카메라의 실제 공간 위치 계산 (로봇 중심 + 카메라 오프셋)
            camera_vector = Vector2D(camera_image.data.horizontal_orientation, camera_image.data.distance_from_center)
            camera_pos = camera_vector.to_position()
            camera_pos += robot_position

            # 최대 감지 거리만큼의 감지 벡터 끝점 계산
            detection_vector = Vector2D(fixture_horizontal_angle, self.max_detection_distance)
            detection_pos = detection_vector.to_position()

            detection_pos += camera_pos

            # 카메라 위치와 감지 끝점을 배열 인덱스로 변환
            camera_array_index = self.pixel_grid.coordinates_to_array_index(camera_pos)
            detection_array_index = self.pixel_grid.coordinates_to_array_index(detection_pos)

            # 카메라→감지점 사이의 모든 픽셀 좌표 계산
            line_xx, line_yy = line_indices(camera_array_index[0], camera_array_index[1], detection_array_index[0], detection_array_index[1])

            index = 0
            for x, y in zip(line_xx, line_yy):
                if x >= 0 and y >= 0 and x < self.pixel_grid.array_shape[0] and y < self.pixel_grid.array_shape[1]:
                    back_index = index - 2
                    back_index = max(back_index, 0)
                    # 레이가 벽에 닿으면 직전 위치(벽 앞)를 fixture 위치로 기록
                    if self.pixel_grid.arrays["walls"][x, y]:
                        x1 = line_xx[back_index]
                        y1 = line_yy[back_index]
                        fixture_positions.append(self.pixel_grid.array_index_to_coordinates(np.array([x1, y1])))
                        fixture_angles.append(copy.deepcopy(fixture_horizontal_angle))
                        break
                index += 1

        return fixture_positions, fixture_angles

    def get_fixture_positions_in_image(self, image: np.ndarray) -> List[Position2D]:
        """
        이미지에서 fixture 후보의 이미지 내 위치(픽셀 중심점)를 반환합니다.

        동작:
        1. 2026 token 색상 필터를 OR 합산하여 이진 이미지 생성
        2. 벽 마스크 적용 (벽 내부 영역만)
        3. 윤곽선 감지 후 경계 박스 중심점 계산
        """
        # 지도에 미리 찍는 후보는 cognitive target의 유채색 링만 사용한다.
        # 검정/흰색은 일반 벽 음영과 섞여 오탐이 많아 실제 보고 단계에서만 판별한다.
        image_sum = np.zeros(image.shape[:2], dtype=np.bool_)
        for name in self.passive_map_colors:
            filter = self.color_filters[name]
            color_image = filter.filter(image)
            if name == "red":
                color_image += self.red_low_filter.filter(image)
                color_image[color_image > 255] = 255
            image_sum += color_image > 0

        image_sum = image_sum.astype(np.uint8) * 255

        # 벽 마스크 적용 (벽 내부에 있는 픽셀만 유효)
        wall_mask = self.get_wall_mask(image)

        image_sum *= wall_mask

        if SHOW_FIXTURE_DEBUG:
            cv.imshow("fixtures", image_sum)

        # 윤곽선 감지 후 각 윤곽선의 경계 박스 중심점을 fixture 위치로 반환
        contours, _ = cv.findContours(image_sum, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        final_victims = []
        contours = sorted(contours, key=cv.contourArea, reverse=True)
        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            area = cv.contourArea(c)
            if area < self.min_candidate_area:
                continue
            if w < 5 or h < 5:
                continue
            if w > image.shape[1] * 0.75 or h > image.shape[0] * 0.75:
                continue
            aspect = w / max(h, 1)
            if aspect < 0.35 or aspect > 2.8:
                continue
            # 경계 박스 중심점을 fixture 위치로 설정
            final_victims.append(Position2D((x + x + w) / 2, (y + y + h) / 2))
            if len(final_victims) >= self.max_fixtures_per_camera:
                break

        if SHOW_FIXTURE_DEBUG:
            debug = copy.deepcopy(image)
            for f in final_victims:
                debug = cv.circle(debug, np.array(f, dtype=int), 3, (255, 0, 0), -1)

            cv.imshow("victim_pos_debug", debug)

        return final_victims

    def map_fixtures(self, camera_images, robot_position):
        """
        모든 카메라 이미지에서 fixture를 감지하여
        victims 레이어에 위치를, victim_angles 레이어에 감지 각도를 기록합니다.
        """
        for i in camera_images:
            positions, angles = self.get_fixture_positions_and_angles(robot_position, i)
            for pos, angle in zip(positions, angles):
                index = self.pixel_grid.coordinates_to_array_index(pos)
                if index[0] < 0 or index[1] < 0 or index[0] >= self.pixel_grid.array_shape[0] or index[1] >= self.pixel_grid.array_shape[1]:
                    continue
                nearby = self.pixel_grid.arrays["victims"][
                    max(index[0] - 4, 0):min(index[0] + 5, self.pixel_grid.array_shape[0]),
                    max(index[1] - 4, 0):min(index[1] + 5, self.pixel_grid.array_shape[1])]
                if np.any(nearby):
                    continue
                candidate_key = (index[0] // self.candidate_bin_size, index[1] // self.candidate_bin_size)
                self.candidate_hits[candidate_key] = self.candidate_hits.get(candidate_key, 0) + 1
                if len(self.candidate_hits) > 500:
                    self.candidate_hits.clear()
                    self.candidate_hits[candidate_key] = 1
                if self.candidate_hits[candidate_key] < self.required_candidate_hits:
                    continue
                # victims 레이어: fixture 감지 위치 마킹
                self.pixel_grid.arrays["victims"][index[0], index[1]] = True
                # victim_angles 레이어: 감지 방향 각도 저장 (보고 방향 결정에 사용)
                self.pixel_grid.arrays["victim_angles"][index[0], index[1]] = angle.radians

    def mark_reported_fixture(self, robot_position, fixture_position):
        """
        보고가 완료된 fixture 위치를 fixture_detection 레이어에 원(반경 4픽셀)으로 표시합니다.
        이미 보고된 위치를 재보고하지 않도록 하는 역할을 합니다.
        """
        fixture_array_index = self.pixel_grid.coordinates_to_array_index(fixture_position)
        detection = self.pixel_grid.arrays["fixture_detection"]
        mask = np.zeros_like(detection, dtype=np.uint8)
        center = (int(fixture_array_index[1]), int(fixture_array_index[0]))
        cv.circle(mask, center, 4, 1, -1)
        detection[mask.astype(np.bool_)] = True
