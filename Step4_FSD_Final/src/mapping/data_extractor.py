import numpy as np
import cv2 as cv
import copy

import utilities

from flags import SHOW_DEBUG

class FloorColorExtractor:
    """
    바닥 이미지에서 각 타일의 색상 종류를 추출하는 클래스입니다.
    (현재는 FloorMapper로 대체되어 직접 사용되지 않을 수 있습니다)

    각 타일 영역의 HSV 색상 분포를 분석하여
    normal/hole/swamp/checkpoint 등의 타일 종류를 판별합니다.
    """
    def __init__(self, tile_resolution) -> None:
        self.tile_resolution = tile_resolution
        # 타일 색상 분류 기준: HSV 범위와 비율 임계값
        self.floor_color_ranges = {
                    "normal":
                        {
                            "range":   ((0, 0, 37), (0, 0, 192)),
                            "threshold":0.2},

                    "nothing":
                        {
                            "range":((100, 0, 0), (101, 1, 1)),
                            "threshold":0.9},

                    "checkpoint":
                        {
                            "range":((95, 0, 65), (128, 122, 198)),
                            "threshold":0.2},
                    "hole":
                        {
                            "range":((0, 0, 10), (0, 0, 30)),
                            "threshold":0.2},

                    "swamp":
                        {
                            "range":((19, 112, 32), (19, 141, 166)),
                            "threshold":0.2},

                    # 레벨 간 연결 통로 타일 (색상으로 판별)
                    "connection1-2":
                        {
                            "range":((120, 182, 49), (120, 204, 232)),
                            "threshold":0.2},

                    "connection1-3":
                        {
                            "range":((132, 156, 36), (133, 192, 185)),
                            "threshold":0.2},

                    "connection2-3":
                        {
                            "range":((0, 182, 49), (0, 204, 232)),
                            "threshold":0.2},
                    }
        self.final_image = np.zeros((700, 700, 3), np.uint8)

    def get_square_color(self, image, square_points):
        """
        지정된 사각형 영역의 HSV 색상을 분석하여 타일 종류 문자열을 반환합니다.
        비율 임계값을 초과하는 색상 중 가장 많이 검출된 색상을 선택합니다.
        """
        square = image[square_points[0]:square_points[1], square_points[2]:square_points[3]]
        square = cv.cvtColor(square, cv.COLOR_BGR2HSV)
        # 빈 타일이면 "nothing" 반환
        if np.count_nonzero(square) == 0:
            return "nothing"
        color_counts = {}
        for color_key, color_range in self.floor_color_ranges.items():
            colour_count = np.count_nonzero(cv.inRange(square, color_range["range"][0], color_range["range"][1]))
            # 임계값 비율을 초과해야만 해당 색상으로 인정
            if colour_count > color_range["threshold"] * square.shape[0] * square.shape[1]:
                color_counts[color_key] = colour_count

        if len(color_counts) == 0:
            return "nothing"
        else:
            return max(color_counts, key=color_counts.get)

    def get_sq_color(self, image, square_points):
        """
        사각형 영역에서 흰색/검은색 픽셀 수를 비교하여
        밝은 타일 (255,255,255) 또는 어두운 타일 (100,100,100)을 반환합니다.
        """
        square = image[square_points[0]:square_points[1], square_points[2]:square_points[3]]
        white_count = np.count_nonzero(cv.inRange(square, (180, 180, 180), (255, 255, 255)))
        black_count = np.count_nonzero(cv.inRange(square, (20, 20, 20), (180, 180, 180)))

        if white_count > black_count and white_count > square.shape[0] * square.shape[1] / 8:
            return (255, 255, 255)
        else:
            return (100, 100, 100)

    def get_floor_colors(self, floor_image, robot_position):
        """
        바닥 이미지를 타일 격자로 분할하여 각 타일의 색상 종류와 위치를 반환합니다.
        로봇 위치를 기준으로 격자 오프셋을 계산합니다.
        """
        # 로봇 위치 기반 격자 오프셋 계산 (타일이 로봇 위치에 정렬되도록)
        grid_offsets = [(((p + 0) % 0.06) / 0.06) * 50 for p in robot_position]

        grid_offsets = [int(o) for o in grid_offsets]

        offsets = [((((p + 0.03) % 0.06) - 0.03) / 0.06) * 50 for p in robot_position]

        offsets = [int(o) for o in offsets]


        utilities.save_image(floor_image, "floor_image.png")

        # 이미지를 타일 격자로 분할
        squares_grid = utilities.get_squares(floor_image, self.tile_resolution, offsets)

        color_tiles = []
        for row in squares_grid:
            for square in row:
                color_key = self.get_square_color(floor_image, square)
                # 타일 종류별 색상 설정 (최종 맵용)
                if color_key == "normal":
                    color = (255, 255, 255)
                elif color_key == "checkpoint":
                    color = (100, 100, 100)
                else:
                    color = (0, 0, 0)

                # 픽셀 좌표를 타일 인덱스로 변환
                tile = [square[2], square[0]]
                tile = utilities.substractLists(tile, (350 - offsets[0], 350 - offsets[1]))
                tile = utilities.divideLists(tile, [self.tile_resolution, self.tile_resolution])
                tile = [int(t) for t in tile]
                if color_key != "nothing":
                    if SHOW_DEBUG:
                        print(tile, color_key)
                    color_tiles.append((tile, color_key))

        # 디버그: 격자와 로봇 위치를 시각화
        if SHOW_DEBUG:
            drawing_image = floor_image.copy()
            utilities.draw_grid(drawing_image, self.tile_resolution, offset=grid_offsets)
            cv.circle(drawing_image, (350 - offsets[0], 350 - offsets[1]), 10, (255, 0, 0), -1)
            cv.imshow("final_floor_image", utilities.resize_image_to_fixed_size(drawing_image, (600, 600)))
        return color_tiles


class PointCloudExtarctor:
    """
    포인트 클라우드를 타일 단위로 분석하여 벽의 방향(상하좌우, 코너)을 추출하는 클래스입니다.
    (현재 직접 사용되지 않을 수 있으며, WallMapper로 대체되었을 가능성 있음)

    직선 벽과 코너(꺾인 벽) 템플릿과의 매칭 점수로 벽 방향을 판별합니다.
    """
    def __init__(self, resolution):
        self.threshold = 8   # 템플릿 매칭 최소 점수 임계값
        self.resolution = resolution

        # 직선 벽 검출 템플릿 (위쪽 2행이 1과 2로 가중치 부여)
        straight = [
            [0, 1, 2, 2, 2, 1, 0],
            [0, 1, 2, 2, 2, 1, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
                ]

        self.straight_template = np.array(straight)

        # 코너(꺾인 벽) 검출 템플릿
        curved = [
            [0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 1, 1, 0],
            [0, 0, 3, 1, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0],
            [1, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
                ]

        self.curved_template = np.array(curved)

        # 4방향 직선 + 4코너 방향의 회전된 템플릿 딕셔너리 생성
        self.templates = {}

        # 직선 템플릿: 상(u), 좌(l), 하(d), 우(r) 각 90도 회전
        for i, name in enumerate([("u",), ("l",), ("d",), ("r",)]):
            self.templates[name] = np.rot90(self.straight_template, i)

        # 코너 템플릿: 좌상(ul), 좌하(dl), 우하(dr), 우상(ur) 각 90도 회전
        for i, name in enumerate([("u", "l"), ("d", "l"), ("d", "r"),  ("u", "r")]):
            self.templates[name] = np.rot90(self.curved_template, i)

    def get_tile_status(self, min_x, min_y, max_x, max_y, point_cloud):
        """
        지정된 타일 영역에서 각 템플릿과의 매칭 점수를 계산하고,
        임계값 이상인 방향 목록을 반환합니다.
        """
        counts = {name: 0 for name in self.templates}
        square = point_cloud[min_x:max_x+1, min_y:max_y+1]
        # 타일 크기가 맞지 않으면 빈 목록 반환
        if square.shape != (self.resolution+1, self.resolution+1):
            return []

        # 포인트 클라우드의 비영 위치에서 각 템플릿 값을 합산
        non_zero_indices = np.where(square != 0)
        for name, template in self.templates.items():
            counts[name] = np.sum(template[non_zero_indices])

        # 임계값 이상인 방향만 반환
        names = [name for name, count in counts.items() if count >= self.threshold]

        return [i for sub in names for i in sub]

    def transform_to_grid(self, point_cloud):
        """
        전체 포인트 클라우드를 타일 단위로 분할하여
        각 타일의 벽 방향 목록으로 구성된 2D 그리드를 반환합니다.
        """
        offsets = point_cloud.offsets
        offsets = [o % self.resolution for o in offsets]
        offsets.reverse()
        grid = []
        bool_array_copy = point_cloud.get_bool_array()
        if SHOW_DEBUG:
            bool_array_copy = bool_array_copy.astype(np.uint8) * 100
        for x in range(offsets[0], bool_array_copy.shape[0] - self.resolution, self.resolution):
            row = []
            for y in range(offsets[1], bool_array_copy.shape[1] - self.resolution, self.resolution):
                min_x = x
                min_y = y
                max_x = x + self.resolution
                max_y = y + self.resolution

                if SHOW_DEBUG:
                    bool_array_copy = cv.rectangle(bool_array_copy, (min_y, min_x), (max_y, max_x), (255,), 1)

                val = self.get_tile_status(min_x, min_y, max_x, max_y, point_cloud.get_bool_array())

                row.append(list(val))
            grid.append(row)

        if SHOW_DEBUG:
            cv.imshow("point_cloud_with_squares", utilities.resize_image_to_fixed_size(bool_array_copy, (600, 600)))
        offsets = point_cloud.offsets
        return grid, [o // self.resolution for o in offsets]
