from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from data_structures.vectors import Position2D
import copy
import math
import skimage

import numpy as np
import cv2 as cv

from flags import SHOW_MAP_AT_END, DO_SAVE_FINAL_MAP, SAVE_FINAL_MAP_DIR, DO_SAVE_DEBUG_GRID, SAVE_DEBUG_GRID_DIR

import time

class WallMatrixCreator:
    """
    픽셀 단위의 벽 배열을 타일 기반 노드 배열로 변환하는 클래스입니다.

    Webots 대회 서버에 제출할 최종 맵은 각 타일 경계를 노드로 표현합니다.
    이 클래스는 벽 픽셀 배열을 분석하여 각 타일에 어느 방향(상하좌우, 코너)의
    벽이 있는지를 2×2 노드 그리드로 변환합니다.

    직선 벽 템플릿과 코너 벽 템플릿을 이용해 각 타일의 벽 방향을 판별합니다.
    임계값(threshold=10) 이상의 매칭 점수가 있어야 벽으로 인정합니다.
    """
    def __init__(self, square_size_px: int):
        # 벽으로 인정하는 최소 템플릿 매칭 점수.
        # 벽 thinning(wall_mapper) 도입으로 벽이 1px 선으로 수렴 → 직선 풀매칭 합=10, 코너 합=9라
        # 기존 10은 코너를 놓침. 7로 낮춰 1px 벽도 매칭(직선 10≥7, 코너 9≥7). ★시뮬 튜닝
        self.threshold = 7
        self.__square_size_px = square_size_px

        # 직선 벽 검출 템플릿 (위쪽 2행에 가중치 1, 2)
        straight = [
            [0, 0, 1, 2, 2, 2, 2, 1, 0, 0],
            [0, 0, 1, 2, 2, 2, 2, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                ]

        self.straight_template = np.array(straight)

        # 코너(꺾인 벽) 검출 템플릿 (좌상단 코너에 높은 가중치 3)
        vortex = [
            [3, 3, 3, 0, 0, 0, 0, 0, 0, 0],
            [3, 3, 3, 0, 0, 0, 0, 0, 0, 0],
            [3, 3, 3, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                ]

        self.vortex_template = np.array(vortex)

        # 4방향 직선 + 4코너 방향의 회전된 템플릿 딕셔너리 생성
        self.templates = {}

        # 직선 벽: (-1,0)=위, (0,-1)=좌, (1,0)=아래, (0,1)=우 방향으로 90도씩 회전
        for i, name in enumerate([(-1, 0), (0,-1), (1,0), (0,1)]):
            self.templates[name] = np.rot90(self.straight_template, i)

        # 코너 벽: (-1,-1)=좌상, (1,-1)=좌하, (1,1)=우하, (-1,1)=우상 방향으로 90도씩 회전
        for i, name in enumerate([(-1,-1), (1, -1), (1, 1), (-1, 1)]):
           self.templates[name] = np.rot90(self.vortex_template, i)


    def __get_tile_status(self, min_x, min_y, max_x, max_y, wall_array: np.ndarray) -> list:
        """
        지정된 타일 영역에서 각 방향 템플릿과의 매칭 점수를 계산하고,
        임계값 이상인 방향 목록을 반환합니다.
        """
        counts = {name: 0 for name in self.templates}
        square = wall_array[min_x:max_x, min_y:max_y]
        # 타일 크기가 맞지 않으면 빈 목록 반환
        if square.shape != (self.__square_size_px, self.__square_size_px):
            return []

        non_zero_indices = np.where(square != 0)
        for orientation, template in self.templates.items():
            counts[orientation] = np.sum(template[non_zero_indices])

        # 임계값 이상의 점수를 가진 방향만 반환
        status = []
        for orientation, count in counts.items():
            if count >= self.threshold:
                status.append(orientation)

        return status

    def get_obstacle_tiles(self, wall_array: np.ndarray, offsets: np.ndarray) -> list:
        """
        타일 중심 영역(25%~75% 내부)에 벽 픽셀이 임계값 이상 집중된 타일을 장애물 타일('x')로 판정합니다.
        규정상 장애물은 항상 타일 중앙에 위치하고 엣지 벽과 겹치지 않으므로
        중심 픽셀 군집으로 구분 가능합니다.
        """
        full_tile_px = self.__square_size_px * 2  # 12cm 타일 전체 = 20px
        c_start = full_tile_px // 4               # 5
        c_end = full_tile_px * 3 // 4             # 15
        threshold = 5

        obstacle_grid = []
        for x in range(offsets[0], wall_array.shape[0] - full_tile_px, full_tile_px):
            row = []
            for y in range(offsets[1], wall_array.shape[1] - full_tile_px, full_tile_px):
                square = wall_array[x:x + full_tile_px, y:y + full_tile_px]
                if square.shape != (full_tile_px, full_tile_px):
                    row.append(False)
                    continue
                row.append(int(np.sum(square[c_start:c_end, c_start:c_end])) >= threshold)
            obstacle_grid.append(row)
        return obstacle_grid

    def transform_wall_array_to_bool_node_array(self, wall_array: np.ndarray, offsets: np.ndarray) -> np.ndarray:
        """
        픽셀 단위 벽 배열을 타일 단위로 분할하여 벽 방향을 추출하고,
        이를 최종 노드 배열(2×타일수 크기)로 변환합니다.
        """
        grid = []
        if SHOW_MAP_AT_END:
            bool_array_copy = wall_array.astype(np.uint8) * 100
        # 타일 단위로 배열 순회
        for x in range(offsets[0], wall_array.shape[0] - self.__square_size_px, self.__square_size_px):
            row = []
            for y in range(offsets[1], wall_array.shape[1] - self.__square_size_px, self.__square_size_px):
                min_x = x
                min_y = y
                max_x = x + self.__square_size_px
                max_y = y + self.__square_size_px
                if SHOW_MAP_AT_END:
                    bool_array_copy = cv.rectangle(bool_array_copy, (min_y, min_x), (max_y, max_x), (255,), 1)

                val = self.__get_tile_status(min_x, min_y, max_x, max_y, wall_array)

                row.append(list(val))
            grid.append(row)

        if SHOW_MAP_AT_END:
            cv.imshow("point_cloud_with_squares", cv.resize(bool_array_copy, (0, 0), fx=1, fy=1, interpolation=cv.INTER_AREA))

        # 방향 그리드를 최종 벽 노드 배열로 변환
        grid = self.__orientation_grid_to_final_wall_grid(grid)

        return grid

    def __orientation_grid_to_final_wall_grid(self, orientation_grid: list) -> np.ndarray:
        """
        각 타일의 벽 방향 목록을 최종 노드 배열로 변환합니다.
        각 타일은 2×2 노드로 표현되며, 벽 방향에 따라 해당 노드를 True로 설정합니다.

        예: 타일 (y, x)의 위쪽 벽 → 노드 (y*2 + (-1), x*2 + 0) = True
        """
        shape = np.array([len(orientation_grid), len(orientation_grid[0])])
        shape *= 2  # 각 타일이 2×2 노드로 표현

        final_wall_grid = np.zeros(shape, dtype=np.bool_)

        for y, row in enumerate(orientation_grid):
            for x, value in enumerate(row):
                x1 = x * 2
                y1 = y * 2

                for orientation in value:
                    # orientation = (dy, dx) 방향 오프셋으로 노드 위치 계산
                    final_x = x1 + orientation[1]
                    final_y = y1 + orientation[0]

                    final_wall_grid[final_y, final_x] = True

        return final_wall_grid


class FloorMatrixCreator:
    """
    픽셀 단위의 바닥 색상 배열을 타일 기반 색상 코드 배열로 변환하는 클래스입니다.

    각 타일 영역의 HSV 색상 분포를 분석하여 대회 서버 제출 형식의
    타일 코드(0=일반, 2=구멍, 3=늪, 4=체크포인트 등)를 결정합니다.

    WallMatrixCreator와 달리 타일 크기의 2배를 사용합니다.
    """
    def __init__(self, square_size_px: int) -> None:
        self.__square_size_px = square_size_px * 2  # 바닥 타일은 2배 크기
        # 타일 코드별 HSV 범위와 비율 임계값
        # 규정 §5.6.10.b.ii: 통로 코드는 b/y/g/p/o/r "문자" (숫자 6/7/8 아님 — 기존
        # 숫자 제출은 통로 셀 전부 불일치로 채점됨). 라벨이 아닌 실측 hue 기준으로 매핑:
        # 파랑(H120)=b(1↔2), 보라(H132)=p(2↔3), 빨강(H0)=r(3↔4). 파/보/빨의 S/V 범위는
        # 기존 시뮬 실측값 유지. 노랑=y(1↔3)/초록=g(1↔4)/주황=o(2↔4)는 표준 hue 범위
        # 신규 추가. ★시뮬 튜닝
        self.__floor_color_ranges = {
                    "0": # 빈 공간 (void)
                        {
                            "range":((100, 0, 0), (101, 1, 1)),
                            "threshold":0.9},

                    "4": # 체크포인트
                        {
                            "range":((95, 0, 65), (128, 122, 198)),
                            "threshold":0.2},
                    "2": # 구멍
                        {
                            "range":((0, 0, 10), (0, 0, 30)),
                            "threshold":0.2},

                    "3": # 늪지대
                        {
                            "range":((19, 112, 32), (19, 141, 166)),
                            "threshold":0.2},

                    "b": # 통로 1↔2 (파랑)
                        {
                            "range":((120, 182, 49), (120, 204, 232)),
                            "threshold":0.2},

                    "p": # 통로 2↔3 (보라)
                        {
                            "range":((132, 156, 36), (133, 192, 185)),
                            "threshold":0.2},

                    "r": # 통로 3↔4 (빨강)
                        {
                            "range":((0, 150, 49), (5, 255, 232)),
                            "threshold":0.2},

                    "y": # 통로 1↔3 (노랑) ★시뮬 튜닝
                        {
                            "range":((25, 150, 80), (35, 255, 255)),
                            "threshold":0.2},

                    "g": # 통로 1↔4 (초록) ★시뮬 튜닝
                        {
                            "range":((50, 150, 50), (70, 255, 255)),
                            "threshold":0.2},

                    "o": # 통로 2↔4 (주황) ★시뮬 튜닝
                        {
                            "range":((10, 150, 80), (20, 255, 255)),
                            "threshold":0.2},
                    }

        self.final_image = np.zeros((700, 700, 3), np.uint8)

    def __get_square_color(self, min_x, min_y, max_x, max_y, floor_array: np.ndarray) -> str:
        """
        지정된 타일 영역의 HSV 색상을 분석하여 타일 코드 문자열을 반환합니다.
        비율 임계값을 초과하는 색상 중 가장 많이 검출된 코드를 선택합니다.
        """
        square = floor_array[min_x:max_x, min_y:max_y]

        square = cv.cvtColor(square, cv.COLOR_BGR2HSV)

        if np.count_nonzero(square) == 0:
            return "0"

        color_counts = {}
        for color_key, color_range in self.__floor_color_ranges.items():
            colour_count = np.count_nonzero(cv.inRange(square, color_range["range"][0], color_range["range"][1]))
            if colour_count > color_range["threshold"] * square.shape[0] * square.shape[1]:
                color_counts[color_key] = colour_count

        if len(color_counts) == 0:
            return "0"
        else:
            return max(color_counts, key=color_counts.get)


    def get_floor_colors(self, floor_array: np.ndarray, offsets: np.ndarray) -> np.ndarray:
        """
        바닥 색상 배열을 타일 단위로 분할하여 각 타일의 코드 문자열로 구성된
        2D 그리드를 반환합니다.
        """
        if SHOW_MAP_AT_END:
            array_copy = copy.deepcopy(floor_array)

        grid = []

        for x in range(offsets[0], floor_array.shape[0] - self.__square_size_px, self.__square_size_px):
            row = []
            for y in range(offsets[1], floor_array.shape[1] - self.__square_size_px, self.__square_size_px):
                min_x = x
                min_y = y
                max_x = x + self.__square_size_px
                max_y = y + self.__square_size_px

                if SHOW_MAP_AT_END:
                    array_copy = cv.rectangle(array_copy, (min_y, min_x), (max_y, max_x), (255, 255, 255), 1)

                color_key = self.__get_square_color(min_x, min_y, max_x, max_y, floor_array)

                row.append(color_key)
            grid.append(row)

        if SHOW_MAP_AT_END:
            cv.imshow("array copy", array_copy)

        return grid


class FinalMatrixCreator:
    """
    픽셀 그리드에서 대회 서버 제출용 최종 텍스트 매트릭스를 생성하는 클래스입니다.

    동작 흐름:
    1. WallMatrixCreator: 벽 배열 → 벽 노드 배열 (0/1 이진)
    2. FloorMatrixCreator: 바닥 색상 배열 → 타일 코드 배열
    3. __get_final_text_grid(): 벽 노드에 바닥 코드와 로봇 시작 위치(5)를 합성
    4. 각 타일은 4×4 노드로 표현되며, 코드는 타일 중심(+3 오프셋)에 배치됨

    최종 형식:
    - "0": 통로
    - "1": 벽
    - "2": 구멍
    - "3": 늪지대
    - "4": 체크포인트
    - "5": 로봇 시작 위치
    - "6"/"7"/"8": 레벨 간 연결 통로
    """
    def __init__(self, tile_size: float, resolution: float):
        # 타일 절반 크기를 픽셀 단위로 변환
        self.__square_size_px = round(tile_size / 2 * resolution)

        self.wall_matrix_creator = WallMatrixCreator(self.__square_size_px)
        self.floor_matrix_creator = FloorMatrixCreator(self.__square_size_px)


    def pixel_grid_to_final_grid(self, pixel_grid: CompoundExpandablePixelGrid, robot_start_position: np.ndarray,
                                 area4_positions=None, reported_fixtures=None) -> np.ndarray:
        """
        픽셀 그리드 전체를 최종 텍스트 매트릭스로 변환합니다.

        1. 벽 배열과 바닥 색상 배열 추출
        2. 필요 시 이미지 파일로 저장 (DO_SAVE_FINAL_MAP, DO_SAVE_DEBUG_GRID 플래그)
        3. 타일 오프셋 계산
        4. WallMatrixCreator로 벽 노드 배열 생성
        5. FloorMatrixCreator로 바닥 타일 코드 배열 생성
        6. 로봇 시작 위치를 타일 인덱스로 변환
        7. __get_final_text_grid()로 모든 정보를 하나의 텍스트 그리드로 합성
        """
        np.set_printoptions(linewidth=1000000000000, threshold=100000000000000)
        wall_array = pixel_grid.arrays["walls"]
        color_array = pixel_grid.arrays["floor_color"]

        # 디버그용 이미지 저장 (플래그가 True일 때만)
        if DO_SAVE_FINAL_MAP:
            cv.imwrite(f"{SAVE_FINAL_MAP_DIR}/WALL_PIXEL_GRID{str(time.time()).rjust(50)}.png", wall_array.astype(np.uint8) * 255)

        if DO_SAVE_DEBUG_GRID:
            cv.imwrite(f"{SAVE_DEBUG_GRID_DIR}/DEBUG_GRID{str(time.time()).rjust(50)}.png", (pixel_grid.get_colored_grid() * 255).astype(np.uint8))

        # 벽 배열 타일 오프셋 계산 (픽셀 그리드의 원점 오프셋을 타일 크기로 나눈 나머지)
        offsets = self.__get_offsets(self.__square_size_px, pixel_grid.offsets)

        # 벽 노드 배열 생성
        wall_node_array = self.wall_matrix_creator.transform_wall_array_to_bool_node_array(wall_array, offsets)

        # 바닥 색상 배열의 오프셋은 타일 크기의 2배 기준 (FloorMatrixCreator가 2배 크기 사용)
        floor_offsets = self.__get_offsets(self.__square_size_px * 2, pixel_grid.offsets + self.__square_size_px)

        # 바닥 타일 코드 배열 생성
        floor_string_array = self.floor_matrix_creator.get_floor_colors(color_array, floor_offsets)

        # 시작 위치 없으면 빈 배열 반환
        if robot_start_position is None:
            return np.array([])

        # 로봇 시작 위치를 노드 인덱스로 변환
        start_array_index = pixel_grid.coordinates_to_array_index(robot_start_position)
        start_array_index -= offsets
        # 타일 단위로 변환 후 2배 스케일링 (각 타일이 2노드이므로) - 1 보정
        robot_node = np.round((start_array_index / self.__square_size_px) * 2).astype(int) - 1

        # 장애물 타일 감지 (floor_offsets와 동일 기준 사용)
        obstacle_tile_array = self.wall_matrix_creator.get_obstacle_tiles(wall_array, floor_offsets)

        # 벽 노드 + 바닥 코드 + 시작 위치 + 장애물을 하나의 텍스트 그리드로 합성
        text_grid = self.__get_final_text_grid(wall_node_array, floor_string_array, robot_node, obstacle_tile_array)

        # Area 4 타일 '*' 채움 (규정 §5.6.10.c / MapAnswer.py: room==4 → 5×5 전부 '*').
        # '*' 미기록은 area4 셀 전부 불일치 = 최대 감점원 (world2 실측: 일치율 ~46%).
        if area4_positions:
            self.__fill_area4(text_grid, pixel_grid, offsets, area4_positions)

        # 보고한 토큰(victim H/S/U, hazmat F/P/C/O)을 사인이 붙은 벽 칸에 글자로 배치
        if reported_fixtures:
            self.__place_fixtures(text_grid, pixel_grid, offsets, reported_fixtures)

        return np.array(text_grid)

    def __place_fixtures(self, text_grid: list, pixel_grid: CompoundExpandablePixelGrid,
                         offsets: np.ndarray, reported_fixtures) -> None:
        """보고한 토큰을 최종 행렬에 글자로 기록합니다(규정: victim=H/S/U, hazmat=F/P/C/O).
        토큰 위치를 텍스트 격자 셀로 변환(area4와 동일 매핑) 후, 주변에서 가장 가까운
        벽('1') 칸을 글자로 덮습니다(토큰은 벽에 붙어 있으므로). 벽을 못 찾으면 추정 칸에 기록."""
        rows = len(text_grid)
        cols = len(text_grid[0]) if text_grid else 0
        placed = 0
        for pos, letter in reported_fixtures:
            if not letter:
                continue
            arr = pixel_grid.coordinates_to_array_index(np.array([pos.x, pos.y])) - offsets
            half = arr // self.__square_size_px           # 반타일 인덱스
            tr, tc = int(half[0]) * 2, int(half[1]) * 2   # 텍스트 격자 셀(타일 경계 = 벽 자리)
            best, best_d = None, 1e9
            for dr in range(-3, 4):
                for dc in range(-3, 4):
                    r, c = tr + dr, tc + dc
                    if 0 <= r < rows and 0 <= c < cols and text_grid[r][c] == "1":
                        d = dr * dr + dc * dc
                        if d < best_d:
                            best_d, best = d, (r, c)
            if best is not None:
                text_grid[best[0]][best[1]] = letter
                placed += 1
            elif 0 <= tr < rows and 0 <= tc < cols:
                text_grid[tr][tc] = letter
                placed += 1
        if placed:
            print(f"[최종맵:final_matrix_creator] 토큰 {placed}개 배치")

    def __fill_area4(self, text_grid: list, pixel_grid: CompoundExpandablePixelGrid,
                     offsets: np.ndarray, area4_positions) -> None:
        """Area 4 체류 위치들이 속한 타일의 5×5 셀 블록을 '*'로 채웁니다 (경계 포함)."""
        filled_tiles = set()
        rows, cols = len(text_grid), len(text_grid[0]) if text_grid else 0
        for pos in area4_positions:
            arr = pixel_grid.coordinates_to_array_index(np.array([pos.x, pos.y])) - offsets
            quarter = arr // self.__square_size_px        # 쿼터타일 인덱스
            tile = (int(quarter[0]) // 2, int(quarter[1]) // 2)
            if tile in filled_tiles:
                continue
            filled_tiles.add(tile)
            r0, c0 = tile[0] * 4, tile[1] * 4
            for r in range(r0, r0 + 5):
                for c in range(c0, c0 + 5):
                    if 0 <= r < rows and 0 <= c < cols:
                        text_grid[r][c] = "*"
        if filled_tiles:
            print(f"[최종맵:final_matrix_creator] Area4 타일 {len(filled_tiles)}개 '*' 채움")

    def __get_final_text_grid(self, wall_node_array: np.ndarray, floor_type_array: np.ndarray, robot_node: np.ndarray, obstacle_tile_array: list) -> list:
        """
        벽 노드 배열, 바닥 타일 코드, 로봇 시작 위치, 장애물 타일을 합성하여
        최종 텍스트 그리드를 생성합니다.

        벽은 "1", 통로는 "0", 장애물 타일은 "x", 바닥 타일 코드는 타일 중심 4개 노드에,
        로봇 시작 위치는 "5"로 설정됩니다. 우선순위: 5 > 2/3/4/6/7/8 > x > 1/0
        """
        if SHOW_MAP_AT_END:
            cv.imshow("final_grid", cv.resize(wall_node_array.astype(np.uint8) * 255, (0, 0), fx=10, fy=10, interpolation=cv.INTER_AREA))

        if DO_SAVE_FINAL_MAP:
            cv.imwrite(f"{SAVE_FINAL_MAP_DIR}/WALL_GRID{str(time.time()).rjust(50)}.png", wall_node_array.astype(np.uint8) * 255)

        final_text_grid = []

        # 벽 노드 배열을 "1"(벽) 또는 "0"(통로)으로 변환
        for row in wall_node_array:
            f_row = []
            for val in row:
                if val:
                    f_row.append("1")
                else:
                    f_row.append("0")
            final_text_grid.append(f_row)

        # 장애물 타일을 "x"로 표시 (빈 공간 "0"인 위치만 덮어씀 → 구멍/늪/체크포인트 보존)
        for y, row in enumerate(obstacle_tile_array):
            for x, is_obstacle in enumerate(row):
                if is_obstacle:
                    x1 = x * 4 + 3
                    y1 = y * 4 + 3
                    for diagonal in np.array(((1, 1), (-1, 1), (-1, -1), (1, -1))):
                        ny, nx = y1 + diagonal[0], x1 + diagonal[1]
                        try:
                            if final_text_grid[ny][nx] == "0":
                                final_text_grid[ny][nx] = "x"
                        except IndexError:
                            pass

        # 바닥 타일 코드를 각 타일의 중심 4개 노드(x1+3, y1+3)에 설정
        for y, row in enumerate(floor_type_array):
            for x, val in enumerate(row):
                x1 = x * 4 + 3  # 각 타일의 중심 노드 위치 (4노드씩, 3 오프셋)
                y1 = y * 4 + 3
                self.__set_node_as_character(final_text_grid, np.array([y1, x1]), val)

        # 로봇 시작 위치를 "5"로 설정
        self.__set_node_as_character(final_text_grid, robot_node, "5")

        return final_text_grid


    def __get_offsets(self, square_size: float, raw_offsets: np.array) -> np.ndarray:
        """픽셀 그리드의 오프셋을 타일 크기로 모듈러 계산하여 정수 오프셋을 반환합니다."""
        return np.round(raw_offsets % square_size).astype(int)


    def __set_node_as_character(self, final_text_grid: list, node: np.ndarray, character: str) -> list:
        """
        지정된 노드와 대각선 방향 4개 이웃 노드에 character를 설정합니다.
        타일 중심 근처 2×2 영역에 동일한 타일 코드를 기록합니다.
        IndexError는 배열 경계 밖 노드에 대해 무시합니다.
        """
        for diagonal in np.array(((1, 1), (-1, 1), (-1, -1), (1, -1))):
            n = node + diagonal
            try:
                final_text_grid[n[0]][n[1]] = character
            except IndexError:
                pass

        return final_text_grid
