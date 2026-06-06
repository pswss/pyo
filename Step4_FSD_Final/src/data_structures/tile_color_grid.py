import numpy as np
import cv2 as cv
import copy


class TileColorExpandableGrid:
    """
    타일(격자칸) 단위로 색상 정보를 저장하는 동적 확장 가능 그리드입니다.

    CompoundExpandablePixelGrid보다 해상도가 낮으며,
    타일 한 칸이 tile_size(m) 크기를 나타냅니다.
    현재는 주로 참조용으로 유지되며 실제 색상 저장은 픽셀 그리드가 담당합니다.
    """
    def __init__(self, initial_shape, tile_size):
        self.array_shape = np.array(initial_shape, dtype=int)
        self.offsets = self.array_shape // 2

        self.grid_index_max = self.array_shape - self.offsets
        self.grid_index_min = self.offsets * -1

        self.array = np.zeros(self.array_shape, np.bool_)
        self.resolution = 1 / tile_size  # 그리드 해상도 (타일/미터)

    # -------- 좌표 변환 --------

    def coordinates_to_grid_index(self, coordinates: np.ndarray) -> np.ndarray:
        """실제 좌표(m) → 그리드 인덱스"""
        coords = (coordinates * self.resolution).astype(int)
        return np.flip(coords)

    def grid_index_to_coordinates(self, grid_index: np.ndarray) -> np.ndarray:
        """그리드 인덱스 → 실제 좌표(m)"""
        index = (grid_index.astype(float) / self.resolution)
        return np.flip(index)

    def array_index_to_grid_index(self, array_index: np.ndarray) -> np.ndarray:
        return array_index - self.offsets

    def grid_index_to_array_index(self, grid_index: np.ndarray) -> np.ndarray:
        return grid_index + self.offsets

    def array_index_to_coordinates(self, array_index) -> np.ndarray:
        grid_index = self.array_index_to_grid_index(array_index)
        return self.grid_index_to_coordinates(grid_index)

    def coordinates_to_array_index(self, coordinates) -> np.ndarray:
        grid_index = self.coordinates_to_grid_index(coordinates)
        return self.grid_index_to_array_index(grid_index)

    # -------- 그리드 동적 확장 --------

    def expand_to_grid_index(self, grid_index: np.ndarray):
        """지정 인덱스를 포함하도록 그리드를 확장합니다."""
        array_index = self.grid_index_to_array_index(grid_index)
        if array_index[0] + 1 > self.array_shape[0]:
            self.add_end_row(array_index[0] - self.array_shape[0] + 1)
        if array_index[1] + 1 > self.array_shape[1]:
            self.add_end_column(array_index[1] - self.array_shape[1] + 1)
        if array_index[0] < 0:
            self.add_begining_row(array_index[0] * -1)
        if array_index[1] < 0:
            self.add_begining_column(array_index[1] * -1)

    def add_end_row(self, size):
        self.array_shape = np.array([self.array_shape[0] + size, self.array_shape[1]])
        self.array = self.__add_end_row_to_array(self.array, size)

    def add_begining_row(self, size):
        self.offsets[0] += size
        self.array_shape = np.array([self.array_shape[0] + size, self.array_shape[1]])
        self.array = self.__add_begining_row_to_array(self.array, size)

    def add_end_column(self, size):
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])
        self.array = self.__add_end_column_to_array(self.array, size)

    def add_begining_column(self, size):
        self.offsets[1] += size
        self.array_shape = np.array([self.array_shape[0], self.array_shape[1] + size])
        self.array = self.__add_begining_column_to_array(self.array, size)

    def __add_end_row_to_array(self, array, size):
        return np.vstack((array, np.zeros((size, self.array_shape[1]), dtype=array.dtype)))

    def __add_begining_row_to_array(self, array, size):
        return np.vstack((np.zeros((size, self.array_shape[1]), dtype=array.dtype), array))

    def __add_end_column_to_array(self, array, size):
        return np.hstack((array, np.zeros((self.array_shape[0], size), dtype=array.dtype)))

    def __add_begining_column_to_array(self, array, size):
        return np.hstack((np.zeros((self.array_shape[0], size), dtype=array.dtype), array))

    def get_colored_grid(self):
        pass  # 디버그용 컬러 이미지 (미구현)
