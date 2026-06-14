import numpy as np
import cv2 as cv

from data_structures.vectors import Position2D

# 통로 문자 → 연결하는 두 구역 (규정 §5.6.10.b.ii)
PASSAGE_ENDS = {
    "b": (1, 2), "y": (1, 3), "g": (1, 4),
    "p": (2, 3), "o": (2, 4), "r": (3, 4),
}

# 통로 색 HSV 범위 — final_matrix_creator.FloorMatrixCreator와 동기 유지할 것.
# 파/보/빨 = 시뮬 실측, 노/초/주 = 표준 hue ★시뮬 튜닝
PASSAGE_HSV_RANGES = {
    "b": ((120, 182, 49), (120, 204, 232)),
    "p": ((132, 156, 36), (133, 192, 185)),
    "r": ((0, 150, 49), (5, 255, 232)),
    "y": ((25, 150, 80), (35, 255, 255)),
    "g": ((50, 150, 50), (70, 255, 255)),
    "o": ((10, 150, 80), (20, 255, 255)),
}


def classify_passage_patch(bgr_patch) -> "str | None":
    """로봇 발밑 바닥색 패치(BGR)가 통로 색이면 해당 문자, 아니면 None.
    패치의 30% 이상이 한 통로 색이어야 인정(타일 경계 걸침/노이즈 면역)."""
    if bgr_patch.size == 0 or not bgr_patch.any():
        return None
    hsv = cv.cvtColor(bgr_patch, cv.COLOR_BGR2HSV)
    area = hsv.shape[0] * hsv.shape[1]
    best, best_count = None, 0
    for letter, (lo, hi) in PASSAGE_HSV_RANGES.items():
        count = int(np.count_nonzero(cv.inRange(hsv, lo, hi)))
        if count > 0.3 * area and count > best_count:
            best, best_count = letter, count
    return best

# 같은 쪽으로 되돌아 나간 것과 실제 횡단을 구분하는 최소 변위(m). 통로=풀타일 12cm.
MIN_CROSS_DISPLACEMENT = 0.06


class AreaTracker:
    """로봇이 현재 어느 구역(Area 1~4)에 있는지 통로 횡단으로 추적합니다.

    필요한 이유(맵 일치율): 정답 행렬은 Area 4 타일을 셀 단위 '*'로 채운다
    (MapAnswer.py: room==4 → 5×5 전부 '*'). 제출 행렬에 '*'가 없으면 그 셀
    전부 불일치 — world2 기준 ~200셀로 일치율을 ~50%로 깎는 최대 감점원.

    동작: 통로 타일(색 b/y/g/p/o/r)에 '진입'할 때 위치를 기억하고, 통로를
    '벗어날 때' 진입점에서 충분히 멀면(횡단) 구역을 통로의 반대쪽 끝으로 전환.
    같은 쪽으로 되돌아 나가면(변위 작음) 구역 유지. Area 4에 있는 동안의
    로봇 위치(절대좌표)를 기록해 최종 행렬에서 해당 타일을 '*'로 채운다.
    """

    def __init__(self) -> None:
        self.current_area = 1                 # 시작 타일은 항상 Area 1
        self.area4_positions: list[Position2D] = []   # Area 4 체류 중 로봇 위치 기록
        self.__on_passage = None              # 현재 밟고 있는 통로 문자 (없으면 None)
        self.__entry_position = None          # 통로 진입 시 위치
        self.__entered_from = 1               # 통로 진입 직전 구역

    def step(self, passage_letter, robot_position: Position2D) -> None:
        """매 스텝 호출. passage_letter = 현재 타일의 통로 문자(통로 아니면 None)."""
        if passage_letter is not None:
            if self.__on_passage is None:
                self.__on_passage = passage_letter
                self.__entry_position = Position2D(robot_position.x, robot_position.y)
                self.__entered_from = self.current_area
            return

        if self.__on_passage is not None:
            crossed = (self.__entry_position is not None and
                       robot_position.get_distance_to(self.__entry_position)
                       > MIN_CROSS_DISPLACEMENT)
            ends = PASSAGE_ENDS.get(self.__on_passage)
            if crossed and ends:
                if self.__entered_from == ends[0]:
                    self.current_area = ends[1]
                elif self.__entered_from == ends[1]:
                    self.current_area = ends[0]
                # 진입 구역이 통로 양끝 어디도 아니면(추적 꼬임) 구역 유지 — 오전환 방지
                print(f"[구역추적:area_tracker] 통로 '{self.__on_passage}' 횡단 → Area {self.current_area}")
            self.__on_passage = None
            self.__entry_position = None

        if self.current_area == 4:
            self.area4_positions.append(Position2D(robot_position.x, robot_position.y))
