import math
import random

import numpy as np
import cv2 as cv

from fixture_detection.victim_clasification import VictimClassifier
from fixture_detection.color_filter import ColorFilter
from fixture_detection.non_fixture_filterer import NonFixtureFilter

from flags import SHOW_DEBUG, SHOW_FIXTURE_DEBUG, TUNE_FILTER


class FixtureType:
    """
    하나의 fixture 종류(조난자/위험물/이미 감지됨)를 정의하는 데이터 클래스입니다.

    색상별 픽셀 수 범위(ranges)로 fixture 여부를 판별합니다.
    모든 색상의 범위를 동시에 만족해야 해당 fixture로 분류됩니다.
    """
    def __init__(self, fixture_type, default_letter, ranges=None):
        self.fixture_type = fixture_type     # fixture 종류 (예: "victim", "flammable", "already_detected")
        self.default_letter = default_letter # 해당 fixture의 기본 보고 글자
        self.ranges = ranges                 # 색상별 픽셀 수 범위 딕셔너리

    def is_fixture(self, colour_counts: dict):
        """
        주어진 색상별 픽셀 수가 이 fixture 타입의 모든 범위 조건을 만족하는지 확인합니다.
        ranges에 정의된 모든 색상이 해당 범위 내에 있어야 True를 반환합니다.
        """
        for color in self.ranges:
            if not self.ranges[color][0] <= colour_counts[color] <= self.ranges[color][1]:
                return False
        return True


class FixtureClasiffier:
    """
    카메라 이미지에서 fixture(조난자 및 위험물 표지)를 찾고 분류하는 클래스입니다.

    동작 흐름:
    1. find_fixtures(): 이미지에서 색상 기반으로 fixture 후보 윤곽선 추출
       - 4가지 색상(검정/흰색/노란색/빨간색) 이진 이미지 합산
       - 벽 마스크와 배경 색상 마스크로 비fixture 영역 제거
       - 최소 크기 필터링
    2. classify_fixture(): 각 후보에 대해 색상 픽셀 수를 계산하고 fixture 타입 결정
       - fixture_types 목록을 우선순위대로 순차 검사
       - victim: VictimClassifier로 H/S/U 글자 분류
       - already_detected: 보고하지 않음 (None 반환)
       - 기타: 해당 타입의 default_letter 반환
    """
    def __init__(self):
        # victim 이미지의 글자 분류기
        self.victim_classifier = VictimClassifier()

        # fixture 색상 필터 (4가지 주요 색상)
        self.colors = ("black", "white", "yellow", "red")
        self.color_filters = {
            "black":  ColorFilter(lower_hsv=(0, 0, 0),     upper_hsv=(0, 0, 160)),
            "white":  ColorFilter(lower_hsv=(0, 0, 170),   upper_hsv=(255, 110, 208)),
            "yellow": ColorFilter(lower_hsv=(25, 170, 82), upper_hsv=(30, 255, 255)),
            "red":    ColorFilter(lower_hsv=(134, 91, 155),upper_hsv=(175, 255, 204))
        }
        # 벽 색상 필터 (fixture와 겹치지 않도록 벽 영역 마스킹)
        self.wall_color_filter = ColorFilter((90, 44,  0), (95, 213, 158))

        # fixture 최소 크기 필터링 기준
        self.min_fixture_height = 16
        self.min_fixture_width_factor = 0.8  # 최소 너비 = min_fixture_height * 0.8

        # 가능한 fixture 글자 목록
        self.possible_fixture_letters = ["P", "O", "F", "C", "S", "H", "U"]

        # fixture 타입 목록 (우선순위 순서로 검사됨)
        self.fixture_types = (
            # 이미 보고된 위치 (흰색 픽셀 5000 이상 = 전체 흰색 = 감지된 위치 표시)
            FixtureType("already_detected", "",  {"white": (5000,    math.inf),
                                                  "black": (0,    0),
                                                  "red":   (0,    0),
                                                  "yellow":(0,    0),}),

            # 이미 보고된 위치 (흰색 거의 없음 = 배경 또는 공백)
            FixtureType("already_detected", "",  {"white": (0,    2000),
                                                  "black": (0,    0),
                                                  "red":   (0,    0),
                                                  "yellow":(0,    0),}),

            # 유기 과산화물: 빨간색 + 노란색 모두 있음
            FixtureType("organic_peroxide", "O", {"red":   (1,    math.inf),
                                                  "yellow":(1,    math.inf),}),

            # 가연성: 흰색 + 빨간색 모두 있음
            FixtureType("flammable", "F",        {"white": (1,    math.inf),
                                                  "red":   (1,    math.inf),}),

            # 조난자(victim): 흰색 많음 + 검정 중간
            FixtureType("victim",    "H",        {"white": (4500, math.inf),
                                                  "black": (1000,  4000),}),

            # 부식성: 흰색 중간 + 검정 중간
            FixtureType("corrosive", "C",        {"white": (700,  4500),
                                                  "black": (900, 3000),}),

            # 독성: 흰색 적음~중간 + 검정 적음
            FixtureType("poison",    "P",        {"white": (2000,  5000),
                                                  "black": (100,    1000),}),
        )

        self.non_fixture_filter = NonFixtureFilter()


    def get_wall_mask(self, image: np.ndarray):
        """
        이미지에서 벽 영역을 채운 마스크를 생성합니다.
        벽 윤곽선을 채워서 벽 내부 영역도 마스킹합니다.
        좌우에 여백(margin)을 추가하여 벽 경계의 fixture를 놓치지 않도록 합니다.
        """
        margin = 1
        raw_wall = self.wall_color_filter.filter(image)

        # 좌우 여백 추가 (벽 가장자리가 이미지 끝에 있을 때 처리)
        wall = np.ones(shape=(raw_wall.shape[0], raw_wall.shape[1] + margin * 2), dtype=np.uint8) * 255

        wall[:, margin: -margin] = raw_wall

        conts, _ = cv.findContours(wall, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        debug = np.copy(image)

        filled_wall = np.zeros_like(wall, dtype=np.bool_)

        # 각 벽 윤곽선 내부를 채움 (벽 안쪽 공간을 fixture 영역으로 인정)
        for c in conts:
            this_cont = np.zeros_like(wall, dtype=np.uint8)
            cv.fillPoly(this_cont, [c,], 255)
            filled_wall += this_cont > 0

        filled_wall = filled_wall[:, margin:-margin]

        return filled_wall

    def sum_images(self, images):
        """여러 이진 이미지를 합산하고 255를 초과하는 값을 255로 클리핑합니다."""
        final_img = images[0]
        for index, image in enumerate(images):
            final_img += image
        final_img[final_img > 255] = 255
        return final_img

    def filter_fixtures(self, victims) -> list:
        """
        최소 크기 기준에 맞지 않는 작은 후보를 제거합니다.
        너무 작은 윤곽선은 노이즈로 간주합니다.
        """
        final_victims = []
        for vic in victims:
            if vic["image"].shape[0] > self.min_fixture_height and vic["image"].shape[1] > self.min_fixture_height * self.min_fixture_width_factor:
                final_victims.append(vic)

        return final_victims

    def find_fixtures(self, image) -> list:
        """
        이미지에서 fixture 후보를 찾아 위치와 이미지 슬라이스 목록을 반환합니다.

        동작:
        1. 4개 색상 필터의 이진 이미지를 합산
        2. 벽 내부 영역만 유지 (벽 마스크 적용)
        3. 배경 색상(non-fixture) 영역 제거
        4. 윤곽선 감지 후 경계 박스 추출
        5. 최소 크기 필터링
        """
        # 이미지를 90도 회전 (Webots 카메라 방향 보정)
        image = np.rot90(image, k=3)
        if SHOW_FIXTURE_DEBUG:
            cv.imshow("image", image)

        # 4가지 색상의 이진 이미지를 합산하여 fixture 후보 영역 생성
        binary_images = []
        for f in self.color_filters.values():
            binary_images.append(f.filter(image))

        binary_image = self.sum_images(binary_images)

        # 벽 마스크: 벽 내부 영역만 fixture로 인정
        walls_mask = self.get_wall_mask(image)

        # 배경 색상 마스크: 배경 픽셀 제거
        non_fixture_by_color = self.non_fixture_filter.filter(image)

        # 벽 내부이고 배경 색상이 아닌 픽셀만 유지
        binary_image *= (walls_mask + (non_fixture_by_color == 0))

        if SHOW_FIXTURE_DEBUG:
            cv.imshow("binaryImage", binary_image)

        # 윤곽선 감지 후 경계 박스 추출
        # (한 번 더 감지하여 글자 내부 윤곽선 제거)
        contours, _ = cv.findContours(binary_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        final_victims = []
        contours, _ = cv.findContours(binary_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            final_victims.append({"image":image[y:y + h, x:x + w], "position":(x, y)})

        # 최소 크기 미달 후보 제거
        return self.filter_fixtures(final_victims)

    def get_bounding_rect_of_contours(self, contours):
        """모든 윤곽선을 포함하는 최소 경계 박스 좌표를 반환합니다."""
        min_x = 0
        min_y = 0
        max_x = math.inf
        max_y = math.inf

        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            min_x = min(x, min_x)
            min_y = min(y, min_y)
            max_x = max(x + w, max_x)
            max_y = max(y + h, max_y)

        return min_x, min_y, max_x, max_y


    def count_colors(self, image) -> dict:
        """
        이미지에서 각 색상(검정/흰색/노란색/빨간색)의 픽셀 수를 계산하여
        색상명 → 픽셀 수 딕셔너리로 반환합니다.
        """
        color_point_counts = {}

        for name, filter in self.color_filters.items():
            color_image = filter.filter(image)

            color_point_counts[name] = np.count_nonzero(color_image)

        return color_point_counts

    def classify_fixture(self, fixture) -> str:
        """
        fixture 이미지를 분석하여 보고할 글자를 반환합니다.

        동작:
        1. 100×100으로 리사이즈
        2. 4가지 색상 픽셀 수 계산
        3. fixture_types 목록에서 조건 맞는 첫 번째 타입 선택 (우선순위 순)
        4. victim이면 VictimClassifier로 H/S/U 분류
        5. already_detected이면 None 반환 (보고 안 함)
        6. 기타 타입은 default_letter 반환
        7. 매칭 없으면 무작위 글자 반환

        반환값: 보고할 글자 문자열 또는 None (already_detected)
        """
        image = cv.resize(fixture["image"], (100, 100), interpolation=cv.INTER_AREA)

        color_point_counts = self.count_colors(image)

        if SHOW_FIXTURE_DEBUG:
            print(f"[Fixture:fixture_clasification.classify_fixture] 색상 픽셀 수: 검정={color_point_counts.get('black',0)}, 흰색={color_point_counts.get('white',0)}, 노란색={color_point_counts.get('yellow',0)}, 빨간색={color_point_counts.get('red',0)}")

        # fixture_types를 우선순위 순서대로 검사하여 첫 번째 매칭 타입 선택
        final_fixture_filter = None
        for filter in self.fixture_types:
            if filter.is_fixture(color_point_counts):
                final_fixture_filter = filter
                break

        # 매칭 없으면 무작위 글자 반환 (대회 점수 최소화 전략)
        if final_fixture_filter is None:
            letter = random.choice(self.possible_fixture_letters)

        # victim이면 글자 인식 모듈로 상세 분류
        elif final_fixture_filter.fixture_type == "victim":
            letter = self.victim_classifier.classify_victim(fixture)

        # 이미 감지된 위치면 보고하지 않음 (중복 보고 방지)
        elif final_fixture_filter.fixture_type == "already_detected":
            letter = None

        # 그 외 fixture 타입은 고정 글자 반환
        else:
            letter = final_fixture_filter.default_letter

        if SHOW_FIXTURE_DEBUG:
            print(f"[Fixture:fixture_clasification.classify_fixture] 최종 결과: 타입='{final_fixture_filter.fixture_type if final_fixture_filter else '없음'}', 글자='{letter}'")

        return letter
