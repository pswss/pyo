import math
import numpy as np
import cv2 as cv

from fixture_detection.victim_clasification import VictimClassifier
from fixture_detection.color_filter import ColorFilter, get_wall_mask
from fixture_detection.non_fixture_filterer import NonFixtureFilter

from flags import SHOW_FIXTURE_DEBUG


class FixtureType:
    """색상별 픽셀 수 범위(ranges)로 fixture 여부를 판별하는 데이터 클래스"""
    def __init__(self, fixture_type, default_letter, ranges=None):
        self.fixture_type = fixture_type
        self.default_letter = default_letter
        self.ranges = ranges

    def is_fixture(self, colour_counts: dict):
        for color in self.ranges:
            if not self.ranges[color][0] <= colour_counts[color] <= self.ranges[color][1]:
                return False
        return True


class FixtureClasiffier:
    """
    v26 fixture 분류기 (RCJ 2026 규정 §3.7 기준).

    2026 토큰 체계 (옛 UN 플래카드 hazmat 폐지):
    - Letter victim: 벽에 인쇄된 검정 대문자 심볼(Φ→H, Ψ→S, Ω→U). 흑백만.
    - Cognitive target: 지름 5cm 동심원 5개(원+4링). 색값 합산
      (검정 -2, 빨강 -1, 노랑 0, 초록 +1, 파랑 +2) → 0:F 1:P 2:C 3:O,
      그 외 합 → fake victim(보고 안 함).
    - Fake letter victim: 글자가 3D로 돌출 → depth 센서로만 구분(여기선 미처리).

    분류 순서:
    1. already_detected → None
    2. 채도색(R/Y/G/B) 존재 → cognitive target (링 합산 → F/P/C/O)
       (양수합 P/C/O는 항상 초록/파랑 포함, F=0은 노랑 포함 → 채도색 유무로 정확히 라우팅)
    3. 흑백만 → letter victim (Φ/Ψ/Ω → H/S/U)
    4. 매칭 없음 → None
    """
    def __init__(self):
        self.victim_classifier = VictimClassifier()

        self.color_filters = {
            "black":    ColorFilter(lower_hsv=(0, 0, 0),     upper_hsv=(0, 0, 160)),
            "white":    ColorFilter(lower_hsv=(0, 0, 170),   upper_hsv=(255, 110, 208)),
            "yellow":   ColorFilter(lower_hsv=(25, 170, 82), upper_hsv=(30, 255, 255)),
        }
        self.extra_color_filters = {
            "red_low":  ColorFilter(lower_hsv=(0, 80, 80),   upper_hsv=(10, 255, 255)),
            "red_high": ColorFilter(lower_hsv=(160, 80, 80), upper_hsv=(179, 255, 255)),
            "green":    ColorFilter(lower_hsv=(40, 80, 80),  upper_hsv=(85, 255, 255)),
            "blue":     ColorFilter(lower_hsv=(100, 80, 80), upper_hsv=(130, 255, 255)),
        }
        self.min_fixture_height = 16
        self.min_fixture_width_factor = 0.8

        # already_detected 판별 규칙
        self.already_detected_types = (
            FixtureType("already_detected", "", {"white": (5000, math.inf),
                                                  "black": (0, 0),
                                                  "red":   (0, 0),
                                                  "yellow":(0, 0)}),
            FixtureType("already_detected", "", {"white": (0, 2000),
                                                  "black": (0, 0),
                                                  "red":   (0, 0),
                                                  "yellow":(0, 0)}),
        )

        self.non_fixture_filter = NonFixtureFilter()

    def sum_images(self, images):
        final_img = np.zeros_like(images[0])
        for image in images:
            final_img += image
        final_img[final_img > 255] = 255
        return final_img

    def filter_fixtures(self, victims) -> list:
        final_victims = []
        for vic in victims:
            if vic["image"].shape[0] > self.min_fixture_height and vic["image"].shape[1] > self.min_fixture_height * self.min_fixture_width_factor:
                final_victims.append(vic)
        return final_victims

    def find_fixtures(self, image) -> list:
        image = np.rot90(image, k=3)
        if SHOW_FIXTURE_DEBUG:
            cv.imshow("image", image)

        binary_images = []
        # 탐지 윤곽: victim(흑백) + cognitive target 링 색(노랑/빨강/초록/파랑)을 모두 포함.
        # 흑백+노랑만 쓰면 초록/파랑/빨강 위주의 cognitive target은 윤곽이 안 잡혀 탐지조차
        # 안 됐다(토큰 대량 누락의 진짜 원인). 분류는 classify_fixture가 따로 한다.
        for f in list(self.color_filters.values()) + list(self.extra_color_filters.values()):
            binary_images.append(f.filter(image))
        binary_image = self.sum_images(binary_images)

        walls_mask = get_wall_mask(image)
        non_fixture_by_color = self.non_fixture_filter.filter(image)
        binary_image *= (walls_mask + (non_fixture_by_color == 0))

        if SHOW_FIXTURE_DEBUG:
            cv.imshow("binaryImage", binary_image)

        contours, _ = cv.findContours(binary_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        final_victims = []

        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            final_victims.append({"image":image[y:y + h, x:x + w], "position":(x, y)})

        return self.filter_fixtures(final_victims)

    def count_colors(self, image) -> dict:
        color_point_counts = {}
        for name, f in self.color_filters.items():
            color_point_counts[name] = np.count_nonzero(f.filter(image))
        for name, f in self.extra_color_filters.items():
            color_point_counts[name] = np.count_nonzero(f.filter(image))
        color_point_counts["red"] = color_point_counts.pop("red_low", 0) + color_point_counts.pop("red_high", 0)
        return color_point_counts

    def classify_fixture(self, fixture) -> str:
        """
        fixture를 분류하여 보고 글자를 반환합니다 (RCJ 2026 §3.7).

        분류 순서:
        1. already_detected → None
        2. 채도색(R/Y/G/B) 존재 → cognitive target (동심원 5개 합산 0~3 → F/P/C/O, 그 외 → None=fake)
        3. 흑백만 → letter victim (Φ→H, Ψ→S, Ω→U)
        4. 매칭 없음 → None (보고 안 함)
        """
        image = cv.resize(fixture["image"], (100, 100), interpolation=cv.INTER_AREA)
        color_counts = self.count_colors(image)

        if SHOW_FIXTURE_DEBUG:
            print(f"[Fixture] 색상: black={color_counts['black']}, white={color_counts['white']}, "
                  f"yellow={color_counts['yellow']}, red={color_counts['red']}, "
                  f"green={color_counts.get('green',0)}, blue={color_counts.get('blue',0)}")

        # 1) already_detected 체크
        for ad in self.already_detected_types:
            if ad.is_fixture(color_counts):
                if SHOW_FIXTURE_DEBUG:
                    print("[Fixture] already_detected → None")
                return None

        # 2) 채도색(R/Y/G/B) 존재 → cognitive target (동심원 합산 → F/P/C/O)
        #    2026 hazmat은 동심원만(옛 UN 플래카드 폐지). 유효 target은 항상 채도색 포함
        #    (P/C/O=양수합→초록/파랑, F=0→노랑)이라 채도색 유무로 victim과 분리.
        saturated = (color_counts["red"] + color_counts["yellow"]
                     + color_counts.get("green", 0) + color_counts.get("blue", 0))
        if saturated > 300:
            return self._classify_cognitive_target(fixture)

        # 3) 흑백만 → letter victim (Φ/Ψ/Ω → H/S/U)
        if color_counts["black"] > 300 and color_counts["white"] > 300:
            letter = self.victim_classifier.classify_victim(fixture)
            if SHOW_FIXTURE_DEBUG:
                print(f"[Fixture] victim 분류 결과: '{letter}'")
            return letter

        # 4) 매칭 없음
        if SHOW_FIXTURE_DEBUG:
            print("[Fixture] 매칭 없음 → None")
        return None

    def _classify_cognitive_target(self, fixture) -> str:
        """5-ring 동심원 CognitiveTarget 분류.
        바깥→안쪽 순서로 5개 링 색상(K/R/Y/G/B) 식별 후 합산.
        합계 0~3 → F/P/C/O, 그 외 → None(fake)."""
        image = cv.resize(fixture["image"], (100, 100), interpolation=cv.INTER_AREA)
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        cx, cy = 50, 50
        max_r = 45
        ring_ratios = [0.92, 0.72, 0.52, 0.32, 0.12]
        color_values = {"K": -2, "R": -1, "Y": 0, "G": 1, "B": 2}
        target_types = ['F', 'P', 'C', 'O']

        score_sum = 0
        ring_colors = []
        for ratio in ring_ratios:
            r = int(max_r * ratio)
            samples = []
            for deg in range(0, 360, 45):
                rad = np.radians(deg)
                x = np.clip(int(cx + r * np.cos(rad)), 0, 99)
                y = np.clip(int(cy + r * np.sin(rad)), 0, 99)
                samples.append(hsv[y, x])
            avg = np.median(samples, axis=0)
            color = self._hsv_to_color_code(avg)
            ring_colors.append(color)
            score_sum += color_values[color]

        if SHOW_FIXTURE_DEBUG:
            print(f"[CognitiveTarget] 링: {ring_colors}, 합계: {score_sum}")

        if score_sum < 0 or score_sum > 3:
            if SHOW_FIXTURE_DEBUG:
                print(f"[CognitiveTarget] 합계 {score_sum} → 범위 밖 → None (fake)")
            return None

        letter = target_types[score_sum]
        if SHOW_FIXTURE_DEBUG:
            print(f"[CognitiveTarget] 합계 {score_sum} → '{letter}'")
        return letter

    def _hsv_to_color_code(self, hsv_val) -> str:
        """HSV 값 → K/R/Y/G/B 색상 코드 변환"""
        h, s, v = float(hsv_val[0]), float(hsv_val[1]), float(hsv_val[2])
        if v < 60:
            return "K"
        if s < 50:
            return "K"
        if h <= 10 or h >= 170:
            return "R"
        if 20 <= h <= 35:
            return "Y"
        if 40 <= h <= 85:
            return "G"
        if 100 <= h <= 130:
            return "B"
        return "K"
