import math
from pathlib import Path
import numpy as np
import cv2 as cv

from fixture_detection.victim_clasification import VictimClassifier
from fixture_detection.color_filter import ColorFilter
from fixture_detection.non_fixture_filterer import NonFixtureFilter

from flags import SHOW_FIXTURE_DEBUG


class HazardPlacardClassifier:
    """Template classifier for 2026 placard hazards: F/P/C/O."""
    def __init__(self):
        texture_dir = Path("/Users/pysw/Downloads/erebus-26.0.1/game/protos/textures")
        paths = {
            "F": texture_dir / "placard-2-flammable-gas.png",
            "O": texture_dir / "placard-5.2-organic-peroxide.png",
            "P": texture_dir / "placard-6-poison.png",
            "C": texture_dir / "placard-8-corrosive.png",
        }
        self.templates = {}
        for letter, path in paths.items():
            image = cv.imread(str(path), cv.IMREAD_UNCHANGED)
            if image is not None:
                self.templates[letter] = self.extract_features(image)

    def classify(self, image: np.ndarray) -> str | None:
        if not self.templates:
            return None

        features = self.extract_features(image)
        best_letter = None
        best_score = -1.0
        for letter, template_features in self.templates.items():
            score = self.compare_features(features, template_features)
            if score > best_score:
                best_score = score
                best_letter = letter

        if SHOW_FIXTURE_DEBUG:
            print(f"[Fixture:hazard_placard] best='{best_letter}', score={best_score:.3f}")

        return best_letter if best_score >= 0.28 else None

    def normalize_image(self, image: np.ndarray) -> np.ndarray:
        if image.shape[2] == 4:
            alpha = image[:, :, 3] > 100
            rows, cols = np.where(alpha)
            if len(rows) and len(cols):
                image = image[np.min(rows):np.max(rows) + 1, np.min(cols):np.max(cols) + 1, :3]
            else:
                image = image[:, :, :3]
        else:
            image = image[:, :, :3]

        h, w = image.shape[:2]
        scale = 96 / max(h, w)
        resized = cv.resize(image, (max(1, round(w * scale)), max(1, round(h * scale))), interpolation=cv.INTER_AREA)
        canvas = np.full((100, 100, 3), 255, dtype=np.uint8)
        y = (100 - resized.shape[0]) // 2
        x = (100 - resized.shape[1]) // 2
        canvas[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
        return canvas

    def extract_features(self, image: np.ndarray) -> dict:
        normalized = self.normalize_image(image)
        hsv = cv.cvtColor(normalized, cv.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]

        red = (((hue <= 10) | (hue >= 160)) & (sat > 55) & (val > 45))
        orange = ((hue > 10) & (hue <= 35) & (sat > 55) & (val > 45))
        yellow = ((hue > 35) & (hue <= 45) & (sat > 55) & (val > 70))
        green = ((hue >= 45) & (hue <= 90) & (sat > 55) & (val > 45))
        blue = ((hue >= 90) & (hue <= 135) & (sat > 55) & (val > 45))
        dark = val < 115
        gray = (sat < 45) & (val >= 115) & (val < 235)
        light = (sat < 45) & (val >= 180)

        return {
            "red": red,
            "orange": orange,
            "yellow": yellow,
            "green": green,
            "blue": blue,
            "dark": dark,
            "gray": gray,
            "light": light,
        }

    def compare_features(self, features: dict, template_features: dict) -> float:
        weights = {
            "red": 1.4,
            "orange": 1.4,
            "yellow": 1.0,
            "green": 1.0,
            "blue": 1.0,
            "dark": 1.8,
            "gray": 0.8,
            "light": 0.5,
        }
        total = 0.0
        weight_sum = 0.0
        for key, weight in weights.items():
            a = features[key]
            b = template_features[key]
            union = np.count_nonzero(a | b)
            if union < 20:
                continue
            score = np.count_nonzero(a & b) / union
            total += score * weight
            weight_sum += weight
        return total / weight_sum if weight_sum else 0.0


class FixtureClasiffier:
    """
    카메라 이미지에서 2026 wall token(letter victim/cognitive target)을 찾고 분류하는 클래스입니다.

    동작 흐름:
    1. find_fixtures(): 이미지에서 색상 기반으로 fixture 후보 윤곽선 추출
       - 검정/흰색/빨강/노랑/초록/파랑 이진 이미지 합산
       - 벽 마스크와 배경 색상 마스크로 비fixture 영역 제거
       - 최소 크기 필터링
    2. classify_fixture(): cognitive target은 색상 합산, letter victim은 Φ/Ψ/Ω 형태로 분류
    """
    def __init__(self):
        # 2026 letter victim(Φ/Ψ/Ω) 분류기. 서버 보고 코드는 기존 H/S/U를 그대로 쓴다.
        self.victim_classifier = VictimClassifier()
        self.hazard_classifier = HazardPlacardClassifier()

        # fixture 색상 필터: 2026 cognitive target은 K/R/Y/G/B 5색 동심원이다.
        self.colors = ("black", "white", "yellow", "red", "green", "blue")
        self.color_filters = {
            "black":  ColorFilter(lower_hsv=(0, 0, 0),      upper_hsv=(179, 80, 120)),
            "white":  ColorFilter(lower_hsv=(0, 0, 170),    upper_hsv=(179, 80, 255)),
            "yellow": ColorFilter(lower_hsv=(18, 80, 80),   upper_hsv=(40, 255, 255)),
            "red":    ColorFilter(lower_hsv=(160, 80, 80),  upper_hsv=(179, 255, 255)),
            "green":  ColorFilter(lower_hsv=(45, 80, 80),   upper_hsv=(85, 255, 255)),
            "blue":   ColorFilter(lower_hsv=(95, 80, 80),   upper_hsv=(130, 255, 255)),
        }
        self.red_low_filter = ColorFilter(lower_hsv=(0, 80, 80), upper_hsv=(10, 255, 255))
        # 벽 색상 필터 (fixture와 겹치지 않도록 벽 영역 마스킹)
        self.wall_color_filter = ColorFilter((90, 44,  0), (95, 213, 158))

        # fixture 최소 크기 필터링 기준
        self.min_fixture_height = 16
        self.min_fixture_width_factor = 0.8  # 최소 너비 = min_fixture_height * 0.8
        self.min_fixture_area = 120

        # cognitive target 합산값 0..3은 Erebus 26 supervisor에서 F/P/C/O로 판정된다.
        self.target_letters_by_sum = {0: "F", 1: "P", 2: "C", 3: "O"}
        self.target_scores_by_color = {
            "black": -2,
            "red": -1,
            "yellow": 0,
            "green": 1,
            "blue": 2,
        }

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
            h, w = vic["image"].shape[:2]
            if h > self.min_fixture_height and w > self.min_fixture_height * self.min_fixture_width_factor:
                final_victims.append(vic)

        return final_victims

    def find_fixtures(self, image) -> list:
        """
        이미지에서 fixture 후보를 찾아 위치와 이미지 슬라이스 목록을 반환합니다.

        동작:
        1. 2026 token 색상 필터의 이진 이미지를 합산
        2. 벽 내부 영역만 유지 (벽 마스크 적용)
        3. 배경 색상(non-fixture) 영역 제거
        4. 윤곽선 감지 후 경계 박스 추출
        5. 최소 크기 필터링
        """
        # 이미지를 90도 회전 (Webots 카메라 방향 보정)
        image = np.rot90(image, k=3)
        if SHOW_FIXTURE_DEBUG:
            cv.imshow("image", image)

        # fixture 색상의 이진 이미지를 합산하여 후보 영역 생성
        binary_images = []
        for name, f in self.color_filters.items():
            color_image = f.filter(image)
            if name == "red":
                color_image += self.red_low_filter.filter(image)
                color_image[color_image > 255] = 255
            binary_images.append(color_image)

        binary_image = self.sum_images(binary_images)

        # 벽 마스크: 벽 내부 영역만 fixture로 인정
        walls_mask = self.get_wall_mask(image)

        # 배경 색상 마스크: 배경 픽셀 제거
        non_fixture_by_color = self.non_fixture_filter.filter(image)

        # 벽 내부이고 배경 색상이 아닌 픽셀만 유지
        valid_area = np.logical_and(walls_mask, non_fixture_by_color == 0)
        binary_image = (binary_image > 0).astype(np.uint8) * 255
        binary_image[~valid_area] = 0
        binary_image = cv.morphologyEx(binary_image, cv.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        if SHOW_FIXTURE_DEBUG:
            cv.imshow("binaryImage", binary_image)

        # 윤곽선 감지 후 경계 박스 추출
        final_victims = []
        contours, _ = cv.findContours(binary_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv.contourArea, reverse=True)

        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            area = cv.contourArea(c)
            if area < self.min_fixture_area:
                continue
            aspect = w / max(h, 1)
            if aspect < 0.35 or aspect > 3.0:
                continue
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
        이미지에서 각 색상(검정/흰색/빨강/노랑/초록/파랑)의 픽셀 수를 계산하여
        색상명 → 픽셀 수 딕셔너리로 반환합니다.
        """
        color_point_counts = {}

        for name, filter in self.color_filters.items():
            color_image = filter.filter(image)
            if name == "red":
                color_image += self.red_low_filter.filter(image)
                color_image[color_image > 255] = 255

            color_point_counts[name] = np.count_nonzero(color_image)

        return color_point_counts

    def classify_cognitive_target(self, image: np.ndarray, color_counts: dict) -> str | None:
        """
        2026 cognitive target을 5개 동심원 색으로 읽어서 F/P/C/O 중 하나를 반환한다.
        유효 합산값(0..3)이 아니거나 target으로 보기 어려우면 None을 반환한다.
        """
        colored_count = sum(color_counts.get(name, 0) for name in ("red", "yellow", "green", "blue"))
        if colored_count < 250:
            return None

        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]
        target_mask = ((sat > 70) & (val > 70)) | (val < 90)
        if image.shape[2] == 4:
            target_mask &= image[:, :, 3] > 100
        target_mask = target_mask.astype(np.uint8) * 255

        contours, _ = cv.findContours(target_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return None

        contour = max(contours, key=cv.contourArea)
        area = cv.contourArea(contour)
        if area < 800:
            return None

        (center_x, center_y), radius = cv.minEnclosingCircle(contour)
        if radius < 15:
            return None

        sample_radii = [radius * ratio for ratio in (0.10, 0.30, 0.50, 0.70, 0.90)]
        ring_colors = []
        yy, xx = np.indices(image.shape[:2])
        distance_from_center = np.sqrt((xx - center_x) ** 2 + (yy - center_y) ** 2)

        for sample_radius in sample_radii:
            band_width = max(radius * 0.055, 1.5)
            ring_mask = np.abs(distance_from_center - sample_radius) <= band_width
            if image.shape[2] == 4:
                ring_mask &= image[:, :, 3] > 100
            samples = image[:, :, :3][ring_mask]
            if len(samples) < 10:
                return None
            ring_colors.append(self.get_target_color_from_pixels(np.array(samples)))

        if any(color is None for color in ring_colors):
            return None

        target_sum = sum(self.target_scores_by_color[color] for color in ring_colors)
        letter = self.target_letters_by_sum.get(target_sum)

        if SHOW_FIXTURE_DEBUG:
            print(f"[Fixture:cognitive_target] rings={ring_colors}, sum={target_sum}, report='{letter}'")

        return letter

    def get_target_color_from_pixels(self, pixels: np.ndarray) -> str | None:
        bgr = np.median(pixels, axis=0)
        blue, green, red = bgr

        if max(bgr) < 80:
            return "black"
        if red > 140 and green > 120 and blue < 120:
            return "yellow"
        if red > green + 35 and red > blue + 35:
            return "red"
        if green > red + 35 and green > blue + 35:
            return "green"
        if blue > red + 35 and blue > green + 35:
            return "blue"
        return None

    def classify_fixture(self, fixture) -> str:
        """
        fixture 이미지를 분석하여 보고할 글자를 반환합니다.

        1. 100×100으로 리사이즈
        2. cognitive target이면 5개 링 색상 합산으로 F/P/C/O 반환
        3. 아니면 letter victim(Φ/Ψ/Ω)을 H/S/U 보고 코드로 반환
        4. 유효하지 않은 target은 None 반환
        """
        image = cv.resize(fixture["image"], (100, 100), interpolation=cv.INTER_AREA)

        color_point_counts = self.count_colors(image)

        if SHOW_FIXTURE_DEBUG:
            print(f"[Fixture:fixture_clasification.classify_fixture] 색상 픽셀 수: {color_point_counts}")

        colored_count = sum(color_point_counts.get(name, 0) for name in ("red", "yellow", "green", "blue"))
        if colored_count >= 250:
            # 색 링이 보이면 cognitive target으로 간주합니다. 합산값이 무효면 보고하지 않습니다.
            letter = self.classify_cognitive_target(image, color_point_counts)
            if letter is None:
                letter = self.hazard_classifier.classify(image)
        else:
            letter = self.hazard_classifier.classify(image)
            if letter is None:
                letter = self.victim_classifier.classify_victim({"image": image})
 
        if SHOW_FIXTURE_DEBUG:
            print(f"[Fixture:fixture_clasification.classify_fixture] 최종 결과: 글자='{letter}'")

        return letter
