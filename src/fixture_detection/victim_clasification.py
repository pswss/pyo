from pathlib import Path

import cv2 as cv
import numpy as np

from flags import SHOW_FIXTURE_DEBUG
from fixture_detection.color_filter import ColorFilter


class VictimClassifier:
    """
    2026 letter victim(Φ/Ψ/Ω)을 서버 보고 코드(H/S/U)로 분류합니다.
    Erebus supervisor는 harmed/stable/unharmed를 각각 H/S/U 1바이트 코드로 받습니다.
    """
    def __init__(self):
        self.victim_letter_filter = ColorFilter(lower_hsv=(0, 0, 0), upper_hsv=(0, 0, 130))
        self.templates = self.load_templates()

    def load_templates(self) -> dict:
        texture_dir = Path("/Users/pysw/Downloads/erebus-26.0.1/game/protos/textures")
        paths = {
            "H": texture_dir / "victim_harmed_not_found.png",
            "S": texture_dir / "victim_stable_not_found.png",
            "U": texture_dir / "victim_unharmed_not_found.png",
        }

        templates = {}
        for letter, path in paths.items():
            if not path.is_file():
                continue
            image = cv.imread(str(path), cv.IMREAD_UNCHANGED)
            if image is None:
                continue
            templates[letter] = self.normalize_letter(self.victim_letter_filter.filter(image))
        return templates

    def crop_white(self, binary_img):
        rows, cols = np.where(binary_img == 255)
        if len(rows) == 0 or len(cols) == 0:
            return binary_img

        min_y, max_y = np.min(rows), np.max(rows)
        min_x, max_x = np.min(cols), np.max(cols)
        return binary_img[min_y:max_y + 1, min_x:max_x + 1]

    def isolate_victim(self, image):
        binary = self.victim_letter_filter.filter(image)
        letter = self.get_biggest_blob(binary)

        if SHOW_FIXTURE_DEBUG:
            cv.imshow("thresh", binary)

        return letter

    def get_biggest_blob(self, binary_image: np.ndarray) -> np.ndarray | None:
        contours, _ = cv.findContours(binary_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        max_size = 0
        biggest_blob = None
        for contour in contours:
            x, y, w, h = cv.boundingRect(contour)
            size = w * h
            if size > max_size:
                biggest_blob = binary_image[y:y + h, x:x + w]
                max_size = size

        return biggest_blob

    def normalize_letter(self, binary_image: np.ndarray) -> np.ndarray:
        binary_image = self.crop_white(binary_image)
        if binary_image.size == 0:
            return np.zeros((100, 100), dtype=np.uint8)

        h, w = binary_image.shape[:2]
        scale = 84 / max(h, w)
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))
        resized = cv.resize(binary_image, (new_w, new_h), interpolation=cv.INTER_AREA)
        resized = (resized > 80).astype(np.uint8) * 255

        canvas = np.zeros((100, 100), dtype=np.uint8)
        y = (100 - new_h) // 2
        x = (100 - new_w) // 2
        canvas[y:y + new_h, x:x + new_w] = resized
        return canvas

    def classify_by_template(self, letter: np.ndarray) -> str | None:
        if len(self.templates) == 0:
            return None

        best_letter = None
        best_score = -1
        letter_mask = letter > 0
        for template_letter, template in self.templates.items():
            template_mask = template > 0
            union = np.count_nonzero(letter_mask | template_mask)
            if union == 0:
                continue
            intersection = np.count_nonzero(letter_mask & template_mask)
            score = intersection / union
            if score > best_score:
                best_score = score
                best_letter = template_letter

        if SHOW_FIXTURE_DEBUG:
            print(f"[조난자 분류:victim_clasification.template] best='{best_letter}', score={best_score:.3f}")

        if best_score < 0.20:
            return None
        return best_letter

    def classify_by_shape(self, letter: np.ndarray) -> str | None:
        mask = letter > 0
        if np.count_nonzero(mask) < 500:
            return None

        center_stroke = np.mean(mask[:, 45:55])
        top_left = np.mean(mask[5:25, 5:30])
        top_right = np.mean(mask[5:25, 70:95])
        middle_left = np.mean(mask[40:60, 5:30])
        middle_right = np.mean(mask[40:60, 70:95])

        # Ω는 중앙 세로획이 거의 없고, Φ/Ψ는 중앙 세로획이 뚜렷합니다.
        if center_stroke < 0.18:
            return "U"

        # Ψ는 상단 좌우 가지가 열려 있고, Φ는 중간 좌우 타원 획이 강합니다.
        if top_left > 0.20 and top_right > 0.20:
            return "S"
        if middle_left > 0.20 and middle_right > 0.20:
            return "H"
        return "S"

    def classify_victim(self, victim):
        letter = self.isolate_victim(victim["image"])
        if letter is None:
            return None

        letter = self.normalize_letter(letter)

        if SHOW_FIXTURE_DEBUG:
            cv.imshow("letra", letter)

        letter_key = self.classify_by_template(letter)
        if letter_key is None:
            letter_key = self.classify_by_shape(letter)

        if SHOW_FIXTURE_DEBUG:
            print(f"[조난자 분류:victim_clasification.classify_victim] 보고 코드: '{letter_key}'")

        return letter_key
