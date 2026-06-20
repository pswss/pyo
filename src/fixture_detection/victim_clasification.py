from flags import SHOW_FIXTURE_DEBUG
import cv2 as cv
import numpy as np

from fixture_detection.color_filter import ColorFilter


class VictimClassifier:
    """
    v26 조난자 심볼(Φ/Ψ/Ω)을 분류하는 클래스입니다.

    v26 심볼 → 보고 글자:
    - Φ (Phi)   → 'H' (Harmed):   타원+세로바 → 내부에 닫힌 공간(구멍) 존재
    - Ψ (Psi)   → 'S' (Stable):   삼지창 → 구멍 없음, 하단 중앙에 세로바 존재
    - Ω (Omega) → 'U' (Unharmed): 말발굽 → 구멍 없음, 하단 중앙 비어있음(두 발 사이 간격)

    분류 방법:
    1. 심볼 이진화 및 최대 blob 추출
    2. 컨투어 계층 분석 → 닫힌 내부 공간(구멍) 유무 판별
       - 구멍 있음 → Φ → 'H'
    3. 구멍 없을 때 하단 중앙 픽셀 밀도로 Ψ/Ω 구분
       - 하단 중앙 밀도 높음 → Ψ → 'S' (세로바)
       - 하단 중앙 밀도 낮음 → Ω → 'U' (두 발 사이 간격)
    """
    def __init__(self):
        self.victim_letter_filter = ColorFilter(lower_hsv=(0, 0, 0), upper_hsv=(0, 0, 130))
        self.min_hole_area = 80

    def isolate_victim(self, image):
        binary = self.victim_letter_filter.filter(image)
        letter = self.get_biggest_blob(binary)
        if SHOW_FIXTURE_DEBUG:
            cv.imshow("thresh", binary)
        return letter

    def get_biggest_blob(self, binary_image: np.ndarray) -> np.ndarray:
        contours, _ = cv.findContours(binary_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        max_size = 0
        biggest_blob = None
        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            if w * h > max_size:
                biggest_blob = binary_image[y:y + h, x:x + w]
                max_size = w * h
        return biggest_blob

    def classify_victim(self, victim):
        letter = self.isolate_victim(victim["image"])
        if letter is None or letter.size == 0:
            return None

        letter = cv.resize(letter, (100, 100), interpolation=cv.INTER_AREA)

        if SHOW_FIXTURE_DEBUG:
            cv.imshow("victim_symbol", letter)

        # 1) 닫힌 내부 공간(구멍) 검출 — Φ 판별
        hole_count = self._count_holes(letter)

        if hole_count >= 1:
            if SHOW_FIXTURE_DEBUG:
                print(f"[조난자 분류] Φ(Phi) 감지: 내부 구멍 {hole_count}개 → 'H'")
            return 'H'

        # 2) 하단 중앙 픽셀 밀도 — Ψ vs Ω 판별
        bottom_center = letter[70:95, 35:65]
        if bottom_center.size > 0:
            density = np.count_nonzero(bottom_center) / bottom_center.size
        else:
            density = 0

        if SHOW_FIXTURE_DEBUG:
            print(f"[조난자 분류] 하단 중앙 밀도: {density:.3f}")

        if density > 0.25:
            if SHOW_FIXTURE_DEBUG:
                print(f"[조난자 분류] Ψ(Psi) 감지: 하단 밀도={density:.3f} → 'S'")
            return 'S'
        else:
            if SHOW_FIXTURE_DEBUG:
                print(f"[조난자 분류] Ω(Omega) 감지: 하단 밀도={density:.3f} → 'U'")
            return 'U'

    def _count_holes(self, binary_100x100):
        """RETR_CCOMP 계층으로 닫힌 내부 공간(구멍) 개수를 반환합니다."""
        contours, hierarchy = cv.findContours(
            binary_100x100, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)

        if hierarchy is None:
            return 0

        count = 0
        for i, h in enumerate(hierarchy[0]):
            if h[3] != -1:  # parent 있음 = 내부 컨투어(구멍)
                area = cv.contourArea(contours[i])
                if area >= self.min_hole_area:
                    count += 1
        return count
