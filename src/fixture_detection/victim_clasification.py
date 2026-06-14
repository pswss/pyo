from flags import SHOW_FIXTURE_DEBUG
import cv2 as cv
import numpy as np
import random


from fixture_detection.color_filter import ColorFilter

class VictimClassifier:
    """
    조난자(victim) 이미지에서 글자(H/S/U)를 분류하는 클래스입니다.

    분류 방법:
    1. 이진화하여 글자 픽셀 추출 (어두운 색 = 글자)
    2. 가장 큰 blob(윤곽선 영역)을 글자 영역으로 선택
    3. 글자를 100×100으로 리사이즈
    4. 무게중심(centroid)으로 글자 수평 중심 보정
    5. 세 영역(상/중/하)에서 픽셀 수 계산
    6. 영역별 픽셀 유무 패턴으로 H/S/U 분류:
       - H: 중간만 있음 (가로 획이 중간)
       - S: 상+중+하 또는 상+하 (S자 굴곡)
       - U: 상/중 없음, 하만 있음 (U자 아랫부분)
    """
    def __init__(self):
        self.white = 255

        # 글자 픽셀 추출 필터 (어두운 픽셀 = 글자)
        self.victim_letter_filter = ColorFilter(lower_hsv=(0, 0, 0), upper_hsv=(0, 0, 130))

        self.top_image_reduction = 0
        self.horizontal_image_reduction = 1

        # 각 영역의 크기 설정
        self.area_width = 10
        self.area_height = 30
        # 각 영역에서 글자로 인정하는 최소 픽셀 수 (20% 기준)
        self.min_count_in_area = int(self.area_height * self.area_width * 0.2)

        # 세 영역(상/중/하) 정의: ((y 시작, y 끝), (x 오프셋 시작, x 오프셋 끝))
        # x 오프셋은 글자 무게중심(center)을 기준으로 상대적 위치
        self.areas = {
            "top":    ((0, self.area_height),                                       (self.area_width // -2, self.area_width // 2)),
            "middle": ((50 - self.area_height // 2, 50 + self.area_height // 2),   (self.area_width // -2, self.area_width // 2)),
            "bottom": ((100 - self.area_height, 100),                              (self.area_width // -2, self.area_width // 2))
            }

        # 글자별 영역 패턴 정의 (True = 해당 영역에 픽셀 있음)
        self.letters = {
            "H":[{'top': False, 'middle': True, 'bottom': False}],

            "S":[{'top': True, 'middle': True, 'bottom': True},
                 {'top': True, 'middle': False, 'bottom': True}],

            "U":[{'top': False, 'middle': False, 'bottom': True},
                 {'top': False, 'middle': False, 'bottom': False}],

            }

    def crop_white(self, binaryImg):
        """이진 이미지에서 흰색(255) 픽셀이 있는 영역의 경계 박스로 크롭합니다."""
        white = 255
        rows, cols = np.where(binaryImg == white)
        if len(rows) == 0 or len(cols) == 0:
            return binaryImg
        else:
            minY, maxY = np.min(rows), np.max(rows)
            minX, maxX = np.min(cols), np.max(cols)
            return binaryImg[minY:maxY+1, minX:maxX+1]

    def isolate_victim(self, image):
        """
        조난자 이미지에서 글자만 추출합니다.
        이진화 후 가장 큰 blob(글자 윤곽선)을 선택합니다.
        """
        binary = self.victim_letter_filter.filter(image)
        letter = self.get_biggest_blob(binary)

        if SHOW_FIXTURE_DEBUG:
            cv.imshow("thresh", binary)

        return letter

    def get_biggest_blob(self, binary_image: np.ndarray) -> np.ndarray:
        """
        이진 이미지에서 가장 면적이 큰 blob(연결된 픽셀 그룹)을 반환합니다.
        글자 외의 작은 노이즈 blob을 제거하는 역할을 합니다.
        """
        contours, _ = cv.findContours(binary_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        max_size = 0
        biggest_blob = None
        for c0 in contours:
            x, y, w, h = cv.boundingRect(c0)

            if w*h > max_size:
                biggest_blob = binary_image[y:y + h, x:x + w]
                max_size = w*h

        return biggest_blob

    def classify_victim(self, victim):
        """
        조난자 이미지에서 글자를 분류하여 H/S/U 중 하나를 반환합니다.

        1. 글자 추출 및 100×100 리사이즈
        2. 모멘트로 무게중심 계산 후 수평 중심 보정
        3. 상/중/하 영역별 픽셀 수 계산
        4. 패턴 매칭으로 글자 분류
        5. 매칭 실패 시 무작위 선택 (fallback)
        """
        letter = self.isolate_victim(victim["image"])

        letter = cv.resize(letter, (100, 100), interpolation=cv.INTER_AREA)

        # 글자의 무게중심(centroid)을 계산하여 수평 중심을 글자 중앙으로 보정
        moments = cv.moments(letter)
        center = letter.shape[1] / 2
        offset = (moments["m10"] / moments["m00"] - center) * 1

        center -= offset
        center = round(center)

        if SHOW_FIXTURE_DEBUG:
            cv.imshow("letra", letter)

        letter_color = cv.cvtColor(letter, cv.COLOR_GRAY2BGR)

        # 각 영역의 이미지 슬라이스 추출 (무게중심 보정 적용)
        images = {
            "top":    letter[self.areas["top"][0][0]   :self.areas["top"][0][1],    self.areas["top"][1][0]    + center:self.areas["top"][1][1]    + center],
            "middle": letter[self.areas["middle"][0][0]:self.areas["middle"][0][1], self.areas["middle"][1][0] + center:self.areas["middle"][1][1] + center],
            "bottom": letter[self.areas["bottom"][0][0]:self.areas["bottom"][0][1], self.areas["bottom"][1][0] + center:self.areas["bottom"][1][1] + center]
            }

        if SHOW_FIXTURE_DEBUG:
            cv.rectangle(letter_color,(self.areas["top"][1][0] + center, self.areas["top"][0][0]),        (self.areas["top"][1][1] + center, self.areas["top"][0][1]     ), (0, 255, 0), 1)
            cv.rectangle(letter_color, (self.areas["middle"][1][0] + center, self.areas["middle"][0][0]), (self.areas["middle"][1][1]+ center, self.areas["middle"][0][1]), (0, 0, 255), 1)
            cv.rectangle(letter_color,(self.areas["bottom"][1][0] + center , self.areas["bottom"][0][0]),  (self.areas["bottom"][1][1]+ center, self.areas["bottom"][0][1]), (225, 0, 255), 1)
            cv.imshow("letter_color", letter_color)

        # 각 영역의 흰색(글자) 픽셀 수가 최소 기준을 초과하는지 확인
        counts = {}
        for key in images.keys():
            count = 0
            for row in images[key]:
                for pixel in row:
                    if pixel == self.white:
                        count += 1

            counts[key] = count > self.min_count_in_area

        # 정의된 패턴과 비교하여 글자 분류
        for letter_key in self.letters.keys():
            for template in self.letters[letter_key]:
                if counts == template:
                    print(f"[조난자 분류:victim_clasification.classify_victim] 글자 인식 성공: '{letter_key}' | 영역 패턴: {counts}")
                    return letter_key

        # 패턴 매칭 실패 시 무작위 글자 반환 (대회 규정에 따라 점수 차감보다 나음)
        return random.choice(list(self.letters.keys()))
