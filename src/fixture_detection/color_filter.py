import cv2 as cv
import numpy as np

class ColorFilter:
    """
    HSV 색 공간에서 지정된 범위의 색상을 추출하는 마스크 필터입니다.

    Webots 시뮬레이터의 카메라 이미지(BGR 형식)를 HSV로 변환하여
    lower_hsv ~ upper_hsv 범위에 해당하는 픽셀의 이진 마스크를 반환합니다.

    사용 예:
    - wall token 이미지에서 검정/흰색/빨강/노랑/초록/파랑 픽셀 추출
    - 바닥 타일에서 구멍/늪지대/체크포인트 색상 추출
    - 벽 색상 추출 (fixture 후보에서 벽 제거)
    """
    def __init__(self, lower_hsv, upper_hsv):
        # NumPy 배열로 변환하여 cv.inRange()에 직접 사용
        self.lower = np.array(lower_hsv)
        self.upper = np.array(upper_hsv)

    def filter(self, img):
        """
        입력 이미지를 HSV로 변환 후 lower~upper 범위의 이진 마스크를 반환합니다.
        반환값: 해당 색상 범위 픽셀은 255, 그 외는 0인 uint8 배열
        """
        # BGR → HSV 변환 (Webots 카메라는 BGR 형식)
        hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # HSV 범위 내 픽셀만 255로 표시하는 마스크 생성
        mask = cv.inRange(hsv_image, self.lower, self.upper)
        return mask
