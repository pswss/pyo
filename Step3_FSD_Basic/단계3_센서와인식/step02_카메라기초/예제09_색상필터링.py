"""
[예제 09] 색상 필터링 — OpenCV 기초
--------------------------------------
터미널에서 실행: python 예제09_색상필터링.py
필요 패키지: pip install opencv-python numpy

학습 목표:
  - RGB와 HSV 색상 공간 이해
  - cv.inRange()로 특정 색상만 추출
  - 실제 피해자 감지에 쓰이는 기법 체험
"""

import numpy as np
import cv2 as cv

# ── 테스트 이미지 생성 (실제 카메라 대신) ──────────────────────
def make_test_image():
    """
    다양한 색상의 사각형이 있는 테스트 이미지를 생성합니다.
    실제 프로젝트에서는 robot.get_camera_images()로 얻습니다.
    """
    img = np.zeros((300, 500, 3), dtype=np.uint8)
    img[:] = (200, 200, 200)  # 회색 배경

    # 빨간 사각형 (피해자 - H 표시)
    cv.rectangle(img, (50,  50),  (150, 150), (0,   0,   255), -1)
    cv.putText(img, "RED", (70, 110), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

    # 노란 사각형 (피해자 - S 표시)
    cv.rectangle(img, (200, 50),  (300, 150), (0,   255, 255), -1)
    cv.putText(img, "YEL", (215, 110), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)

    # 초록 사각형
    cv.rectangle(img, (350, 50),  (450, 150), (0,   200, 0  ), -1)
    cv.putText(img, "GRN", (365, 110), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

    # 흰 사각형 (피해자 - U 표시)
    cv.rectangle(img, (50,  180), (150, 280), (255, 255, 255), -1)
    cv.putText(img, "WHT", (65, 240), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)

    # 검은 사각형 (피해자 - H 표시)
    cv.rectangle(img, (200, 180), (300, 280), (30,  30,  30 ), -1)
    cv.putText(img, "BLK", (215, 240), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

    return img

# ── 색상 감지 함수 ────────────────────────────────────────────
def detect_color(img_bgr, lower_hsv, upper_hsv, label):
    """
    이미지에서 HSV 범위에 해당하는 색상을 감지합니다.
    실제 fixture_detection 코드와 동일한 방식!
    """
    img_hsv = cv.cvtColor(img_bgr, cv.COLOR_BGR2HSV)
    mask    = cv.inRange(img_hsv, lower_hsv, upper_hsv)
    count   = cv.countNonZero(mask)

    # 감지된 영역을 원본 이미지에 표시
    result = img_bgr.copy()
    result[mask > 0] = (0, 255, 0)  # 초록색으로 하이라이트

    print(f"  {label:12s}: {count:5d} 픽셀 감지 {'✓' if count > 100 else '✗'}")
    return mask, count

# ── 메인 ─────────────────────────────────────────────────────
print("=" * 50)
print("  색상 필터링 실습")
print("=" * 50)

img = make_test_image()
img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

print("\n[각 색상 감지 결과]")

# 프로젝트의 실제 색상 범위 (fixture_detection 코드 참고)
color_ranges = {
    "빨강(Red)":  (np.array([0,   100, 100]), np.array([10,  255, 255])),
    "노랑(Yellow)":(np.array([20,  100, 100]), np.array([35,  255, 255])),
    "초록(Green)": (np.array([40,  100, 50]),  np.array([80,  255, 255])),
    "흰색(White)": (np.array([0,   0,   200]), np.array([180, 30,  255])),
    "검정(Black)": (np.array([0,   0,   0]),   np.array([180, 255, 50 ])),
}

masks = {}
for label, (lower, upper) in color_ranges.items():
    mask, count = detect_color(img, lower, upper, label)
    masks[label] = mask

# ── 이미지 출력 ───────────────────────────────────────────────
print("\n창이 열립니다. 아무 키나 누르면 다음으로 넘어갑니다.")

cv.imshow("원본 이미지 (BGR)", img)
cv.waitKey(0)

cv.imshow("HSV 변환 이미지", img_hsv)
cv.waitKey(0)

for label, mask in masks.items():
    highlighted = img.copy()
    highlighted[mask > 0] = (0, 255, 0)
    cv.imshow(f"감지 결과: {label}", highlighted)
    cv.imshow(f"마스크: {label}", mask)
    cv.waitKey(0)

cv.destroyAllWindows()

# ── 핵심 정리 ─────────────────────────────────────────────────
print("\n[핵심 정리]")
print("  1. BGR → HSV 변환: cv.cvtColor(img, cv.COLOR_BGR2HSV)")
print("  2. 색상 범위 마스크: cv.inRange(hsv, lower, upper)")
print("  3. 감지 픽셀 수:    cv.countNonZero(mask)")
print("\n[실제 프로젝트 연결]")
print("  src/fixture_detection/fixture_clasification.py 에서")
print("  이와 동일한 방식으로 피해자(H, S, U)를 감지합니다!")

# ── 도전 과제 ─────────────────────────────────────────────────
print("\n[도전 과제]")
print("  1. 파란색(Blue)을 감지하는 HSV 범위를 추가해보세요")
print("  2. 오렌지색(H:10~20)을 감지해보세요")
print("  3. 두 색상이 동시에 있는 영역을 찾으려면?")
print("     힌트: mask1 & mask2 (비트 AND 연산)")
