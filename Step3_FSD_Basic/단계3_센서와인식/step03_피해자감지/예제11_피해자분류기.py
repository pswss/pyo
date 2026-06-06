"""
[예제 11] 피해자 분류기
------------------------
터미널에서 실행: python 예제11_피해자분류기.py

실제 프로젝트의 fixture_clasification.py 원리를 쉽게 재현합니다.
가상의 피해자 이미지를 만들어서 분류 알고리즘을 테스트합니다.

[학생 과제]
  THRESHOLDS 값을 바꿔가며 분류 정확도가 어떻게 변하는지 관찰하세요.
"""

import numpy as np
import cv2 as cv

# ── 설정: 여기 값을 바꿔보세요! ──────────────────────────────
THRESHOLDS = {
    "H": 0.25,   # 빨강 비율이 이 이상이면 → Harmed (부상자)
    "S": 0.25,   # 노랑 비율이 이 이상이면 → Stable (안정)
    "U": 0.40,   # 흰색 비율이 이 이상이면 → Unharmed (무사)
}

# ── 색상 범위 (HSV) ───────────────────────────────────────────
COLOR_RANGES = {
    "red":    (np.array([0,  100, 100]), np.array([10,  255, 255])),
    "yellow": (np.array([20, 100, 100]), np.array([35,  255, 255])),
    "green":  (np.array([40, 100, 50]),  np.array([80,  255, 255])),
    "white":  (np.array([0,  0,   200]), np.array([180, 30,  255])),
    "black":  (np.array([0,  0,   0]),   np.array([180, 255, 50 ])),
}

# ── 가상 피해자 이미지 생성 ──────────────────────────────────
def make_victim_image(victim_type: str):
    """
    피해자 타입별 가상 이미지 생성
    실제 대회에서는 카메라로 촬영한 이미지를 사용
    """
    img = np.zeros((100, 100, 3), dtype=np.uint8)

    if victim_type == "H":     # 빨강(위) + 흰색(아래)
        img[:50, :] = (0, 0, 255)    # BGR: 빨강
        img[50:, :] = (255, 255, 255) # BGR: 흰색

    elif victim_type == "S":   # 노랑(위) + 초록(아래)
        img[:50, :] = (0, 255, 255)  # BGR: 노랑
        img[50:, :] = (0, 200, 0)    # BGR: 초록

    elif victim_type == "U":   # 초록(위) + 흰색(아래)
        img[:50, :] = (0, 200, 0)    # BGR: 초록
        img[50:, :] = (255, 255, 255) # BGR: 흰색

    elif victim_type == "NOISE":  # 랜덤 노이즈 (피해자 아님)
        img = np.random.randint(100, 200, (100, 100, 3), dtype=np.uint8)

    return img

# ── 핵심: 분류 알고리즘 ───────────────────────────────────────
def classify_victim(img_bgr):
    """
    이미지에서 픽셀 색상 비율을 분석해 피해자 타입을 반환합니다.
    실제 프로젝트의 fixture_clasification.py 와 동일한 원리!
    """
    img_hsv = cv.cvtColor(img_bgr, cv.COLOR_BGR2HSV)
    total_pixels = img_bgr.shape[0] * img_bgr.shape[1]

    # 각 색상의 픽셀 수 계산
    counts = {}
    for color_name, (lower, upper) in COLOR_RANGES.items():
        mask = cv.inRange(img_hsv, lower, upper)
        counts[color_name] = cv.countNonZero(mask)

    # 비율 계산
    ratios = {k: v / total_pixels for k, v in counts.items()}

    # 분류 규칙
    if ratios["red"] >= THRESHOLDS["H"]:
        return "H", ratios
    elif ratios["yellow"] >= THRESHOLDS["S"]:
        return "S", ratios
    elif ratios["white"] >= THRESHOLDS["U"]:
        return "U", ratios
    else:
        return "?", ratios   # 알 수 없음

# ── 테스트 실행 ───────────────────────────────────────────────
print("=" * 60)
print("  피해자 분류기 테스트")
print("=" * 60)
print(f"  임계값: H={THRESHOLDS['H']:.0%}  S={THRESHOLDS['S']:.0%}  U={THRESHOLDS['U']:.0%}")
print()

test_cases = ["H", "S", "U", "NOISE"]
all_images = []

for victim_type in test_cases:
    img = make_victim_image(victim_type)
    predicted, ratios = classify_victim(img)

    correct = (predicted == victim_type) or (victim_type == "NOISE" and predicted == "?")
    mark = "✓" if correct else "✗"

    print(f"  실제: {victim_type:5s} | 예측: {predicted} | {mark}")
    print(f"    빨강:{ratios['red']:.1%}  노랑:{ratios['yellow']:.1%}  "
          f"초록:{ratios['green']:.1%}  흰색:{ratios['white']:.1%}  검정:{ratios['black']:.1%}")
    print()

    all_images.append((victim_type, img, predicted))

# ── 시각화 ───────────────────────────────────────────────────
print("창이 열립니다. 아무 키나 누르면 닫힙니다.")

display = np.zeros((120, 440, 3), dtype=np.uint8)
for i, (label, img, pred) in enumerate(all_images):
    x = i * 110
    display[10:110, x:x+100] = img
    color = (0, 255, 0) if (pred == label or (label=="NOISE" and pred=="?")) else (0, 0, 255)
    cv.putText(display, f"{label}->{pred}", (x+2, 15),
               cv.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

cv.imshow("피해자 분류 결과 (초록=정답, 빨강=오답)", display)
cv.waitKey(0)
cv.destroyAllWindows()

print("\n[도전 과제]")
print("  1. THRESHOLDS['H']를 0.1로 줄이면 어떤 문제가 생기나요?")
print("  2. 빨간색과 노란색이 섞인 이미지(NOISE)를 H로 오분류하지 않으려면?")
print("  3. 새로운 피해자 타입을 추가하는 규칙을 직접 만들어보세요!")
