"""통로 색 → 규정 문자 코드 매핑 테스트 (MB 일치율 직결).

규정 §5.6.10.b.ii: 통로는 1↔2='b', 1↔3='y', 1↔4='g', 2↔3='p', 2↔4='o', 3↔4='r' 문자.
기존 코드는 "6"/"7"/"8" 숫자 → 채점 시 통로 셀 전부 불일치 + y/g/o 통로 미인식.
색상(hue) 기준 매핑 검증: 파랑=b, 노랑=y, 초록=g, 보라=p, 주황=o, 빨강=r.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np
import cv2 as cv

from final_matrix_creation.final_matrix_creator import FloorMatrixCreator


def make_tile(hsv):
    """HSV 단색으로 채운 풀타일(BGR) 생성. square_size_px=10 → 내부 2배=20."""
    img = np.zeros((20, 20, 3), np.uint8)
    img[:, :] = hsv
    return cv.cvtColor(img, cv.COLOR_HSV2BGR)


def code_of(hsv):
    fmc = FloorMatrixCreator(square_size_px=10)
    tile = make_tile(hsv)
    return fmc._FloorMatrixCreator__get_square_color(0, 0, 20, 20, tile)


def test_passage_letters():
    cases = {
        "b": (120, 190, 140),  # 파랑 (기존 "6" 실측 범위 중앙)
        "p": (132, 170, 110),  # 보라 (기존 "8" 실측 범위 중앙)
        "r": (0, 190, 140),    # 빨강 (기존 "7" 실측 범위 중앙)
        "y": (30, 200, 200),   # 노랑 ★시뮬 튜닝
        "g": (60, 200, 150),   # 초록 ★시뮬 튜닝
        "o": (15, 200, 200),   # 주황 ★시뮬 튜닝
    }
    for letter, hsv in cases.items():
        got = code_of(hsv)
        assert got == letter, f"{letter} 통로색 {hsv} → '{got}' (기대 '{letter}')"


def test_existing_codes_unchanged():
    assert code_of((0, 0, 20)) == "2"        # 구멍 (검정)
    assert code_of((19, 130, 100)) == "3"    # 늪 (갈색)
    assert code_of((110, 60, 130)) == "4"    # 체크포인트 (은색)


if __name__ == "__main__":
    test_passage_letters()
    test_existing_codes_unchanged()
    print("OK")
