"""
[예제 17] 파라미터 튜닝 가이드
--------------------------------
터미널에서 실행: python 예제17_파라미터튜닝가이드.py

대회 점수를 높이기 위해 수정할 수 있는 파라미터들과
각 변경이 로봇 동작에 미치는 영향을 시뮬레이션합니다.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../src'))

import math
import numpy as np

print("=" * 60)
print("  대회 최적화 파라미터 가이드")
print("=" * 60)

# ─────────────────────────────────────────────────────────────
print("\n[1] 복귀 임계값 — agent.py:60")
print("-" * 60)

robot_diameter = 0.074  # 로봇 직경 74mm

current   = 0.04   # 현재 설정값
improved  = 0.08   # 개선 제안값

print(f"  로봇 직경:     {robot_diameter*100:.1f}cm")
print(f"  현재 임계값:   {current*100:.0f}cm  ← 로봇보다 작아서 도착 못 할 수 있음!")
print(f"  개선 임계값:   {improved*100:.0f}cm  ← 로봇 직경과 비슷, 훨씬 안정적")
print()
print("  수정 위치: src/agent/agent.py 60번 줄")
print("  self.end_reached_distance_threshold = 0.08  # 0.04 → 0.08")

# ─────────────────────────────────────────────────────────────
print("\n[2] A* 경로 선호도 가중치 — efficient_a_star.py:26")
print("-" * 60)

def simulate_path_score(weight, path_length=30, wall_distance=5):
    """가중치에 따른 경로 점수 시뮬레이션"""
    preference_cost = weight * (10 - wall_distance)  # 벽과 가까울수록 높음
    total_cost = path_length + preference_cost
    return total_cost

weights = [0, 1, 2, 5, 10]
print(f"  {'가중치':>6} | {'벽 가까운 경로 점수':>18} | {'벽 먼 경로 점수':>15} | 특성")
print("  " + "-" * 65)
for w in weights:
    near = simulate_path_score(w, 28, wall_distance=2)   # 벽 가까운 경로
    far  = simulate_path_score(w, 35, wall_distance=7)   # 벽 먼 경로(더 긴)
    winner = "벽 가까운 경로 선택" if near < far else "벽 먼 경로 선택"
    print(f"  {w:>6} | {near:>18.1f} | {far:>15.1f} | {winner}")

print()
print("  권장값: 2 (기본) ~ 5 (더 안전한 경로)")
print("  수정 위치: src/algorithms/np_bool_array/efficient_a_star.py 26번 줄")
print("  self.preference_weight = 2  # ← 여기를 바꾸세요")

# ─────────────────────────────────────────────────────────────
print("\n[3] 피해자 감지 임계값 — fixture_clasification.py")
print("-" * 60)

def simulate_detection(actual_red_ratio, threshold):
    """임계값에 따른 피해자 감지 시뮬레이션"""
    return actual_red_ratio >= threshold

test_images = [
    ("진짜 H 피해자 (빨강 40%)", 0.40),
    ("희미한 H (빨강 20%)",      0.20),
    ("배경 잡음 (빨강 15%)",     0.15),
    ("노란 조명 영향 (빨강 12%)", 0.12),
]

thresholds = [0.15, 0.25, 0.35]
print(f"  {'이미지':>28} | {'실제빨강':>8}", end="")
for t in thresholds:
    print(f" | 임계={t:.0%}", end="")
print()
print("  " + "-" * 75)

for label, ratio in test_images:
    print(f"  {label:>28} | {ratio:>8.0%}", end="")
    for t in thresholds:
        detected = simulate_detection(ratio, t)
        mark = "✓감지" if detected else "✗미감지"
        print(f" | {mark:>7}", end="")
    print()

print()
print("  권장: 임계값 0.25 (False Negative와 False Positive 균형)")
print("  수정 위치: src/fixture_detection/fixture_clasification.py")

# ─────────────────────────────────────────────────────────────
print("\n[4] 팀 전략 수립 가이드")
print("-" * 60)

strategies = {
    "안전 우선형":  {"preference_weight": 5,  "threshold": 0.30, "return_dist": 0.10},
    "균형형 (기본)":{"preference_weight": 2,  "threshold": 0.25, "return_dist": 0.08},
    "공격적 탐색형":{"preference_weight": 1,  "threshold": 0.20, "return_dist": 0.06},
}

print(f"  {'전략':>15} | {'경로가중치':>10} | {'감지임계':>8} | {'복귀거리':>8}")
print("  " + "-" * 50)
for name, params in strategies.items():
    print(f"  {name:>15} | {params['preference_weight']:>10} | "
          f"{params['threshold']:>8.0%} | {params['return_dist']*100:>7.0f}cm")

print()
print("  [팀별 과제]")
print("  각 팀이 전략을 선택하고 실제 시뮬레이션에서 테스트 후 발표하세요!")
print()
print("  수정 파일 목록:")
print("    src/agent/agent.py                              ← 복귀 거리")
print("    src/algorithms/np_bool_array/efficient_a_star.py ← A* 가중치")
print("    src/fixture_detection/fixture_clasification.py ← 감지 임계값")
print()
print("  수정 후 반드시 재컴파일!")
print("  stickytape.exe src/main.py --add-python-path src/ | Out-File -Encoding utf8 compiled.py")
