"""
[예제 05] 각도 이해 — 라디안/도 변환 & Angle 클래스
----------------------------------------------------
이 파일은 Erebus 없이 일반 Python으로 실행 가능합니다.
터미널에서: python 예제05_각도이해.py

학습 목표:
  - 라디안과 도 단위 변환 연습
  - Angle 클래스의 normalize() 동작 이해
  - 우리가 고친 버그를 직접 확인
"""

import math
import sys
import os

# 프로젝트 src 경로 추가 (Angle 클래스 불러오기 위해)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../src'))

from data_structures.angle import Angle

print("=" * 55)
print("  1. 도(degree) ↔ 라디안(radian) 변환")
print("=" * 55)

conversions = [0, 30, 45, 90, 120, 180, 270, 360]
print(f"{'도':>6} | {'라디안':>10} | {'파이썬 math':>12}")
print("-" * 35)
for deg in conversions:
    rad = deg * math.pi / 180
    print(f"{deg:>6}° | {rad:>10.4f} | {rad/math.pi:.4f}π")

print()
print("=" * 55)
print("  2. Angle 클래스 사용 방법")
print("=" * 55)

a1 = Angle(math.pi)               # π 라디안 = 180도
a2 = Angle(90, Angle.DEGREES)     # 90도
a3 = Angle(270, Angle.DEGREES)    # 270도

print(f"Angle(π 라디안) → {a1.degrees:.1f}도")
print(f"Angle(90도)     → {a2.radians:.4f} 라디안")
print(f"Angle(270도)    → {a3.radians:.4f} 라디안")

print()
print("  덧셈/뺄셈도 가능!")
a4 = a2 + a3  # 90 + 270 = 360도
print(f"90도 + 270도 = {a4.degrees:.1f}도 (= {a4.radians:.4f} 라디안)")

print()
print("=" * 55)
print("  3. normalize() — 정규화")
print("=" * 55)
print("  목적: 모든 각도를 0° ~ 360° (0 ~ 2π) 범위로 맞추기")
print()

test_angles = [400, -90, -270, 720, -1.0]
print(f"{'입력값':>10} | {'정규화 후(도)':>12} | {'정규화 후(라디안)':>16}")
print("-" * 45)
for val in test_angles:
    if isinstance(val, float):
        a = Angle(val)  # 라디안으로 입력
        label = f"{val} rad"
    else:
        a = Angle(val, Angle.DEGREES)
        label = f"{val}°"
    a.normalize()
    print(f"{label:>10} | {a.degrees:>12.2f}° | {a.radians:>16.4f}")

print()
print("=" * 55)
print("  4. 버그 재현 — 우리가 고친 코드 확인")
print("=" * 55)

angle_rad = -1.0  # -1 라디안 ≈ -57.3도

buggy_result  = angle_rad + (2 + math.pi)   # 틀린 코드
correct_result = angle_rad + (2 * math.pi)  # 맞는 코드

print(f"입력: {angle_rad} 라디안 ({math.degrees(angle_rad):.1f}도)")
print()
print(f"❌ 틀린 코드 (2 + π):  {buggy_result:.4f} 라디안 = {math.degrees(buggy_result):.1f}도")
print(f"✓ 맞는 코드 (2 × π): {correct_result:.4f} 라디안 = {math.degrees(correct_result):.1f}도")
print()
print(f"실제 정답 (손 계산): -1 라디안 → {math.degrees(-1 % (2*math.pi)):.1f}도")
print(f"오차: {abs(math.degrees(buggy_result) - math.degrees(correct_result)):.1f}도 차이!")

print()
print("=" * 55)
print("  5. atan2 — 방향 계산")
print("=" * 55)

points = [
    ((0,0), (1, 0),  "동쪽"),
    ((0,0), (0, 1),  "북쪽"),
    ((0,0), (-1, 0), "서쪽"),
    ((0,0), (0, -1), "남쪽"),
    ((0,0), (1, 1),  "북동쪽"),
]

print(f"{'출발':>8} → {'목표':>8} | {'방향':>6} | {'각도':>8}")
print("-" * 45)
for (sx, sy), (tx, ty), name in points:
    angle = math.atan2(tx - sx, ty - sy)
    print(f"({sx},{sy}) → ({tx},{ty}) | {name:>6} | {math.degrees(angle):>6.1f}도")
