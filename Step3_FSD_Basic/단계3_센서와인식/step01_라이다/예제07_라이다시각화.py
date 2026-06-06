"""
[예제 07] 라이다 포인트 클라우드 시각화
-----------------------------------------
Erebus 없이 터미널에서 실행하는 시뮬레이션입니다.
가상의 라이다 데이터를 만들어 matplotlib으로 시각화합니다.

실행 방법: python 예제07_라이다시각화.py
필요 패키지: pip install matplotlib numpy
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import platform  # 운영체제 확인을 위해 추가

# ── 한글 폰트 및 마이너스 기호 설정 ─────────────────────────
if platform.system() == 'Windows':
    plt.rc('font', family='Malgun Gothic')  # 윈도우 - 맑은 고딕
elif platform.system() == 'Darwin':
    plt.rc('font', family='AppleGothic')    # Mac - 애플 고딕
else:
    plt.rc('font', family='NanumGothic')    # 리눅스 - 나눔 고딕 (설치 필요할 수 있음)

# 축의 마이너스(-) 기호가 깨지는 현상 방지
plt.rc('axes', unicode_minus=False)

# ── 가상 환경 설정 ────────────────────────────────────────────
# 로봇 주변에 벽이 있는 가상의 미로 방 하나를 만듭니다

ROOM_WIDTH  = 2.0   # 방 가로 (미터)
ROOM_HEIGHT = 1.5   # 방 세로 (미터)
ROBOT_X = 0.5       # 로봇 x 위치
ROBOT_Y = 0.4       # 로봇 y 위치

def simulate_lidar(robot_x, robot_y, n_rays=180, max_range=2.0):
    """
    가상 라이다 시뮬레이션.
    로봇 주변 360도를 n_rays 개의 레이저로 스캔합니다.

    반환: [(x, y), ...] — 로봇 기준 상대 좌표 포인트들
    """
    # 벽 정의: [(x1, y1, x2, y2), ...]
    walls = [
        (0, 0, ROOM_WIDTH, 0),               # 아래 벽
        (0, 0, 0, ROOM_HEIGHT),              # 왼쪽 벽
        (ROOM_WIDTH, 0, ROOM_WIDTH, ROOM_HEIGHT),  # 오른쪽 벽
        (0, ROOM_HEIGHT, ROOM_WIDTH, ROOM_HEIGHT), # 위 벽
        # 중간에 기둥 하나
        (0.8, 0.3, 1.0, 0.3),
        (0.8, 0.3, 0.8, 0.7),
        (0.8, 0.7, 1.0, 0.7),
        (1.0, 0.3, 1.0, 0.7),
    ]

    points = []
    for i in range(n_rays):
        angle = 2 * math.pi * i / n_rays
        dx = math.cos(angle)
        dy = math.sin(angle)

        # 각 레이저가 벽과 충돌하는 거리 계산
        min_dist = max_range
        for (wx1, wy1, wx2, wy2) in walls:
            dist = ray_wall_intersection(
                robot_x, robot_y, dx, dy,
                wx1, wy1, wx2, wy2
            )
            if dist is not None and 0 < dist < min_dist:
                min_dist = dist

        if min_dist < max_range:
            # 로봇 기준 상대 좌표로 변환
            rel_x = dx * min_dist
            rel_y = dy * min_dist
            points.append((rel_x, rel_y))

    return points

def ray_wall_intersection(rx, ry, dx, dy, wx1, wy1, wx2, wy2):
    """레이(광선)와 선분(벽)의 교점까지의 거리 계산"""
    # 선분 방향 벡터
    wx = wx2 - wx1
    wy = wy2 - wy1
    # 행렬식으로 교점 계산
    denom = dx * wy - dy * wx
    if abs(denom) < 1e-10:
        return None  # 평행 → 교점 없음
    t = ((wx1 - rx) * wy - (wy1 - ry) * wx) / denom
    u = ((wx1 - rx) * dy - (wy1 - ry) * dx) / denom
    if t >= 0 and 0 <= u <= 1:
        return t
    return None

# ── 시각화 ────────────────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
fig.suptitle("라이다 포인트 클라우드 시각화", fontsize=14, fontweight='bold')

# 왼쪽: 실제 환경 뷰
ax1.set_title("실제 환경 (위에서 본 뷰)")
ax1.set_xlim(-0.1, ROOM_WIDTH + 0.1)
ax1.set_ylim(-0.1, ROOM_HEIGHT + 0.1)
ax1.set_aspect('equal')
ax1.add_patch(patches.Rectangle((0,0), ROOM_WIDTH, ROOM_HEIGHT,
                                  fill=False, edgecolor='black', linewidth=2))
ax1.add_patch(patches.Rectangle((0.8, 0.3), 0.2, 0.4,
                                  fill=True, facecolor='gray', edgecolor='black'))
ax1.plot(ROBOT_X, ROBOT_Y, 'bo', markersize=10, label='로봇')

points = simulate_lidar(ROBOT_X, ROBOT_Y)
abs_x = [ROBOT_X + p[0] for p in points]
abs_y = [ROBOT_Y + p[1] for p in points]
ax1.scatter(abs_x, abs_y, c='red', s=5, alpha=0.6, label='라이다 포인트')

for p in points[::10]:  # 10개마다 레이저 선 표시
    ax1.plot([ROBOT_X, ROBOT_X + p[0]], [ROBOT_Y, ROBOT_Y + p[1]],
             'r-', alpha=0.2, linewidth=0.5)

ax1.legend()
ax1.set_xlabel("X (미터)")
ax1.set_ylabel("Y (미터)")
ax1.grid(True, alpha=0.3)

# 오른쪽: 로봇 기준 상대 좌표 뷰 (실제 라이다 데이터와 동일한 형태)
ax2.set_title("라이다 데이터 (로봇 기준 상대 좌표)")
rel_x = [p[0] for p in points]
rel_y = [p[1] for p in points]
ax2.scatter(rel_x, rel_y, c='red', s=8, alpha=0.8, label='포인트 클라우드')
ax2.plot(0, 0, 'bo', markersize=12, label='로봇 (원점)')
ax2.axhline(0, color='gray', linewidth=0.5)
ax2.axvline(0, color='gray', linewidth=0.5)
ax2.set_aspect('equal')
ax2.legend()
ax2.set_xlabel("X (미터, 로봇 기준)")
ax2.set_ylabel("Y (미터, 로봇 기준)")
ax2.grid(True, alpha=0.3)
ax2.set_title("라이다 데이터 (로봇 기준 상대 좌표)\n← 실제 프로젝트 get_point_cloud() 반환값과 동일")

print(f"총 {len(points)}개의 포인트 감지됨")
print(f"가장 가까운 포인트: {min(math.sqrt(p[0]**2 + p[1]**2) for p in points):.3f}m")
print(f"가장 먼 포인트:     {max(math.sqrt(p[0]**2 + p[1]**2) for p in points):.3f}m")
print("\n실제 프로젝트에서는:")
print("  point_cloud = robot.get_point_cloud()")
print("  for (x, y) in point_cloud:")
print("      distance = math.sqrt(x**2 + y**2)")

plt.tight_layout()
plt.show()
