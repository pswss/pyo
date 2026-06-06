"""
[예제 13] 라이다 포인트 → 격자 지도 변환
------------------------------------------
터미널에서 실행: python 예제13_라이다투맵.py
필요 패키지: pip install numpy matplotlib opencv-python

라이다 데이터가 어떻게 격자 지도가 되는지 단계별로 시각화합니다.
실제 wall_mapper.py 의 핵심 로직을 단순화한 버전입니다.
"""

import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import math

# ── 가상 환경 설정 ────────────────────────────────────────────
RESOLUTION  = 20   # 20격자/미터 (1격자 = 5cm)
ROBOT_DIAM  = 0.074  # 로봇 직경 (74mm)
ROBOT_R_PX  = int(ROBOT_DIAM / 2 * RESOLUTION)  # 픽셀 단위 반지름

GRID_SIZE   = 60   # 격자 크기 (60×60)
GRID_ORIGIN = 30   # 로봇이 시작하는 격자 위치

# ── 가상 라이다 데이터 생성 ──────────────────────────────────
def simulate_lidar_scans(robot_x, robot_y, n_rays=60, noise_prob=0.05):
    """
    가상 미로에서 라이다 스캔 시뮬레이션.
    noise_prob: 노이즈 포인트가 나올 확률
    """
    walls = [
        (0.0, -0.5, 0.0,  0.5),  # 왼쪽 벽
        (1.5, -0.5, 1.5,  0.5),  # 오른쪽 벽
        (0.0, -0.5, 1.5, -0.5),  # 아래 벽
        (0.0,  0.5, 1.5,  0.5),  # 위 벽
        (0.5,  0.0, 0.5,  0.5),  # 내부 벽
        (0.8, -0.5, 0.8,  0.0),  # 내부 벽 2
    ]

    points = []
    for i in range(n_rays):
        angle = 2 * math.pi * i / n_rays
        dx, dy = math.cos(angle), math.sin(angle)

        if np.random.random() < noise_prob:
            # 노이즈: 잘못된 거리 반환
            dist = np.random.uniform(0.1, 0.5)
            points.append((dx * dist, dy * dist))
            continue

        min_dist = 2.0
        for (wx1, wy1, wx2, wy2) in walls:
            wdx, wdy = wx2 - wx1, wy2 - wy1
            denom = dx * wdy - dy * wdx
            if abs(denom) < 1e-10:
                continue
            t = ((wx1 - robot_x) * wdy - (wy1 - robot_y) * wdx) / denom
            u = ((wx1 - robot_x) * dy  - (wy1 - robot_y) * dx)  / denom
            if t >= 0 and 0 <= u <= 1 and t < min_dist:
                min_dist = t

        if min_dist < 2.0:
            points.append((dx * min_dist, dy * min_dist))

    return points

# ── 격자 지도 클래스 ─────────────────────────────────────────
class WallMapperSimple:
    def __init__(self, grid_size, origin, resolution, robot_radius_px):
        self.size        = grid_size
        self.origin      = origin
        self.resolution  = resolution
        self.robot_r     = robot_radius_px

        # 지도 배열들
        self.detected    = np.zeros((grid_size, grid_size), dtype=np.int32)
        self.walls       = np.zeros((grid_size, grid_size), dtype=bool)
        self.traversable = np.zeros((grid_size, grid_size), dtype=bool)
        self.noise_threshold = 3  # 3번 이상 감지 → 벽 확정

        # 로봇 크기 템플릿 (원형)
        d = robot_radius_px * 2 + 1
        self.robot_template = np.zeros((d, d), dtype=np.uint8)
        cv.circle(self.robot_template, (robot_radius_px, robot_radius_px),
                  robot_radius_px, 255, -1)

    def to_grid(self, real_x, real_y):
        """실제 좌표 → 격자 인덱스"""
        col = int(real_x * self.resolution) + self.origin
        row = int(real_y * self.resolution) + self.origin
        return row, col

    def update(self, robot_x, robot_y, points):
        """라이다 포인트로 지도 업데이트"""
        for (px, py) in points:
            abs_x = robot_x + px
            abs_y = robot_y + py
            row, col = self.to_grid(abs_x, abs_y)

            if 0 <= row < self.size and 0 <= col < self.size:
                self.detected[row, col] += 1
                # 3번 이상 감지되면 벽 확정
                if self.detected[row, col] >= self.noise_threshold:
                    self.walls[row, col] = True

        self.update_traversable()

    def update_traversable(self):
        """벽에서 로봇 반지름만큼 확장 → 이동 불가 구역"""
        walls_uint8 = self.walls.astype(np.uint8) * 255
        expanded = cv.filter2D(walls_uint8, -1, self.robot_template)
        self.traversable = expanded > 0

# ── 시뮬레이션 실행 ──────────────────────────────────────────
mapper = WallMapperSimple(GRID_SIZE, GRID_ORIGIN, RESOLUTION, ROBOT_R_PX)

robot_x, robot_y = 0.0, 0.0
print(f"로봇 직경: {ROBOT_DIAM*100:.1f}cm = {ROBOT_R_PX}px 반지름")
print(f"격자 해상도: {RESOLUTION}px/m")
print()

# 여러 번 스캔하여 노이즈 필터링 효과 보기
fig, axes = plt.subplots(2, 3, figsize=(15, 10))
fig.suptitle("라이다 → 격자 지도 변환 과정", fontsize=14, fontweight='bold')

scan_counts = [1, 3, 10]
for col_idx, n_scans in enumerate(scan_counts):
    mapper2 = WallMapperSimple(GRID_SIZE, GRID_ORIGIN, RESOLUTION, ROBOT_R_PX)

    for _ in range(n_scans):
        pts = simulate_lidar_scans(robot_x, robot_y, noise_prob=0.08)
        mapper2.update(robot_x, robot_y, pts)

    # 위 행: 감지 횟수 히트맵
    im = axes[0, col_idx].imshow(mapper2.detected, cmap='hot', interpolation='nearest')
    axes[0, col_idx].set_title(f"감지 횟수 히트맵 ({n_scans}번 스캔)")
    axes[0, col_idx].plot(GRID_ORIGIN, GRID_ORIGIN, 'bo', markersize=8)
    plt.colorbar(im, ax=axes[0, col_idx])

    # 아래 행: 최종 지도
    display = np.ones((GRID_SIZE, GRID_SIZE, 3), dtype=np.uint8) * 200
    display[mapper2.traversable] = [100, 100, 100]  # 회색: 이동 불가(벽 주변)
    display[mapper2.walls]       = [0,   0,   0  ]  # 검정: 벽
    display[GRID_ORIGIN, GRID_ORIGIN] = [0, 0, 255]  # 파랑: 로봇

    axes[1, col_idx].imshow(display, interpolation='nearest')
    axes[1, col_idx].set_title(f"최종 지도 ({n_scans}번 스캔)\n"
                                f"흰색=이동가능 / 회색=위험 / 검정=벽")

    wall_count = mapper2.walls.sum()
    print(f"{n_scans:2d}번 스캔 후: 벽 격자 {wall_count}개 확정")

plt.tight_layout()
plt.show()

print("\n[핵심 정리]")
print("  노이즈 필터링: 같은 위치가 3번 이상 감지되면 벽으로 확정")
print("  로봇 크기 고려: 벽 주변 로봇 반지름만큼 이동 불가로 표시")
print(f"  이 로봇은 {ROBOT_DIAM*100:.1f}cm 이므로 벽에서 {ROBOT_R_PX}격자({ROBOT_DIAM/2*100:.1f}cm) 거리 유지")
