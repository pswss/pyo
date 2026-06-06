"""
[예제 12] 격자 지도 기초
-------------------------
터미널에서 실행: python 예제12_격자지도기초.py
필요 패키지: pip install numpy matplotlib

numpy 2D 배열로 지도를 만들고 matplotlib으로 시각화합니다.
실제 프로젝트의 CompoundExpandablePixelGrid 원리를 단순화한 버전입니다.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

# ── 1. 격자 지도 기초 ─────────────────────────────────────────
print("=" * 50)
print("  1. 격자 지도 기초")
print("=" * 50)

# 10×10 격자 생성
grid = np.zeros((10, 10), dtype=bool)

# 외곽 벽 추가
grid[0, :] = True   # 위
grid[9, :] = True   # 아래
grid[:, 0] = True   # 왼쪽
grid[:, 9] = True   # 오른쪽

# 내부 벽 추가
grid[3, 2:7] = True  # 가로 벽
grid[6, 3:8] = True  # 또 다른 가로 벽
grid[3:7, 5] = True  # 세로 벽

print("격자 지도 (True=벽, False=길):")
print(grid.astype(int))  # 보기 쉽게 0/1로 출력

# ── 2. 좌표 변환 함수 ────────────────────────────────────────
print("\n" + "=" * 50)
print("  2. 좌표 변환")
print("=" * 50)

RESOLUTION = 5   # 5격자 = 1미터 (1격자 = 20cm)

def real_to_grid(real_x, real_y):
    """실제 좌표(미터) → 격자 인덱스 [행, 열]"""
    col = int(real_x * RESOLUTION)
    row = int(real_y * RESOLUTION)
    return row, col

def grid_to_real(row, col):
    """격자 인덱스 → 실제 좌표(미터) 중심점"""
    real_x = (col + 0.5) / RESOLUTION
    real_y = (row + 0.5) / RESOLUTION
    return real_x, real_y

# 테스트
test_coords = [(0.3, 0.5), (1.2, 0.8), (1.9, 1.9)]
print(f"{'실제 좌표':>15} | {'격자 인덱스':>12} | {'복원 좌표':>15}")
print("-" * 50)
for rx, ry in test_coords:
    row, col = real_to_grid(rx, ry)
    restore_x, restore_y = grid_to_real(row, col)
    print(f"({rx:.1f}, {ry:.1f})m     | [{row:2d}, {col:2d}]       | ({restore_x:.2f}, {restore_y:.2f})m")

# ── 3. 동적 지도 확장 시뮬레이션 ─────────────────────────────
print("\n" + "=" * 50)
print("  3. 동적 지도 확장 시뮬레이션")
print("=" * 50)

class SimpleExpandableGrid:
    """실제 CompoundExpandablePixelGrid의 단순화 버전"""

    def __init__(self, resolution=5):
        self.resolution = resolution
        self.grid = np.zeros((10, 10), dtype=bool)
        self.offset = np.array([5, 5])  # 원점 오프셋 (중앙 시작)

    def get_size(self):
        return self.grid.shape

    def add_wall(self, real_x, real_y):
        """실제 좌표에 벽을 추가 (필요하면 확장)"""
        col = int(real_x * self.resolution) + self.offset[1]
        row = int(real_y * self.resolution) + self.offset[0]

        # 범위를 벗어나면 확장
        if row < 0 or row >= self.grid.shape[0] or \
           col < 0 or col >= self.grid.shape[1]:
            self._expand(row, col)
            col = int(real_x * self.resolution) + self.offset[1]
            row = int(real_y * self.resolution) + self.offset[0]

        self.grid[row, col] = True

    def _expand(self, target_row, target_col):
        """지도를 필요한 방향으로 확장"""
        pad = 5  # 한 번에 5칸씩 확장
        pad_top = pad_bottom = pad_left = pad_right = 0

        if target_row < 0:
            pad_top = abs(target_row) + pad
        if target_row >= self.grid.shape[0]:
            pad_bottom = target_row - self.grid.shape[0] + 1 + pad
        if target_col < 0:
            pad_left = abs(target_col) + pad
        if target_col >= self.grid.shape[1]:
            pad_right = target_col - self.grid.shape[1] + 1 + pad

        self.grid = np.pad(self.grid, ((pad_top, pad_bottom), (pad_left, pad_right)))
        self.offset += np.array([pad_top, pad_left])
        print(f"  → 지도 확장: {self.get_size()} (오프셋: {self.offset})")

# 동적 확장 테스트
dyn_grid = SimpleExpandableGrid(resolution=5)
print(f"초기 크기: {dyn_grid.get_size()}")

robot_path = [(0,0), (0.5,0), (1.0,0), (1.5,0), (2.0,0),  # 오른쪽으로
              (2.0,0.5), (2.0,1.0), (2.0,1.5)]              # 위로

for rx, ry in robot_path:
    dyn_grid.add_wall(rx + 0.05, ry + 0.05)  # 약간 옆에 벽 추가

print(f"최종 크기: {dyn_grid.get_size()}")

# ── 4. 시각화 ─────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
fig.suptitle("격자 지도 (Grid Map) 이해하기", fontsize=14, fontweight='bold')

# 왼쪽: 기본 격자
cmap = mcolors.ListedColormap(['white', 'black'])
axes[0].imshow(grid, cmap=cmap, interpolation='nearest')
axes[0].set_title("기본 격자 지도\n(흰색=길, 검정=벽)")
axes[0].set_xlabel("열(x)")
axes[0].set_ylabel("행(y)")

# 격자선 추가
for i in range(grid.shape[0] + 1):
    axes[0].axhline(i - 0.5, color='gray', linewidth=0.5)
for j in range(grid.shape[1] + 1):
    axes[0].axvline(j - 0.5, color='gray', linewidth=0.5)

# 오른쪽: 동적 확장 지도
axes[1].imshow(dyn_grid.grid, cmap=cmap, interpolation='nearest')
axes[1].set_title(f"동적 확장 지도\n크기: {dyn_grid.get_size()}")
axes[1].set_xlabel("열")
axes[1].set_ylabel("행")

plt.tight_layout()
plt.show()

print("\n[도전 과제]")
print("  1. RESOLUTION을 10으로 바꾸면 어떻게 달라지나요?")
print("  2. 원형 장애물을 격자에 표현하려면 어떻게 해야 할까요?")
print("  3. 실제 CompoundExpandablePixelGrid 코드를 열어 비교해보세요:")
print("     src/data_structures/compound_pixel_grid.py")
