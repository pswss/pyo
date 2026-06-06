"""
[예제 15] BFS vs A* 알고리즘 비교
------------------------------------
터미널에서 실행: python 예제15_A스타비교.py
필요 패키지: pip install numpy matplotlib

BFS와 A*를 같은 미로에 실행해서 탐색 횟수와 경로를 비교합니다.
실제 프로젝트의 efficient_a_star.py 를 단순화한 구현입니다.
"""

import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from heapq import heappush, heappop
import time
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

# ── 미로 ─────────────────────────────────────────────────────
MAZE = np.array([
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,1],
    [1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,0,1,1,0,1],
    [1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0,1],
    [1,0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1],
    [1,1,1,0,1,1,1,1,0,0,1,1,1,0,1,1,1,1,0,1],
    [1,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
    [1,0,1,1,1,0,1,1,1,1,1,0,1,1,1,0,1,1,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
], dtype=np.int32)

START = (1, 1)
GOAL  = (9, 18)

# ── BFS 구현 ─────────────────────────────────────────────────
def bfs(maze, start, goal):
    queue     = deque([start])
    came_from = {start: None}
    visited   = 0

    while queue:
        cur = queue.popleft()
        visited += 1

        if cur == goal:
            path = []
            node = goal
            while node:
                path.append(node)
                node = came_from[node]
            return list(reversed(path)), visited, came_from

        r, c = cur
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nxt = (r+dr, c+dc)
            if (0 <= nxt[0] < maze.shape[0] and
                0 <= nxt[1] < maze.shape[1] and
                maze[nxt[0], nxt[1]] == 0 and
                nxt not in came_from):
                came_from[nxt] = cur
                queue.append(nxt)

    return [], visited, came_from

# ── A* 구현 ──────────────────────────────────────────────────
def heuristic(a, b):
    """실제 프로젝트와 동일한 휴리스틱"""
    dy = abs(a[0] - b[0])
    dx = abs(a[1] - b[1])
    return min(dx, dy) * 15 + abs(dx - dy) * 10

def a_star(maze, start, goal):
    counter   = 0   # 동점 처리용
    open_list = [(0, counter, start)]
    came_from = {start: None}
    g_score   = {start: 0}
    closed    = set()
    visited   = 0

    while open_list:
        _, _, cur = heappop(open_list)

        if cur in closed:
            continue
        closed.add(cur)
        visited += 1

        if cur == goal:
            path = []
            node = goal
            while node:
                path.append(node)
                node = came_from[node]
            return list(reversed(path)), visited, came_from

        r, c = cur
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nxt = (r+dr, c+dc)
            if (0 <= nxt[0] < maze.shape[0] and
                0 <= nxt[1] < maze.shape[1] and
                maze[nxt[0], nxt[1]] == 0):

                new_g = g_score[cur] + 1
                if nxt not in g_score or new_g < g_score[nxt]:
                    g_score[nxt] = new_g
                    f = new_g + heuristic(nxt, goal)
                    counter += 1
                    heappush(open_list, (f, counter, nxt))
                    came_from[nxt] = cur

    return [], visited, came_from

# ── 실행 및 비교 ─────────────────────────────────────────────
print("=" * 55)
print("  BFS vs A* 비교")
print("=" * 55)

t0 = time.perf_counter()
bfs_path, bfs_visited, bfs_came = bfs(MAZE, START, GOAL)
bfs_time = time.perf_counter() - t0

t0 = time.perf_counter()
astar_path, astar_visited, astar_came = a_star(MAZE, START, GOAL)
astar_time = time.perf_counter() - t0

print(f"\n{'':>20} {'BFS':>10} {'A*':>10}")
print("-" * 42)
print(f"{'탐색한 노드 수':>20} {bfs_visited:>10} {astar_visited:>10}")
print(f"{'경로 길이':>20} {len(bfs_path):>10} {len(astar_path):>10}")
print(f"{'실행 시간(ms)':>20} {bfs_time*1000:>10.3f} {astar_time*1000:>10.3f}")
print(f"\nA*가 BFS보다 {bfs_visited/max(astar_visited,1):.1f}배 적게 탐색!")

# ── 시각화 ───────────────────────────────────────────────────
def draw_result(ax, maze, path, visited_dict, title, color_path, color_visited):
    h, w = maze.shape
    img = np.ones((h, w, 3), dtype=np.uint8) * 220

    img[maze == 1] = [50, 50, 50]   # 검정: 벽

    for pos in visited_dict:
        if maze[pos[0], pos[1]] == 0:
            img[pos[0], pos[1]] = color_visited

    for pos in path:
        img[pos[0], pos[1]] = color_path

    img[START[0], START[1]] = [255, 50,  50 ]  # 빨강: 시작
    img[GOAL[0],  GOAL[1]]  = [255, 200, 50 ]  # 노랑: 목표

    ax.imshow(img, interpolation='nearest')
    ax.set_title(f"{title}\n탐색: {len(visited_dict)} 노드, 경로: {len(path)} 칸")
    ax.axis('off')

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
fig.suptitle("BFS vs A* 알고리즘 비교", fontsize=14, fontweight='bold')

draw_result(ax1, MAZE, bfs_path, bfs_came,
            "BFS (너비 우선 탐색)",
            [50, 200, 50], [150, 200, 255])

draw_result(ax2, MAZE, astar_path, astar_came,
            "A* (A스타)",
            [50, 200, 50], [255, 180, 100])

# 범례
for ax in [ax1, ax2]:
    ax.text(0.02, 0.02, "■ 벽  ■ 탐색함  ■ 경로  ● 시작  ● 목표",
            transform=ax.transAxes, fontsize=7,
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

plt.tight_layout()
plt.show()

print("\n[도전 과제]")
print("  1. heuristic 함수를 단순 맨해튼 거리로 바꿔보세요:")
print("     return abs(a[0]-b[0]) + abs(a[1]-b[1])")
print("  2. 더 복잡한 미로를 만들어서 차이를 비교해보세요")
print("  3. h(n)=0으로 바꾸면 A*가 BFS처럼 동작하는지 확인하세요")
