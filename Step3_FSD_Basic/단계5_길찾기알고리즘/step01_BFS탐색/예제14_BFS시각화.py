"""
[예제 14] BFS 탐색 단계별 시각화
----------------------------------
터미널에서 실행: python 예제14_BFS시각화.py
필요 패키지: pip install numpy matplotlib

BFS가 격자 미로에서 어떻게 경로를 찾는지 애니메이션으로 보여줍니다.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
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

# ── 미로 정의 ────────────────────────────────────────────────
MAZE = np.array([
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,1,0,0,0,0,0,0,0,1,0,1],
    [1,0,1,0,1,0,1,1,1,1,1,0,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,1],
    [1,0,1,1,1,1,1,0,1,0,1,1,1,0,1],
    [1,0,0,0,0,0,1,0,1,0,0,0,0,0,1],
    [1,0,1,1,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,1,0,1,1,1,1,1,0,1,1,1,0,1],
    [1,0,0,0,0,0,0,0,0,0,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
], dtype=np.int32)  # 1=벽, 0=길

START = (1, 1)   # 출발점 (행, 열)
GOAL  = (9, 13)  # 목표점 (행, 열)

# ── BFS 구현 ─────────────────────────────────────────────────
def bfs(maze, start, goal):
    """
    BFS로 최단 경로 탐색.
    실제 프로젝트의 bfs.py 와 동일한 원리!

    반환: (경로, 탐색 단계 기록)
    """
    rows, cols = maze.shape
    queue   = deque([start])
    came_from = {start: None}
    steps = []  # 시각화용 탐색 기록

    while queue:
        current = queue.popleft()
        steps.append(('visit', current))

        if current == goal:
            # 경로 역추적
            path = []
            node = goal
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            steps.append(('path', path))
            return path, steps

        row, col = current
        # 상하좌우 이웃
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = row + dr, col + dc
            next_pos = (nr, nc)
            if (0 <= nr < rows and 0 <= nc < cols
                    and maze[nr, nc] == 0
                    and next_pos not in came_from):
                came_from[next_pos] = current
                queue.append(next_pos)
                steps.append(('explore', next_pos))

    return [], steps  # 경로 없음

# ── 실행 ─────────────────────────────────────────────────────
print("BFS 탐색 중...")
path, steps = bfs(MAZE, START, GOAL)

if path:
    print(f"최단 경로 발견! 길이: {len(path)} 칸")
    print(f"총 탐색 횟수: {sum(1 for s in steps if s[0]=='visit')} 개 노드")
else:
    print("경로를 찾을 수 없습니다!")

# ── 단계별 시각화 ─────────────────────────────────────────────
# 색상 코드
COLORS = {
    'wall':    [50,  50,  50 ],   # 검정 (벽)
    'path_bg': [230, 230, 230],   # 밝은 회색 (길)
    'visited': [150, 200, 255],   # 하늘색 (방문함)
    'explore': [100, 150, 255],   # 파랑 (탐색 예정)
    'path':    [50,  200, 50 ],   # 초록 (최단 경로)
    'start':   [255, 100, 100],   # 빨강 (출발)
    'goal':    [255, 200, 50 ],   # 노랑 (목표)
}

def make_image(maze, visited_set, explore_set, final_path=None):
    h, w = maze.shape
    img = np.zeros((h, w, 3), dtype=np.uint8)

    for r in range(h):
        for c in range(w):
            if maze[r, c] == 1:
                img[r, c] = COLORS['wall']
            else:
                img[r, c] = COLORS['path_bg']

    for pos in explore_set:
        img[pos[0], pos[1]] = COLORS['explore']
    for pos in visited_set:
        img[pos[0], pos[1]] = COLORS['visited']
    if final_path:
        for pos in final_path:
            img[pos[0], pos[1]] = COLORS['path']

    img[START[0], START[1]] = COLORS['start']
    img[GOAL[0],  GOAL[1]]  = COLORS['goal']
    return img

# 주요 스냅샷 뽑기 (10단계로 나눔)
visited_set = set()
explore_set = set()
snapshots   = []

for i, step in enumerate(steps):
    if step[0] == 'visit':
        visited_set.add(step[1])
        explore_set.discard(step[1])
    elif step[0] == 'explore':
        explore_set.add(step[1])
    elif step[0] == 'path':
        snapshots.append(make_image(MAZE, visited_set, explore_set, step[1]))
        continue

    if i % max(1, len(steps) // 9) == 0:
        snapshots.append(make_image(MAZE, visited_set, explore_set))

fig, axes = plt.subplots(2, 5, figsize=(16, 7))
fig.suptitle("BFS 탐색 단계별 진행", fontsize=14, fontweight='bold')

for idx, (ax, snap) in enumerate(zip(axes.flatten(), snapshots)):
    ax.imshow(snap, interpolation='nearest')
    ax.set_title(f"단계 {idx+1}" if idx < len(snapshots)-1 else "최종 경로!")
    ax.axis('off')

# 범례
legend_info = [
    ('시작점', COLORS['start']),
    ('목표점', COLORS['goal']),
    ('방문함', COLORS['visited']),
    ('탐색 예정', COLORS['explore']),
    ('최단 경로', COLORS['path']),
]
legend_ax = axes.flatten()[-1]
legend_ax.axis('off')
for i, (label, color) in enumerate(legend_info):
    legend_ax.add_patch(plt.Rectangle((0.1, 0.7 - i*0.15), 0.2, 0.1,
                                       color=[c/255 for c in color]))
    legend_ax.text(0.35, 0.75 - i*0.15, label, va='center', fontsize=9)

plt.tight_layout()
plt.show()

print("\n[도전 과제]")
print("  1. 출발점과 목표점을 바꿔보세요")
print("  2. 미로에 새로운 벽(1)을 추가하면 경로가 어떻게 바뀌나요?")
print("  3. DFS(깊이 우선)로 바꾸려면 deque 대신 뭘 써야 할까요?")
print("     힌트: deque.popleft() → deque.pop()")
