"""합성 미로 정의 + 스텝 단위 주행 경로 생성.

미로 규약:
  - 격자 셀 크기 0.12m (Erebus 타일).
  - maze dict: {"walls": [세그먼트...], "free_cells": [셀 중심 (x,y)...]}
  - 곡선 벽은 짧은 세그먼트 폴리라인으로 표현 (가상 라이다는 세그먼트만 알면 됨).
"""
import numpy as np

TILE = 0.12


def _arc(cx, cy, r, a0, a1, n=12):
    """원호 → 세그먼트 폴리라인."""
    angs = np.linspace(a0, a1, n + 1)
    pts = [(cx + r * np.cos(a), cy + r * np.sin(a)) for a in angs]
    return [(pts[i], pts[i + 1]) for i in range(n)]


def demo_maze():
    """5x4 타일 미로: 외곽 폐쇄 + 내부 직선벽/코너 + 60mm 개구부 + 사분원 곡선벽.

    레이아웃 (타일 단위, 원점 = 좌하단 외곽 모서리):
      - 외곽: (0,0)-(5*TILE, 4*TILE) 사각형
      - 내부 세로벽: x=2*TILE, y∈[0, 2*TILE]  (아래쪽 절반)
      - 내부 가로벽: y=2*TILE, x∈[2*TILE, 4*TILE], 단 x∈[2.4T, 2.9T] 구간 60mm 열림
      - 곡선: 중심 (4T, 3T), 반지름 0.5T 사분원 (위-오른쪽 방 모서리)
        - 호는 π/2→π (위→왼쪽), 끝점: (4T, 3.5T)~(3.5T, 3T)
    """
    W, H = 5 * TILE, 4 * TILE
    walls = [
        ((0, 0), (W, 0)), ((W, 0), (W, H)), ((W, H), (0, H)), ((0, H), (0, 0)),
        ((2 * TILE, 0), (2 * TILE, 2 * TILE)),
        ((2 * TILE, 2 * TILE), (2.4 * TILE, 2 * TILE)),
        ((2.9 * TILE, 2 * TILE), (4 * TILE, 2 * TILE)),
    ]
    walls += _arc(4 * TILE, 3 * TILE, 0.5 * TILE, np.pi / 2, np.pi, n=12)

    free_cells = []
    for gx in range(5):
        for gy in range(4):
            free_cells.append(((gx + 0.5) * TILE, (gy + 0.5) * TILE))
    return {"walls": walls, "free_cells": free_cells}


def _adjacent(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) < TILE * 1.01 and a != b


def _crosses_wall(a, b, walls):
    """셀 중심 a→b 직선이 벽 세그먼트와 교차하면 True (간단 교차 판정)."""
    a = np.array(a); b = np.array(b)
    d = b - a
    for (p1, p2) in walls:
        p = np.array(p1); s = np.array(p2) - np.array(p1)
        denom = d[0] * s[1] - d[1] * s[0]
        if abs(denom) < 1e-12:
            continue
        po = p - a
        t = (po[0] * s[1] - po[1] * s[0]) / denom
        u = (po[0] * d[1] - po[1] * d[0]) / denom
        if 0.0 < t < 1.0 and 0.0 <= u <= 1.0:
            return True
    return False


def traversal_path(maze, step=0.004):
    """DFS로 인접 자유 셀들을 방문하는 스텝 경로. 반환: [(pos(x,y), heading), ...].
    벽을 통과하는 인접 이동은 금지. heading 0 = +y.

    heading[i]는 pts[i] - pts[i-1] 방향과 일치 (테스트 규약).
    첫 번째 포인트는 첫 이동 방향을 heading으로 사용.
    """
    cells = maze["free_cells"]
    walls = maze["walls"]
    visited = {cells[0]}
    order = [cells[0]]
    stack = [cells[0]]
    while stack:
        cur = stack[-1]
        nxt = None
        for cand in cells:
            if cand not in visited and _adjacent(cur, cand) \
                    and not _crosses_wall(cur, cand, walls):
                nxt = cand
                break
        if nxt is None:
            stack.pop()
            if stack:
                order.append(stack[-1])     # 되돌아가기도 경로에 포함
            continue
        visited.add(nxt)
        order.append(nxt)
        stack.append(nxt)

    assert len(visited) == len(cells), \
        f"도달 불가 셀 존재: {set(cells) - visited}"

    # 셀 중심 시퀀스를 step 간격으로 보간 — 포인트만 먼저 수집
    pts = []
    for i in range(len(order) - 1):
        a = np.array(order[i], dtype=float)
        b = np.array(order[i + 1], dtype=float)
        d = b - a
        dist = np.linalg.norm(d)
        if dist < 1e-9:
            continue
        n = max(int(dist / step), 1)
        for k in range(n):
            pts.append(a + d * (k / n))
    pts.append(np.array(order[-1], dtype=float))

    # heading[i] = 도착 방향 = pts[i] - pts[i-1], heading[0] = 첫 이동 방향
    path = []
    for i, pos in enumerate(pts):
        if i == 0:
            if len(pts) > 1:
                d = pts[1] - pts[0]
                h = float(np.arctan2(d[0], d[1]))
            else:
                h = 0.0
        else:
            d = pts[i] - pts[i - 1]
            norm = np.linalg.norm(d)
            h = float(np.arctan2(d[0], d[1])) if norm > 1e-9 else path[-1][1]
        path.append((pos, h))
    return path
