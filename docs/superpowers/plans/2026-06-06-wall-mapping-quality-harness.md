# Wall Mapping Quality Harness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an offline map-quality harness (realistic virtual lidar + scripted maze traversal + precision/recall metrics) so wall-mapping changes can be validated numerically WITHOUT a Webots run, then lock in current behavior with regression tests and data-tune the mapping knobs.

**Architecture:** A `tests/harness/` package provides (1) a vectorized ray-cast virtual lidar against wall segments, (2) synthetic maze definitions + a step-by-step traversal generator, (3) a noise model matching Erebus v26 (GPS σ2.5mm, heading σ1°, optional cumulative drift). Tests feed `WallMapper` directly (no Webots) and score the resulting `walls` layer against rasterized ground truth.

**Tech Stack:** Python 3.14, numpy, opencv-python (`cv2`), existing `src/` code (`CompoundExpandablePixelGrid`, `WallMapper`). No pytest — tests are plain scripts run with `python3`, matching the existing `tests/test_*.py` pattern (asserts + final `print("ALL PASS")`).

**Project context (READ FIRST):**
- This is a RoboCup Erebus 2026 rescue-robot codebase. `src/mapping/wall_mapper.py` turns lidar point clouds into a pixel wall map (`pixel grid resolution = 10px / 0.06m` → 1px = 6mm; tile = 0.12m = 20px).
- `WallMapper` current design: lidar endpoints accumulate evidence in `detected_points` (uint16); cells crossing `to_boolean_threshold` (currently 4) set `walls_raw`; `thin_walls()` recomputes display layer `walls` from `walls_raw` each frame inside a window around the robot (pure function, NO feedback) using H/V closing + small-fragment removal. Free-space clearing is DISABLED (`free_space_decrement = 0`) on purpose — pose drift made it erase good walls. Do NOT re-enable it.
- HISTORY WARNING: previous "clever" refinements (median-line projection, skeleton thinning, dilate-to-thickness) all passed dense synthetic tests but destroyed real maps (sparse lidar evidence). That is WHY this plan builds a realistic harness. Do not add geometric refinement steps as part of this plan.
- The webots `controller` module does not exist outside Webots. Every test must install the stub (Task 1 centralizes it).
- Robot facts: lidar 512 rays / 360°, single layer, range 0.036–0.48m, one scan per 32ms step; robot moves ~4mm per step at full speed.

**Working directory:** `/Users/pysw/Downloads/Guide` (this folder; `src/` and `tests/` live here).

---

### Task 0: Initialize git repository

The folder is not a git repo; later tasks need commits.

**Files:**
- Create: `.gitignore`

- [ ] **Step 1: Init repo and write .gitignore**

```bash
cd /Users/pysw/Downloads/Guide
git init
```

Create `.gitignore` with exactly:

```gitignore
__pycache__/
*.pyc
node_modules/
.DS_Store
```

- [ ] **Step 2: Verify clean status listing**

Run: `git status --short | head -30`
Expected: source files listed as untracked; NO `node_modules/` or `__pycache__/` entries.

- [ ] **Step 3: Initial commit**

```bash
git add .gitignore src tests docs Step1_webots-기초 Step2_OOP개념 Step3_FSD_Basic Step4_FSD_Final 코드분석 robot_jsons skills-lock.json
git commit -m "chore: initial commit of Erebus robot codebase"
```

Run: `git log --oneline`
Expected: 1 commit.

---

### Task 1: Harness package — webots stub + virtual lidar

**Files:**
- Create: `tests/harness/__init__.py` (empty)
- Create: `tests/harness/webots_stub.py`
- Create: `tests/harness/virtual_lidar.py`
- Test: `tests/test_virtual_lidar.py`

- [ ] **Step 1: Write `tests/harness/webots_stub.py`**

This centralizes the stub currently copy-pasted at the top of every test file. Existing tests are NOT modified (surgical scope).

```python
"""Webots 'controller' 모듈 스텁 — Webots 밖에서 src/ 코드를 import하기 위함.

사용법 (모든 하니스 테스트 첫 줄):
    from harness.webots_stub import install_stub
    install_stub()        # 이후 src/ import 가능
"""
import os
import sys
import types


def install_stub():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

    if "controller" in sys.modules:
        return

    fake = types.ModuleType("controller")

    class _Any:
        def __init__(self, *a, **k):
            pass

        def __getattr__(self, n):
            return _Any()

    fake.Robot = _Any
    fake.__getattr__ = lambda n: _Any
    sys.modules["controller"] = fake
```

- [ ] **Step 2: Write the failing test for ray casting**

Create `tests/test_virtual_lidar.py`:

```python
"""가상 라이다(레이-세그먼트 교차) 단위 테스트."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from harness.webots_stub import install_stub
install_stub()

import numpy as np
from harness.virtual_lidar import cast_rays


def test_single_wall_straight_ahead():
    # 로봇 (0,0), 벽: y=0.10 에 가로 세그먼트. 위쪽(+y) 레이는 0.10m에서 끊겨야 함.
    walls = [((-1.0, 0.10), (1.0, 0.10))]
    in_pts, out_pts = cast_rays(walls, np.array([0.0, 0.0]), heading=0.0,
                                n_rays=8, max_d=0.48, min_d=0.036)
    # 8방향 중 +y 정방향 레이 1개: 거리 0.10
    dists = [np.hypot(p[0], p[1]) for p in in_pts]
    assert any(abs(d - 0.10) < 1e-6 for d in dists), dists
    # 아래쪽(-y) 레이는 벽 없음 → out_of_bounds (max_d)
    assert any(abs(p[1] + 0.48) < 1e-6 for p in out_pts), out_pts


def test_min_distance_filtered():
    # 너무 가까운 벽(0.02m < min_d)은 in에도 out에도 없어야 함
    walls = [((-1.0, 0.02), (1.0, 0.02))]
    in_pts, out_pts = cast_rays(walls, np.array([0.0, 0.0]), heading=0.0,
                                n_rays=4, max_d=0.48, min_d=0.036)
    dists = [np.hypot(p[0], p[1]) for p in in_pts]
    assert not any(d < 0.036 for d in dists), dists


def test_nearest_of_two_walls():
    walls = [((-1.0, 0.10), (1.0, 0.10)), ((-1.0, 0.20), (1.0, 0.20))]
    in_pts, _ = cast_rays(walls, np.array([0.0, 0.0]), heading=0.0,
                          n_rays=8, max_d=0.48, min_d=0.036)
    dists = sorted(np.hypot(p[0], p[1]) for p in in_pts)
    assert abs(dists[0] - 0.10) < 1e-6  # 가까운 벽이 이김


def test_heading_rotates_rays():
    # heading π/2 회전 시 레이 패턴이 회전해도 같은 벽을 같은 거리에서 맞음
    walls = [((-1.0, 0.10), (1.0, 0.10))]
    in_a, _ = cast_rays(walls, np.array([0.0, 0.0]), heading=0.0, n_rays=64)
    in_b, _ = cast_rays(walls, np.array([0.0, 0.0]), heading=np.pi / 2, n_rays=64)
    # 월드 좌표 endpoint 집합은 동일해야 함 (포인트는 로봇 상대 좌표지만 로봇이 (0,0)이므로 동일)
    da = sorted(round(np.hypot(p[0], p[1]), 6) for p in in_a)
    db = sorted(round(np.hypot(p[0], p[1]), 6) for p in in_b)
    assert da == db


if __name__ == "__main__":
    test_single_wall_straight_ahead()
    test_min_distance_filtered()
    test_nearest_of_two_walls()
    test_heading_rotates_rays()
    print("ALL PASS")
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cd /Users/pysw/Downloads/Guide && python3 tests/test_virtual_lidar.py`
Expected: `ModuleNotFoundError: No module named 'harness.virtual_lidar'` (or ImportError).

- [ ] **Step 4: Implement `tests/harness/virtual_lidar.py`**

```python
"""가상 라이다: 벽 세그먼트 목록에 대한 512레이 레이캐스트 (벡터화).

좌표 규약: 월드 (x, y) 평면. heading 0 = +y 방향, 레이 각도는 heading 기준 시계 전체(2π).
반환 포인트는 '로봇 상대 좌표' (WallMapper.load_point_cloud가 받는 형식).
"""
import numpy as np


def cast_rays(walls, robot_pos, heading=0.0, n_rays=512, max_d=0.48, min_d=0.036):
    """walls: [((x1,y1),(x2,y2)), ...] 세그먼트 목록.
    반환: (in_bounds_points, out_of_bounds_points) — 둘 다 로봇 상대 [x,y] 목록.
    in: min_d < 거리 < max_d 에서 벽에 맞은 끝점.
    out: max_d 안에 벽이 없는 레이의 max_d 지점 (열린 공간 빔)."""
    angles = heading + np.linspace(0.0, 2.0 * np.pi, n_rays, endpoint=False)
    # 레이 방향 (heading 0 = +y): dir = (sin a, cos a)
    dx = np.sin(angles)
    dy = np.cos(angles)

    seg = np.array(walls, dtype=float)            # (S, 2, 2)
    p = seg[:, 0, :]                              # 세그먼트 시작 (S,2)
    s = seg[:, 1, :] - seg[:, 0, :]               # 세그먼트 벡터 (S,2)

    o = np.asarray(robot_pos, dtype=float)

    # 레이 o + t*d, 세그 p + u*s 교차: t = cross(p-o, s)/cross(d, s), u = cross(p-o, d)/cross(d, s)
    # 브로드캐스트: (R, S)
    d = np.stack([dx, dy], axis=1)                # (R,2)
    po = p[None, :, :] - o[None, None, :]         # (1,S,2) - 사실 (R,S,2)로 브로드캐스트
    cross_ds = d[:, None, 0] * s[None, :, 1] - d[:, None, 1] * s[None, :, 0]   # (R,S)
    cross_pos = po[..., 0] * s[None, :, 1] - po[..., 1] * s[None, :, 0]        # (R,S)
    cross_pod = po[..., 0] * d[:, None, 1] - po[..., 1] * d[:, None, 0]        # (R,S)

    with np.errstate(divide="ignore", invalid="ignore"):
        t = cross_pos / cross_ds
        u = cross_pod / cross_ds

    valid = (np.abs(cross_ds) > 1e-12) & (t > 1e-9) & (u >= 0.0) & (u <= 1.0)
    t = np.where(valid, t, np.inf)
    t_min = t.min(axis=1)                         # (R,) 각 레이의 최근접 거리

    in_pts, out_pts = [], []
    for i in range(n_rays):
        if t_min[i] >= max_d or not np.isfinite(t_min[i]):
            out_pts.append([dx[i] * max_d, dy[i] * max_d])
        elif t_min[i] > min_d:
            in_pts.append([dx[i] * t_min[i], dy[i] * t_min[i]])
        # min_d 이하는 버림 (실기 동작과 동일)
    return in_pts, out_pts
```

NOTE: `po` is built with `[None, None, :]` on `o` and `[None, :, :]` on `p` — confirm the broadcast shapes produce `(R, S, 2)` or simplify by tiling. If shapes fight, use `po = p[None, :, :] - o[None, None, :]` then rely on numpy broadcasting `(1,S,2)` against `(R,S)` index ops — the `[..., 0]` slicing keeps it `(1,S)` and broadcasts fine against `(R,S)`.

- [ ] **Step 5: Run test to verify it passes**

Run: `python3 tests/test_virtual_lidar.py`
Expected: `ALL PASS`

- [ ] **Step 6: Run the existing suite to confirm nothing broke**

Run: `for f in tests/test_*.py; do python3 "$f" >/dev/null 2>&1 && echo "$f OK" || echo "$f FAIL"; done`
Expected: every line `OK`.

- [ ] **Step 7: Commit**

```bash
git add tests/harness tests/test_virtual_lidar.py
git commit -m "feat(tests): virtual lidar harness (vectorized ray-cast) + webots stub module"
```

---

### Task 2: Maze definitions + traversal generator

**Files:**
- Create: `tests/harness/mazes.py`
- Test: `tests/test_mazes.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_mazes.py`:

```python
"""미로 정의 + 주행 경로 생성기 테스트."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from harness.webots_stub import install_stub
install_stub()

import numpy as np
from harness.mazes import demo_maze, traversal_path

TILE = 0.12


def test_demo_maze_shapes():
    maze = demo_maze()
    assert len(maze["walls"]) > 10            # 세그먼트 여러 개
    assert len(maze["free_cells"]) >= 12      # 자유 타일 여러 개
    # 곡선(폴리라인) 포함 확인: 길이 0.04m 미만의 짧은 세그먼트 존재
    short = [w for w in maze["walls"]
             if np.hypot(w[1][0] - w[0][0], w[1][1] - w[0][1]) < 0.04]
    assert len(short) >= 4, "곡선 폴리라인 없음"


def test_traversal_visits_all_cells_with_small_steps():
    maze = demo_maze()
    path = traversal_path(maze, step=0.004)
    pts = np.array([p[0] for p in path])
    headings = np.array([p[1] for p in path])
    # 스텝 크기 ≤ 4mm
    deltas = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert deltas.max() < 0.0045, deltas.max()
    # 모든 자유 셀 중심 4cm 이내 통과
    for cell in maze["free_cells"]:
        c = np.array(cell, dtype=float)
        assert np.min(np.linalg.norm(pts - c, axis=1)) < 0.04, cell
    # heading은 이동 방향과 일치 (스텝 벡터와 각도 차 < 0.01rad, 정지 스텝 제외)
    for i in range(1, len(path)):
        d = pts[i] - pts[i - 1]
        n = np.linalg.norm(d)
        if n > 1e-9:
            expect = np.arctan2(d[0], d[1])   # heading 0=+y 규약
            diff = (headings[i] - expect + np.pi) % (2 * np.pi) - np.pi
            assert abs(diff) < 0.01


if __name__ == "__main__":
    test_demo_maze_shapes()
    test_traversal_visits_all_cells_with_small_steps()
    print("ALL PASS")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 tests/test_mazes.py`
Expected: ImportError (`harness.mazes`).

- [ ] **Step 3: Implement `tests/harness/mazes.py`**

```python
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
      - 내부 가로벽: y=2*TILE, x∈[2*TILE, 4*TILE], 단 x∈[2.5T, 3T] 구간 60mm 열림
      - 곡선: 중심 (4T, 3T), 반지름 0.5T 사분원 (위-오른쪽 방 모서리)
    """
    W, H = 5 * TILE, 4 * TILE
    walls = [
        ((0, 0), (W, 0)), ((W, 0), (W, H)), ((W, H), (0, H)), ((0, H), (0, 0)),
        ((2 * TILE, 0), (2 * TILE, 2 * TILE)),
        ((2 * TILE, 2 * TILE), (2.5 * TILE, 2 * TILE)),
        ((3 * TILE, 2 * TILE), (4 * TILE, 2 * TILE)),
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
    벽을 통과하는 인접 이동은 금지. heading 0 = +y."""
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

    # 셀 중심 시퀀스를 step 간격으로 보간
    path = []
    for i in range(len(order) - 1):
        a = np.array(order[i], dtype=float)
        b = np.array(order[i + 1], dtype=float)
        d = b - a
        dist = np.linalg.norm(d)
        if dist < 1e-9:
            continue
        heading = float(np.arctan2(d[0], d[1]))
        n = max(int(dist / step), 1)
        for k in range(n):
            path.append((a + d * (k / n), heading))
    path.append((np.array(order[-1], dtype=float), path[-1][1] if path else 0.0))
    return path
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 tests/test_mazes.py`
Expected: `ALL PASS`. If DFS leaves cells unvisited because walls block adjacency, the test fails on the 4cm-coverage assert — check `_crosses_wall` and the maze layout (every free cell must be reachable).

- [ ] **Step 5: Commit**

```bash
git add tests/harness/mazes.py tests/test_mazes.py
git commit -m "feat(tests): synthetic maze definitions + DFS traversal generator"
```

---

### Task 3: Wall-mapping regression tests on realistic feeds

**Files:**
- Create: `tests/test_wall_mapping.py`

- [ ] **Step 1: Write the test file (these should PASS against current code — they lock in behavior)**

Create `tests/test_wall_mapping.py`:

```python
"""WallMapper 회귀 테스트 — 현실적 라이다 기하(희소·이동 관측) 기반.

역사적 교훈: 빽빽한 합성 피드로 검증한 기하 가공이 실데이터(희소)에서 벽을 지웠다.
여기 테스트는 반드시 cast_rays(각도 분산 레이) 기반으로만 벽 증거를 공급한다.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from harness.webots_stub import install_stub
install_stub()

import numpy as np
from harness.virtual_lidar import cast_rays
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from mapping.wall_mapper import WallMapper

RES = 10 / 0.06   # px per meter


def make_mapper():
    grid = CompoundExpandablePixelGrid(initial_shape=np.array([200, 200]),
                                       pixel_per_m=RES, robot_radius_m=0.035)
    return grid, WallMapper(grid, robot_diameter=0.07)


def run_path(grid, wm, walls, path, seed=11, drift_heading_deg=0.0, drift_pos=0.0):
    """경로를 따라 매 스텝 가상 라이다 1스캔 공급. 노이즈: GPS σ2.5mm, heading σ1°.
    drift_*: 경로 진행에 비례해 0→최대로 커지는 누적 오차."""
    rng = np.random.default_rng(seed)
    n = len(path)
    for i, (pos, heading) in enumerate(path):
        k = i / max(n - 1, 1)
        gps_noise = rng.normal(0, 0.0025, 2)
        head_err = rng.normal(0, np.deg2rad(1.0)) + np.deg2rad(drift_heading_deg) * k
        pos_drift = np.array([drift_pos, -drift_pos * 0.75]) * k
        in_pts, out_pts = cast_rays(walls, pos, heading)
        # 로봇이 '믿는' 포즈 = 진짜 포즈 + 오차. 상대 포인트는 heading 오차만큼 회전돼 보임.
        ca, sa = np.cos(head_err), np.sin(head_err)
        R = np.array([[ca, -sa], [sa, ca]])
        in_meas = [R @ np.array(p) for p in in_pts]
        out_meas = [R @ np.array(p) for p in out_pts] or [[0, 0]]
        believed = np.asarray(pos, dtype=float) + gps_noise + pos_drift
        wm.load_point_cloud(in_meas, out_meas, believed)


def coverage_of_segment(grid, p1, p2, tol_px=2):
    """세그먼트 p1→p2 를 따라 6mm 간격 샘플 중, tol_px 내에 walls 픽셀이 있는 비율."""
    walls = grid.arrays["walls"]
    p1 = np.array(p1, dtype=float); p2 = np.array(p2, dtype=float)
    n = max(int(np.linalg.norm(p2 - p1) / 0.006), 1)
    hit = 0
    for k in range(n + 1):
        pt = p1 + (p2 - p1) * (k / n)
        ai = grid.coordinates_to_array_index(pt)
        r0, r1 = max(ai[0] - tol_px, 0), ai[0] + tol_px + 1
        c0, c1 = max(ai[1] - tol_px, 0), ai[1] + tol_px + 1
        if walls[r0:r1, c0:c1].any():
            hit += 1
    return hit / (n + 1)


def straight_corridor():
    """복도: 로봇이 벽과 평행하게 왕복. 벽 y=0.12 / y=-0.12."""
    walls = [((-0.4, 0.12), (0.4, 0.12)), ((-0.4, -0.12), (0.4, -0.12))]
    path = [(np.array([-0.3 + i * 0.004, 0.0]), np.pi / 2) for i in range(150)]
    return walls, path


def test_corridor_walls_form_and_are_thin():
    grid, wm = make_mapper()
    walls_def, path = straight_corridor()
    run_path(grid, wm, walls_def, path)
    cov = coverage_of_segment(grid, (-0.25, 0.12), (0.25, 0.12))
    assert cov > 0.9, f"커버리지 {cov:.2f}"
    w = grid.arrays["walls"]
    cols = np.where(w.any(axis=0))[0]
    th = [w[:, c].sum() for c in cols]
    assert np.mean(th) <= 4.0, f"평균 두께 {np.mean(th):.2f}px"


def test_no_late_run_degradation():
    """같은 복도를 3배 오래 돌아도 벽 픽셀 수가 안정 (누적 변형 없음)."""
    grid1, wm1 = make_mapper()
    walls_def, path = straight_corridor()
    run_path(grid1, wm1, walls_def, path)
    n1 = int(grid1.arrays["walls"].sum())

    grid3, wm3 = make_mapper()
    run_path(grid3, wm3, walls_def, path * 3)
    n3 = int(grid3.arrays["walls"].sum())
    assert abs(n3 - n1) <= max(0.25 * n1, 20), (n1, n3)


def test_walls_survive_pose_drift():
    """heading 4° + 10mm 위치 드리프트가 누적돼도 기존 벽이 사라지지 않는다."""
    grid, wm = make_mapper()
    walls_def, path = straight_corridor()
    run_path(grid, wm, walls_def, path)
    cov_before = coverage_of_segment(grid, (-0.25, 0.12), (0.25, 0.12))
    run_path(grid, wm, walls_def, path, seed=7, drift_heading_deg=4.0, drift_pos=0.010)
    cov_after = coverage_of_segment(grid, (-0.25, 0.12), (0.25, 0.12))
    assert cov_after >= cov_before - 0.05, (cov_before, cov_after)


def test_opening_preserved():
    """60mm 개구부가 닫히지 않는다."""
    walls_def = [((-0.4, 0.12), (-0.03, 0.12)), ((0.03, 0.12), (0.4, 0.12))]
    path = [(np.array([-0.3 + i * 0.004, 0.0]), np.pi / 2) for i in range(150)]
    grid, wm = make_mapper()
    run_path(grid, wm, walls_def, path)
    w = grid.arrays["walls"]
    gi_l = grid.coordinates_to_array_index(np.array([-0.02, 0.12]))
    gi_r = grid.coordinates_to_array_index(np.array([0.02, 0.12]))
    band = w[gi_l[0] - 4:gi_l[0] + 5, min(gi_l[1], gi_r[1]):max(gi_l[1], gi_r[1]) + 1]
    open_cols = (~band.any(axis=0)).sum()
    assert open_cols >= 4, f"개구부 잔여 폭 {open_cols}px"


def test_underflow_guard():
    """detected_points가 언더플로(0-1=65535)로 오염되지 않는다."""
    grid, wm = make_mapper()
    walls_def, path = straight_corridor()
    run_path(grid, wm, walls_def, path)
    assert grid.arrays["detected_points"].max() < 60000


if __name__ == "__main__":
    test_corridor_walls_form_and_are_thin()
    test_no_late_run_degradation()
    test_walls_survive_pose_drift()
    test_opening_preserved()
    test_underflow_guard()
    print("ALL PASS")
```

- [ ] **Step 2: Run the test**

Run: `python3 tests/test_wall_mapping.py`
Expected: `ALL PASS` against current `src/mapping/wall_mapper.py`. These are regression locks, not new features — if one fails, STOP and report which assert with values (do not "fix" the production code to pass without understanding why; the current code passed equivalent inline checks this session).

NOTE: `free_space_decrement` is 0 in current code, so the drift test passing depends on clearing staying off — if someone re-enabled it, this test catches the wall-erasure regression. That is intentional.

- [ ] **Step 3: Run full suite**

Run: `for f in tests/test_*.py; do python3 "$f" >/dev/null 2>&1 && echo "$f OK" || echo "$f FAIL"; done`
Expected: all `OK`.

- [ ] **Step 4: Commit**

```bash
git add tests/test_wall_mapping.py
git commit -m "test: realistic-geometry regression suite for wall mapping"
```

---

### Task 4: Map quality metric (precision/recall vs ground truth)

**Files:**
- Create: `tests/harness/quality.py`
- Test: `tests/test_map_quality.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_map_quality.py`:

```python
"""전체 미로 주행 → 맵 품질(정밀도/재현율) 측정.

recall  = (도달 가능 진실 벽 중) 맵이 ±2px 내로 찍은 비율 — 벽 누락 감지
precision = (맵이 찍은 벽 중) 진실 벽 ±2px 내인 비율 — 유령 벽/번짐 감지
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from harness.webots_stub import install_stub
install_stub()

import numpy as np
from harness.mazes import demo_maze
from harness.quality import run_quality

# 1차 캘리브레이션 후 관측치 -10%p 로 바닥 설정 (Step 4에서 숫자 갱신)
RECALL_FLOOR = 0.0
PRECISION_FLOOR = 0.0


def test_clean_run_quality():
    m = run_quality(demo_maze(), seed=11)
    print(f"[clean] recall={m['recall']:.3f} precision={m['precision']:.3f}")
    assert m["recall"] >= RECALL_FLOOR, m
    assert m["precision"] >= PRECISION_FLOOR, m


def test_drift_run_quality():
    m = run_quality(demo_maze(), seed=7, drift_heading_deg=4.0, drift_pos=0.010)
    print(f"[drift] recall={m['recall']:.3f} precision={m['precision']:.3f}")
    # 드리프트 시에도 붕괴(절반 이하)는 없어야 함
    assert m["recall"] >= RECALL_FLOOR * 0.7, m
    assert m["precision"] >= PRECISION_FLOOR * 0.7, m


if __name__ == "__main__":
    test_clean_run_quality()
    test_drift_run_quality()
    print("ALL PASS")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 tests/test_map_quality.py`
Expected: ImportError (`harness.quality`).

- [ ] **Step 3: Implement `tests/harness/quality.py`**

```python
"""미로 전체 주행 후 walls 레이어를 진실 벽과 비교해 precision/recall 산출."""
import numpy as np
import cv2 as cv

from harness.virtual_lidar import cast_rays
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from mapping.wall_mapper import WallMapper

RES = 10 / 0.06
TOL_PX = 2
LIDAR_REACH = 0.45   # 이 거리 내에서 관측 가능했던 진실 벽만 recall 분모에 포함


def run_quality(maze, seed=11, drift_heading_deg=0.0, drift_pos=0.0, knobs=None):
    """maze를 주행·맵핑하고 품질 지표 dict 반환.
    knobs: {"to_boolean_threshold": int, "wall_close_kernel_px": int,
            "min_wall_fragment_px": int} 선택적 오버라이드 (튜닝용)."""
    from harness.mazes import traversal_path

    grid = CompoundExpandablePixelGrid(initial_shape=np.array([300, 300]),
                                       pixel_per_m=RES, robot_radius_m=0.035)
    wm = WallMapper(grid, robot_diameter=0.07)
    if knobs:
        for k, v in knobs.items():
            assert hasattr(wm, k), k
            setattr(wm, k, v)
            if k == "wall_close_kernel_px":   # 커널 재생성
                wm._WallMapper__close_kernel_h = np.ones((1, v), np.uint8)
                wm._WallMapper__close_kernel_v = np.ones((v, 1), np.uint8)

    rng = np.random.default_rng(seed)
    path = traversal_path(maze)
    n = len(path)
    for i, (pos, heading) in enumerate(path):
        k = i / max(n - 1, 1)
        gps_noise = rng.normal(0, 0.0025, 2)
        head_err = rng.normal(0, np.deg2rad(1.0)) + np.deg2rad(drift_heading_deg) * k
        pos_drift = np.array([drift_pos, -drift_pos * 0.75]) * k
        in_pts, out_pts = cast_rays(maze["walls"], pos, heading)
        ca, sa = np.cos(head_err), np.sin(head_err)
        R = np.array([[ca, -sa], [sa, ca]])
        in_meas = [R @ np.array(p) for p in in_pts]
        out_meas = [R @ np.array(p) for p in out_pts] or [[0, 0]]
        believed = np.asarray(pos, dtype=float) + gps_noise + pos_drift
        wm.load_point_cloud(in_meas, out_meas, believed)

    mapped = grid.arrays["walls"].astype(np.uint8)

    # 진실 벽 래스터화 (같은 그리드 인덱스 좌표계)
    true_mask = np.zeros_like(mapped)
    for (p1, p2) in maze["walls"]:
        a = grid.coordinates_to_array_index(np.array(p1, dtype=float))
        b = grid.coordinates_to_array_index(np.array(p2, dtype=float))
        cv.line(true_mask, (int(a[1]), int(a[0])), (int(b[1]), int(b[0])), 1, 1)

    # 도달 가능 영역: 주행 지점들에서 LIDAR_REACH 이내
    reach = np.zeros_like(mapped)
    rpx = int(LIDAR_REACH * RES)
    stride = max(len(path) // 200, 1)         # 200포인트 샘플이면 충분
    for (pos, _h) in path[::stride]:
        ai = grid.coordinates_to_array_index(np.asarray(pos, dtype=float))
        cv.circle(reach, (int(ai[1]), int(ai[0])), rpx, 1, -1)
    true_reach = true_mask & reach

    kern = np.ones((2 * TOL_PX + 1, 2 * TOL_PX + 1), np.uint8)
    true_dil = cv.dilate(true_reach, kern)
    mapped_dil = cv.dilate(mapped, kern)

    tp_p = int((mapped & true_dil).sum())
    precision = tp_p / max(int(mapped.sum()), 1)
    tp_r = int((true_reach & mapped_dil).sum())
    recall = tp_r / max(int(true_reach.sum()), 1)

    return {"recall": recall, "precision": precision,
            "mapped_px": int(mapped.sum()), "true_px": int(true_reach.sum())}
```

- [ ] **Step 4: Calibrate floors**

Run: `python3 tests/test_map_quality.py`
Expected: passes with floors at 0.0 and PRINTS the two metric lines, e.g. `[clean] recall=0.87 precision=0.78`.

Then edit `tests/test_map_quality.py`: set `RECALL_FLOOR` and `PRECISION_FLOOR` to the printed **clean-run** values minus 0.10 (e.g. observed 0.87 → floor 0.77). Re-run to confirm still `ALL PASS`.

Sanity expectations: clean recall should land ≥ 0.7 and precision ≥ 0.6. If clean recall < 0.5, something is wrong in the harness wiring (most likely the heading/rotation convention between `cast_rays` and the measured-point rotation, or `coordinates_to_array_index` axis order — debug by dumping `mapped` and `true_mask` to PNG with `cv.imwrite('/tmp/dbg_mapped.png', mapped*255)` and comparing visually). Report as BLOCKED if it cannot be made plausible.

- [ ] **Step 5: Run full suite**

Run: `for f in tests/test_*.py; do python3 "$f" >/dev/null 2>&1 && echo "$f OK" || echo "$f FAIL"; done`
Expected: all `OK`.

- [ ] **Step 6: Commit**

```bash
git add tests/harness/quality.py tests/test_map_quality.py
git commit -m "feat(tests): offline map quality metric (precision/recall vs ground truth)"
```

---

### Task 5: Data-driven knob tuning

**Files:**
- Create: `tests/tune_wall_knobs.py` (tool, not a test — name avoids the `test_` prefix)
- Modify: `src/mapping/wall_mapper.py` (knob values ONLY if a clear winner emerges)

- [ ] **Step 1: Write the tuning script**

Create `tests/tune_wall_knobs.py`:

```python
"""벽 맵핑 노브 그리드서치 — clean + drift 시나리오 평균 F1 비교.

실행: python3 tests/tune_wall_knobs.py
출력: 조합별 표 + 최고 F1 조합. 코드 자동 수정은 하지 않음 (사람이 판단).
"""
import os
import sys
import itertools

sys.path.insert(0, os.path.dirname(__file__))

from harness.webots_stub import install_stub
install_stub()

from harness.mazes import demo_maze
from harness.quality import run_quality


def f1(m):
    p, r = m["precision"], m["recall"]
    return 2 * p * r / max(p + r, 1e-9)


def main():
    grid_space = {
        "to_boolean_threshold": [3, 4, 5],
        "wall_close_kernel_px": [3, 5, 7],
        "min_wall_fragment_px": [2, 3, 4],
    }
    maze = demo_maze()
    rows = []
    keys = list(grid_space)
    for combo in itertools.product(*grid_space.values()):
        knobs = dict(zip(keys, combo))
        clean = run_quality(maze, seed=11, knobs=knobs)
        drift = run_quality(maze, seed=7, drift_heading_deg=4.0,
                            drift_pos=0.010, knobs=knobs)
        score = (f1(clean) + f1(drift)) / 2
        rows.append((score, knobs, clean, drift))
        print(f"{knobs} → clean R={clean['recall']:.2f}/P={clean['precision']:.2f} "
              f"drift R={drift['recall']:.2f}/P={drift['precision']:.2f} "
              f"avgF1={score:.3f}")
    rows.sort(key=lambda r: -r[0])
    best = rows[0]
    print("\n=== BEST ===")
    print(best[1], f"avgF1={best[0]:.3f}")
    cur = {"to_boolean_threshold": 4, "wall_close_kernel_px": 5,
           "min_wall_fragment_px": 3}
    cur_row = next(r for r in rows if r[1] == cur)
    print(f"현재 노브 avgF1={cur_row[0]:.3f} (순위 {rows.index(cur_row)+1}/{len(rows)})")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the grid search**

Run: `python3 tests/tune_wall_knobs.py` (27 combos × 2 runs each — may take a few minutes; if over ~10 min, reduce `demo_maze` traversal by passing a larger step in `traversal_path` default... do NOT shrink the ray count).
Expected: full table + BEST line + current-knob ranking.

- [ ] **Step 3: Apply the winner (CONDITIONAL)**

Decision rule:
- If best avgF1 beats current knobs' avgF1 by **> 0.02**, edit the three knob values in `src/mapping/wall_mapper.py` `__init__` to the winning combo (values only — keep comments, append `# 그리드서치 2026-06-06` to each changed line).
- Otherwise change nothing and record the table location in the commit message.

- [ ] **Step 4: Re-run the full suite (floors must still hold)**

Run: `for f in tests/test_*.py; do python3 "$f" >/dev/null 2>&1 && echo "$f OK" || echo "$f FAIL"; done`
Expected: all `OK`. If a knob change broke a regression floor, revert the knob change and report.

- [ ] **Step 5: Commit**

```bash
git add tests/tune_wall_knobs.py src/mapping/wall_mapper.py
git commit -m "feat(tests): wall-knob grid search tool; apply tuned knobs if winner"
```

---

## Out of scope (explicitly)

- Re-enabling free-space clearing (`free_space_decrement`) — refuted on real data this session.
- Geometric refinement (line projection / skeletonization / thickness dilation) — refuted on real data.
- Parsing real `.wbt` worlds (halfTile bitmask protos = separate project).
- Webots/sim verification — only the user can run the sim; the harness approximates it.
- Touching pose/heading code (`pose_manager.py`, `gps.py`) — separate workstream.

## Final verification

All of: `python3 tests/test_virtual_lidar.py`, `tests/test_mazes.py`, `tests/test_wall_mapping.py`, `tests/test_map_quality.py` print `ALL PASS`, plus the 10 pre-existing test files still pass.
