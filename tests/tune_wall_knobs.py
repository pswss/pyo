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
