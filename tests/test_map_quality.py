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

# 1차 캘리브레이션 후 관측치 -10%p 로 바닥 설정
# clean run 실측: recall=1.000, precision=0.954
RECALL_FLOOR = 0.90
PRECISION_FLOOR = 0.85


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


def test_recall_detects_missing_walls():
    """지표 자기검증: 맵에서 내부 세로벽 영역(x=0.24, y 0~0.24)을 지우면
    recall이 바닥(RECALL_FLOOR) 아래로 떨어져야 한다 — 벽 누락 감지 능력 증명.
    (역사적 사고: 정제 단계가 희소 확정 벽을 삭제 → 이 지표가 잡아야 하는 모드)"""
    # 영역: 세로벽 전체 + 인접 가로벽 일부 (x 0.22~0.30) — 바닥 대비 여유 마진 확보
    # (세로벽만 지우면 recall 0.899로 바닥 0.90에 0.001 차 → 플레이크 위험)
    m = run_quality(demo_maze(), seed=11,
                    blank_region=((0.22, -0.02), (0.30, 0.26)))
    print(f"[blank] recall={m['recall']:.3f} (기대: < {RECALL_FLOOR})")
    assert m["recall"] < RECALL_FLOOR - 0.02, m


if __name__ == "__main__":
    test_clean_run_quality()
    test_drift_run_quality()
    test_recall_detects_missing_walls()
    print("ALL PASS")
