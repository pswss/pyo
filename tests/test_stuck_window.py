"""창(window) 기반 정체 감지 테스트.

실런 확인 데드락: 블랙홀/벽 옆에서 정지 분기 진입 → 각속도 0, 위치는 GPS 노이즈로
±2~4mm 떨림. 기존 per-step 감지는 ① 바퀴 0이면 카운터 리셋 ② 노이즈 2~4mm > 1mm
임계라 둘 다 못 잡음. 추가 규칙: 최근 N프레임 순이동 < 임계 → stuck.
"""
import os
import sys
import random

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from executor.stuck_detector import StuckDetector
from data_structures.vectors import Position2D


def feed(det, positions, wheel=0.0):
    prev = positions[0]
    for p in positions:
        det.update(p, prev, wheel)
        prev = p


def noisy_stationary(n, sigma=0.003, seed=1):
    rng = random.Random(seed)
    base = Position2D(0.5, 0.5)
    return [Position2D(base.x + rng.gauss(0, sigma), base.y + rng.gauss(0, sigma))
            for _ in range(n)]


def test_stationary_with_gps_noise_triggers():
    """정지 + GPS 노이즈 떨림 + 바퀴 0 → 창 채워지면 stuck (기존 감지는 놓침)."""
    det = StuckDetector()
    feed(det, noisy_stationary(det.window_frames + 5), wheel=0.0)
    assert det.is_stuck(), "정지 데드락 미감지"


def test_normal_driving_does_not_trigger():
    """정상 주행(스텝당 5mm 전진) → stuck 아님."""
    det = StuckDetector()
    pts = [Position2D(0.5 + 0.005 * i, 0.5) for i in range(det.window_frames + 5)]
    feed(det, pts, wheel=1.0)
    assert not det.is_stuck()


def test_window_not_full_does_not_trigger():
    """창 미충족(이력 부족) 시 미발동 — 시작 직후 오탐 방지."""
    det = StuckDetector()
    feed(det, noisy_stationary(det.window_frames // 2), wheel=0.0)
    assert not det.is_stuck()


def test_reset_clears_window():
    """reset() 후엔 창 다시 채워질 때까지 미발동 (wiggle/보고 복귀 직후 오탐 방지)."""
    det = StuckDetector()
    feed(det, noisy_stationary(det.window_frames + 5), wheel=0.0)
    assert det.is_stuck()
    det.reset()
    assert not det.is_stuck()
    feed(det, noisy_stationary(10, seed=2), wheel=0.0)
    assert not det.is_stuck()


def test_legacy_per_step_detection_intact():
    """기존 규칙 유지: 바퀴 돌고 스텝 이동 <1mm가 50스텝 초과 → stuck."""
    det = StuckDetector()
    p = Position2D(0.5, 0.5)
    for _ in range(det.stuck_threshold + 2):
        det.update(p, p, 1.0)
    assert det.is_stuck()


if __name__ == "__main__":
    test_stationary_with_gps_noise_triggers()
    test_normal_driving_does_not_trigger()
    test_window_not_full_does_not_trigger()
    test_reset_clears_window()
    test_legacy_per_step_detection_intact()
    print("OK")
