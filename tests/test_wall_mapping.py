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

RES = 10 / 0.06   # px per meter  # 타일당 10px / 반타일 0.06m = 166.7px/m (1px=6mm) — src Mapper와 동일


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
        r0, r1 = max(int(ai[0]) - tol_px, 0), int(ai[0]) + tol_px + 1
        c0, c1 = max(int(ai[1]) - tol_px, 0), int(ai[1]) + tol_px + 1
        if walls[r0:r1, c0:c1].any():
            hit += 1
    return hit / (n + 1)


def straight_corridor():
    """복도: 로봇이 벽과 평행하게 주행. 벽 y=0.12 / y=-0.12."""
    walls = [((-0.4, 0.12), (0.4, 0.12)), ((-0.4, -0.12), (0.4, -0.12))]
    path = [(np.array([-0.3 + i * 0.004, 0.0]), np.pi / 2) for i in range(150)]
    return walls, path


def test_corridor_walls_form_and_are_thin():
    grid, wm = make_mapper()
    walls_def, path = straight_corridor()
    run_path(grid, wm, walls_def, path)
    cov = coverage_of_segment(grid, (-0.25, 0.12), (0.25, 0.12))
    assert cov > 0.9, f"커버리지 {cov:.2f}"  # 실측 1.00 (seed 11)
    w = grid.arrays["walls"]
    gi = grid.coordinates_to_array_index(np.array([0.0, 0.12]))
    band = w[int(gi[0]) - 5:int(gi[0]) + 6, :]   # 상단 벽 주변 ±5px (벽 간격 40px → 단일 벽만)
    cols = np.where(band.any(axis=0))[0]
    th = [band[:, c].sum() for c in cols]
    assert np.mean(th) <= 4.0, f"평균 두께 {np.mean(th):.2f}px"  # 실측 3.03px (seed 11), 노이즈 envelope 상한
    cov_bot = coverage_of_segment(grid, (-0.25, -0.12), (0.25, -0.12))
    assert cov_bot > 0.9, f"하단 벽 커버리지 {cov_bot:.2f}"


def test_no_late_run_degradation():
    """반복 주행 시 벽 픽셀 증가가 '감속(포화)'해야 한다.

    역사적 버그: 정제 출력이 다음 프레임 입력으로 피드백되며 증가가 가속(복리) →
    후반부 맵 붕괴. 잡을 불변량 = 가속 성장. 노이즈 envelope까지의 유한 포화
    (실측: 818→995→1065→1088, 증분 177→70→23)는 정상이다.
    """
    walls_def, path = straight_corridor()
    counts = []
    grid3 = None
    for mult in (1, 2, 3):
        grid, wm = make_mapper()
        run_path(grid, wm, walls_def, path * mult)
        counts.append(int(grid.arrays["walls"].sum()))
        if mult == 3:
            grid3 = grid
    n1, n2, n3 = counts
    # 1) 증분 감속 (가속 = 복리 피드백 회귀)
    assert (n3 - n2) <= 0.7 * (n2 - n1) + 10, counts  # 실측 증분 177→70→23 (감속); 0.7는 가속/감속 분리 마진
    # 2) 절대 상한 (폭주 방지)
    assert n3 <= 1.5 * n1, counts  # 실측 비율 1.30
    # 3) 3배 주행 후에도 단일 벽 두께 유계
    w = grid3.arrays["walls"]
    gi = grid3.coordinates_to_array_index(np.array([0.0, 0.12]))
    band = w[int(gi[0]) - 5:int(gi[0]) + 6, :]
    cols = np.where(band.any(axis=0))[0]
    th = [band[:, c].sum() for c in cols]
    assert np.mean(th) <= 4.5, f"3x 주행 후 두께 {np.mean(th):.2f}px"  # 실측 3.88px


def test_walls_survive_pose_drift():
    """heading 4° + 10mm 위치 드리프트가 누적돼도 기존 벽이 사라지지 않는다."""
    # 의도: 현재 설계(free_space_decrement=0)에선 항상 통과하는 게 정상.
    # 누군가 클리어링을 재활성화해 드리프트가 벽을 지우면 이 테스트가 알람을 울린다.
    grid, wm = make_mapper()
    walls_def, path = straight_corridor()
    run_path(grid, wm, walls_def, path)
    cov_before = coverage_of_segment(grid, (-0.25, 0.12), (0.25, 0.12))
    run_path(grid, wm, walls_def, path, seed=7, drift_heading_deg=4.0, drift_pos=0.010)
    cov_after = coverage_of_segment(grid, (-0.25, 0.12), (0.25, 0.12))
    assert cov_after >= cov_before - 0.05, (cov_before, cov_after)  # 실측: 드리프트 후에도 1.00 (append-only 설계)


def test_opening_preserved():
    """60mm 개구부가 닫히지 않는다."""
    walls_def = [((-0.4, 0.12), (-0.03, 0.12)), ((0.03, 0.12), (0.4, 0.12))]
    path = [(np.array([-0.3 + i * 0.004, 0.0]), np.pi / 2) for i in range(150)]
    grid, wm = make_mapper()
    run_path(grid, wm, walls_def, path)
    w = grid.arrays["walls"]
    gi_l = grid.coordinates_to_array_index(np.array([-0.02, 0.12]))
    gi_r = grid.coordinates_to_array_index(np.array([0.02, 0.12]))
    band = w[int(gi_l[0]) - 4:int(gi_l[0]) + 5, min(int(gi_l[1]), int(gi_r[1])):max(int(gi_l[1]), int(gi_r[1])) + 1]
    open_cols = (~band.any(axis=0)).sum()
    assert open_cols >= 4, f"개구부 잔여 폭 {open_cols}px"  # 60mm=10px 틈 − closing 침식 2px×2 = 최소 6px 예상, 실측 후 여유 4


def test_underflow_guard():
    """detected_points가 언더플로(0-1=65535)로 오염되지 않는다.

    주의: 프로덕션 기본값 free_space_decrement=0이면 클리어링 경로가 아예 실행되지
    않아 가드가 검증되지 않는다 → 여기서 명시적으로 1로 켜서 마스크-감산 가드를
    직접 통과시킨다 (켜도 언더플로 없어야 함).
    """
    grid, wm = make_mapper()
    wm.free_space_decrement = 1   # 가드 코드 경로 강제 실행
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
