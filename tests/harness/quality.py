"""미로 전체 주행 후 walls 레이어를 진실 벽과 비교해 precision/recall 산출."""
import numpy as np
import cv2 as cv

from harness.virtual_lidar import cast_rays
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from mapping.wall_mapper import WallMapper

RES = 10 / 0.06   # 타일당 10px / 반타일 0.06m (1px=6mm) — src Mapper와 동일
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
