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
    d = np.stack([dx, dy], axis=1)                # (R,2)
    po = p[None, :, :] - o[None, None, :]         # (1,S,2) — robot_pos 기준 세그먼트 시작점
    cross_ds = d[:, None, 0] * s[None, :, 1] - d[:, None, 1] * s[None, :, 0]   # (R,S)
    cross_pos = po[..., 0] * s[None, :, 1] - po[..., 1] * s[None, :, 0]        # (1,S)→브로드캐스트 OK
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
