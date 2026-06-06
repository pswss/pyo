"""
[예제 00c] 데이터 흐름 추적 시뮬레이터
-----------------------------------------
터미널에서 실행: python 예제00c_데이터흐름추적.py

센서 -> 지도 -> 결정 -> 행동까지의 데이터 변환 과정을
단계별로 시각화합니다. Webots 없이 실행 가능합니다.
"""

import math
import random

print("=" * 65)
print("  데이터 흐름 추적 시뮬레이터")
print("=" * 65)

# ── 1. 센서 데이터 시뮬레이션 ────────────────────────────────
print("\n[1단계] 센서 데이터 읽기 - robot.update()")
print("-" * 65)

class FakeGPS:
    def read(self):
        return (0.24, -0.36)   # (x, y) 미터

class FakeGyroscope:
    def read(self):
        return math.pi / 2     # 90도 (동쪽 방향)

class FakeLidar:
    def read(self, robot_pos, robot_angle):
        """로봇 위치를 중심으로 가상 라이다 포인트 생성"""
        points = []
        rx, ry = robot_pos
        # 북/동/남쪽은 벽 있음, 서쪽은 열려 있음
        wall_angles = {
            0:   0.3,    # 북쪽 30cm
            90:  0.5,    # 동쪽 50cm
            180: 0.3,    # 남쪽 30cm
        }
        for angle_deg, dist in wall_angles.items():
            rad = math.radians(angle_deg)
            wx = rx + dist * math.cos(rad)
            wy = ry + dist * math.sin(rad)
            points.append((round(wx, 3), round(wy, 3)))
        return points

class FakeCamera:
    def __init__(self, has_victim=False):
        self.has_victim = has_victim

    def read(self):
        if self.has_victim:
            return {"type": "H", "confidence": 0.85, "position": (64, 48)}
        return None

gps   = FakeGPS()
gyro  = FakeGyroscope()
lidar = FakeLidar()
cam   = FakeCamera(has_victim=True)

pos   = gps.read()
angle = gyro.read()
cloud = lidar.read(pos, angle)
image = cam.read()

print(f"  GPS 위치:          x={pos[0]:.2f}m, y={pos[1]:.2f}m")
print(f"  자이로 방향:        {math.degrees(angle):.1f}° ({angle:.4f} rad)")
print(f"  라이다 포인트:      {len(cloud)}개 감지")
for i, pt in enumerate(cloud):
    dx = pt[0] - pos[0]
    dy = pt[1] - pos[1]
    dist = math.sqrt(dx**2 + dy**2)
    print(f"    [{i+1}] ({pt[0]:+.3f}, {pt[1]:+.3f}) - 거리 {dist:.2f}m")
print(f"  카메라:             {'피해자 감지됨 (타입: H, 신뢰도: 85%)' if image else '감지 없음'}")

# ── 2. 지도 가공 ──────────────────────────────────────────────
print("\n[2단계] 지도 생성 - mapper.update()")
print("-" * 65)

GRID_SIZE   = 10
TILE_SIZE   = 0.12
START_TILES = (4, 4)   # 격자 중앙을 출발점으로

def world_to_grid(wx, wy):
    """월드 좌표 -> 격자 좌표"""
    gx = int(wx / TILE_SIZE) + START_TILES[0]
    gy = int(wy / TILE_SIZE) + START_TILES[1]
    return max(0, min(GRID_SIZE-1, gx)), max(0, min(GRID_SIZE-1, gy))

grid = [['.' for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

# 로봇 위치 표시
rx_g, ry_g = world_to_grid(*pos)
grid[ry_g][rx_g] = 'R'

# 벽 위치 표시
for wx, wy in cloud:
    gx, gy = world_to_grid(wx, wy)
    if (gx, gy) != (rx_g, ry_g):
        grid[gy][gx] = '#'

print(f"\n  격자 지도 (tile={TILE_SIZE}m, 범위={GRID_SIZE}×{GRID_SIZE})")
print(f"  R=로봇, #=벽, .=빈공간\n")
print("    " + " ".join(str(i) for i in range(GRID_SIZE)))
for row_i, row in enumerate(grid):
    print(f"  {row_i} " + " ".join(row))

print(f"\n  로봇 격자 위치: ({rx_g}, {ry_g})")
wall_cells = [(x, y) for y, row in enumerate(grid)
              for x, c in enumerate(row) if c == '#']
print(f"  벽 격자 수:    {len(wall_cells)}칸")

# ── 3. 에이전트 결정 ─────────────────────────────────────────
print("\n[3단계] 목표 결정 - agent.update()")
print("-" * 65)

def find_nearest_undiscovered(grid, robot_pos):
    """BFS로 가장 가까운 미탐색 구역 찾기 (단순 시뮬레이션)"""
    from collections import deque
    rx, ry = robot_pos
    queue  = deque([(rx, ry, 0)])
    visited = {(rx, ry)}

    while queue:
        x, y, dist = queue.popleft()
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and (nx,ny) not in visited:
                visited.add((nx,ny))
                if grid[ny][nx] == '.':
                    return (nx, ny), dist+1
                queue.append((nx, ny, dist+1))
    return None, -1

agents_checked = [
    ("GoToFixturesAgent",    image is not None,  "피해자가 감지된 경우 실행 가능"),
    ("FollowWallsAgent",     len(cloud) >= 2,    "벽이 2개 이상 있는 경우 실행 가능"),
    ("GoToNonDiscoveredAgent", True,             "항상 실행 가능 (최후 수단)"),
]

selected_agent = None
for name, possible, condition in agents_checked:
    status = "O 가능" if possible else "X 불가"
    print(f"  {status} | {name:<30} ({condition})")
    if possible and selected_agent is None:
        selected_agent = name

print(f"\n  선택된 에이전트: {selected_agent}")

if selected_agent == "GoToFixturesAgent":
    target_world = (pos[0] + 0.12, pos[1])
    print(f"  목표: 피해자 위치로 이동 -> ({target_world[0]:.2f}, {target_world[1]:.2f})")
elif selected_agent == "FollowWallsAgent":
    target_world = (pos[0], pos[1] + 0.12)
    print(f"  목표: 벽 근처 탐색 -> ({target_world[0]:.2f}, {target_world[1]:.2f})")
else:
    target_grid, dist = find_nearest_undiscovered(grid, (rx_g, ry_g))
    tx = (target_grid[0] - START_TILES[0]) * TILE_SIZE
    ty = (target_grid[1] - START_TILES[1]) * TILE_SIZE
    target_world = (tx, ty)
    print(f"  목표: 미탐색 구역 -> 격자{target_grid} = ({tx:.2f}, {ty:.2f})")

# ── 4. 이동 명령 계산 ─────────────────────────────────────────
print("\n[4단계] 바퀴 속도 계산 - drive_base.move_to_coords()")
print("-" * 65)

def calc_wheel_speeds(robot_pos, robot_angle, target_pos):
    """목표 좌표까지 차동 구동 속도 계산 (단순화)"""
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]
    distance = math.sqrt(dx**2 + dy**2)

    target_angle = math.atan2(dy, dx)
    angle_diff = target_angle - robot_angle

    # 각도 정규화 (-π ~ π)
    while angle_diff >  math.pi: angle_diff -= 2 * math.pi
    while angle_diff < -math.pi: angle_diff += 2 * math.pi

    # 거리에 따른 기본 속도
    base_speed = min(1.0, distance * 3)

    # 각도 차이에 따른 회전 보정
    turn = angle_diff / math.pi   # -1.0 ~ 1.0

    left_speed  = base_speed * (1 - turn)
    right_speed = base_speed * (1 + turn)

    # 클리핑 (-1 ~ 1)
    left_speed  = max(-1.0, min(1.0, left_speed))
    right_speed = max(-1.0, min(1.0, right_speed))

    return left_speed, right_speed, distance, math.degrees(angle_diff)

left, right, dist, angle_err = calc_wheel_speeds(pos, angle, target_world)

print(f"  현재 위치:   ({pos[0]:.2f}, {pos[1]:.2f})")
print(f"  목표 위치:   ({target_world[0]:.2f}, {target_world[1]:.2f})")
print(f"  남은 거리:   {dist:.3f}m")
print(f"  각도 오차:   {angle_err:.1f}°")
print(f"\n  바퀴 속도 명령:")
print(f"    왼쪽:  {left:+.3f}  {'<<' if left < 0 else '>>' if left > 0.5 else '>'}")
print(f"    오른쪽: {right:+.3f}  {'<<' if right < 0 else '>>' if right > 0.5 else '>'}")

if abs(left - right) < 0.05:
    movement = "직진"
elif left > right:
    movement = "우회전"
else:
    movement = "좌회전"
print(f"    -> 동작: {movement}")

# ── 5. 전체 요약 ─────────────────────────────────────────────
print("\n[요약] 한 스텝(32ms)에 일어난 일")
print("=" * 65)
summary = [
    ("센서 읽기",  f"GPS({pos[0]:.2f},{pos[1]:.2f}), 자이로({math.degrees(angle):.0f}°), 라이다({len(cloud)}점)"),
    ("지도 갱신",  f"격자 {GRID_SIZE}×{GRID_SIZE}, 벽 {len(wall_cells)}칸 기록"),
    ("피해자 감지", f"{'H 타입 감지됨' if image else '없음'}"),
    ("에이전트",   f"{selected_agent} 선택"),
    ("목표 설정",  f"({target_world[0]:.2f}, {target_world[1]:.2f})"),
    ("바퀴 명령",  f"L={left:+.2f}, R={right:+.2f} -> {movement}"),
]
for label, value in summary:
    print(f"  {label:<12}: {value}")

print()
print("  다음 스텝에서 이 과정이 다시 반복됩니다 (반복)")
