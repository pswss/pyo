"""
5회차 완성본 — NumPy 배열, OpenCV
실행: python study/complete/05_complete.py
필요: pip install numpy opencv-python
"""

import numpy as np
import math

try:
    import cv2 as cv
    OPENCV_AVAILABLE = True
except ImportError:
    print("OpenCV 없이 NumPy 실습만 진행합니다.")
    OPENCV_AVAILABLE = False


print("=" * 50)
print("실습 1: NumPy 배열 기초")
print("=" * 50)

# 10x10 미로 (True=벽, False=통로)
maze = np.array([
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    [1, 0, 1, 0, 1, 0, 1, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 1, 1, 1, 0, 1, 0, 1],
    [1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
    [1, 1, 1, 0, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
], dtype=bool)

print(f"미로 크기: {maze.shape}")
print(f"벽 개수: {np.sum(maze)}")
print(f"통로 개수: {np.sum(~maze)}")   # ~ = 논리 NOT
print(f"[1,1] 은 벽인가?: {maze[1, 1]}")
print(f"[0,0] 은 벽인가?: {maze[0, 0]}")

wall_positions = np.argwhere(maze)
print(f"\n벽 위치 (처음 5개):")
for pos in wall_positions[:5]:
    print(f"  행={pos[0]}, 열={pos[1]}")

print()

# -------------------------------------------------------
# [완성] TODO 5-1
# -------------------------------------------------------

# 1) discovered 레이어: 처음엔 모두 미탐색(False)
discovered = np.zeros(maze.shape, dtype=bool)
# np.zeros: 모든 원소가 0 (bool 타입이면 False) 인 배열을 만든다.

# 2) 로봇이 (1,1) → (1,8) 경로를 이동했다고 가정
# 배열 슬라이싱: discovered[행, 열범위] = True
# 1:9 는 인덱스 1~8 (9는 포함 안 됨)을 의미한다.
discovered[1, 1:9] = True   # 1행의 1~8열을 탐색됨으로 표시

# 실제 map_visualizer.py 에서는 로봇이 이동할 때마다
# robot_mapper.map_discovered_by_robot() 가 170도 콘 형태로 discovered 를 True 로 설정한다.
# 여기서는 단순히 직선 경로로 시뮬레이션했다.

# 3) 탐색된 통로 비율
total_passages = np.sum(~maze)          # 미로 전체 통로 수 (~maze = 통로 = False 였던 곳)
discovered_passages = np.sum(discovered & ~maze)
# & (비트 AND): discovered=True 이고 maze=False(통로) 인 칸의 수
# 즉, "탐색됨" 이면서 "통로" 인 칸

ratio = discovered_passages / total_passages * 100 if total_passages > 0 else 0.0
print(f"탐색률: {ratio:.1f}%  ({discovered_passages}/{total_passages} 칸)")

print()

# victims 레이어
victims = np.zeros(maze.shape, dtype=bool)
victims[3, 5] = True
victims[7, 3] = True

victim_positions = np.argwhere(victims)
print(f"발견된 조난자 위치:")
for pos in victim_positions:
    print(f"  행={pos[0]}, 열={pos[1]}")


print()
print("=" * 50)
print("실습 2: OpenCV 로 미로 시각화하기")
print("=" * 50)

if not OPENCV_AVAILABLE:
    print("OpenCV 가 없어서 시각화 실습을 건너뜁니다.")
else:
    # -------------------------------------------------------
    # [완성] 레이어 합성 시각화
    #
    # map_visualizer.py 의 _render() 메서드와 완전히 동일한 원리:
    #   1. 배경 이미지 생성
    #   2. 불리언 레이어를 인덱스로 써서 해당 픽셀 색상을 덮어씀
    #   3. OpenCV 도형으로 특수 마커 추가
    # -------------------------------------------------------

    SCALE = 50       # 한 칸 = 50×50 픽셀
    H, W = maze.shape
    IMG_H, IMG_W = H * SCALE, W * SCALE

    # BGR 색상표 (map_visualizer.py 의 _COLORS 와 같은 구조)
    COLORS = {
        "undiscovered": ( 30,  30,  30),   # 미탐색 — 매우 어두운 회색
        "discovered":   ( 60,  60,  60),   # 탐색됨 — 어두운 회색
        "wall":         (220, 220, 220),   # 벽 — 밝은 흰색
        "victim":       (  0, 165, 255),   # 조난자 — 주황색 (BGR)
        "robot":        (  0,   0, 255),   # 로봇 — 빨간색 (BGR)
        "start":        (255,   0, 255),   # 출발점 — 마젠타 (BGR)
        "path":         (255, 220,   0),   # 경로 — 시안 (BGR)
    }

    def make_frame(maze_layer, discovered_layer, victims_layer,
                   robot_pos, start_pos, path_cells=None):
        """
        여러 레이어를 합성해서 시각화 이미지(ndarray)를 반환한다.

        레이어 합성 순서가 중요하다. 나중에 그린 것이 위에 덮인다.
        map_visualizer.py 의 _render() 와 같은 순서를 따른다:
          배경 → 탐색됨 → 벽 → 조난자(원) → 경로 → 출발점 → 로봇
        """
        # 1. 배경: 모두 미탐색 색상으로 시작
        image = np.full((H, W, 3), COLORS["undiscovered"], dtype=np.uint8)

        # 2. 탐색된 영역: 불리언 마스킹으로 한 번에 색 변경
        # image[discovered_layer] = COLORS["discovered"]
        # → discovered_layer 가 True 인 픽셀만 discovered 색으로 덮어씀
        image[discovered_layer] = COLORS["discovered"]

        # 3. 벽: 탐색 여부와 무관하게 항상 흰색
        image[maze_layer] = COLORS["wall"]

        # 4. SCALE 배로 확대 (각 픽셀을 50x50 블록으로)
        # INTER_NEAREST: 가장 가까운 픽셀값을 사용 → 픽셀 경계가 선명
        big = cv.resize(image, (IMG_W, IMG_H), interpolation=cv.INTER_NEAREST)

        # 5. 경로 표시 (작은 점들)
        if path_cells:
            for cell in path_cells:
                # NumPy: (row, col) → OpenCV: (x=col*SCALE+중앙, y=row*SCALE+중앙)
                cx = cell[1] * SCALE + SCALE // 2
                cy = cell[0] * SCALE + SCALE // 2
                cv.circle(big, (cx, cy), 4, COLORS["path"], -1)

        # 6. 조난자: 주황 원 + 흰색 테두리
        for pos in np.argwhere(victims_layer):
            cx = int(pos[1] * SCALE + SCALE // 2)
            cy = int(pos[0] * SCALE + SCALE // 2)
            cv.circle(big, (cx, cy), SCALE // 3, COLORS["victim"], -1)  # 채움
            cv.circle(big, (cx, cy), SCALE // 3, (255, 255, 255), 2)    # 흰 테두리

        # 7. 출발점: 마젠타 별 마커
        sx = int(start_pos[1] * SCALE + SCALE // 2)
        sy = int(start_pos[0] * SCALE + SCALE // 2)
        cv.drawMarker(big, (sx, sy), COLORS["start"],
                      cv.MARKER_STAR, SCALE // 2, 2)

        # 8. 로봇: 빨간 원
        rx = int(robot_pos[1] * SCALE + SCALE // 2)
        ry = int(robot_pos[0] * SCALE + SCALE // 2)
        cv.circle(big, (rx, ry), SCALE // 3, COLORS["robot"], -1)

        return big

    # 시뮬레이션 설정
    robot_pos  = [1, 1]
    start_pos  = [1, 1]
    sim_disc   = np.zeros(maze.shape, dtype=bool)
    visited    = []   # 지금까지 방문한 칸들 (경로 표시용)

    # 이동 경로
    path = [
        [1,1],[1,2],[1,3],
        [2,3],[3,3],[3,4],[3,5],
        [4,5],[5,5],[5,4],[5,3],[5,2],[5,1],
        [6,3],[7,3],[7,4],[7,5],[7,6],[7,7],[7,8],
    ]

    print("미로 탐색 시뮬레이션 (아무 키: 다음 / q: 종료)")

    for step, pos in enumerate(path):
        row, col = pos

        if maze[row, col]:   # 벽이면 건너뜀
            continue

        robot_pos = pos
        visited.append(tuple(pos))

        # 로봇 주변 3x3 을 탐색됨으로 표시
        # → 실제로는 170도 시야각 콘, 여기서는 단순히 3x3 박스
        r0, r1 = max(0, row-1), min(H-1, row+1)
        c0, c1 = max(0, col-1), min(W-1, col+1)
        sim_disc[r0:r1+1, c0:c1+1] = True

        # 탐색률 계산
        total_p = int(np.sum(~maze))
        found_p = int(np.sum(sim_disc & ~maze))
        pct = found_p / total_p * 100 if total_p > 0 else 0

        # 렌더링
        frame = make_frame(maze, sim_disc, victims,
                           robot_pos, start_pos, visited)

        # 텍스트 오버레이
        cv.putText(frame,
                   f"Step {step+1}/{len(path)}",
                   (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv.putText(frame,
                   f"Explored: {pct:.0f}%  ({found_p}/{total_p})",
                   (10, IMG_H - 10), cv.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        cv.imshow("미로 탐색 시뮬레이션 (완성본)", frame)
        key = cv.waitKey(250)   # 250ms 대기 (자동 진행)
        if key == ord('q'):
            break

    # 최종 상태에서 키 입력 대기
    cv.waitKey(0)
    cv.destroyAllWindows()
    print(f"시뮬레이션 완료. 최종 탐색률: {pct:.1f}%")


print()
print("=" * 50)
print("실습 3: map_visualizer.py 수정 가이드 [정답]")
print("=" * 50)
print("""
[색상 변경 방법]
src/map_visualizer.py 의 _COLORS 딕셔너리:

  "traversed": (180, 80, 20)  →  파란색: (255, 100, 0)
  "path":      (255, 220, 0)  →  노란색: (0, 255, 255)

BGR 순서이므로:
  파란색(RGB: 0,100,255) → BGR: (255,100,0)
  노란색(RGB: 255,255,0) → BGR: (0,255,255)

[BGR 변환 정답]
  하늘색 RGB(135,206,235) → BGR(235,206,135)
  금색   RGB(255,215,0)   → BGR(0,215,255)

[탐색률 표시 추가 방법]
_render() 메서드 맨 끝에 추가:

  total = np.sum(~self._mapper.pixel_grid.arrays["occupied"])
  disc  = np.sum(self._mapper.pixel_grid.arrays["discovered"])
  pct   = disc / total * 100 if total > 0 else 0
  cv.putText(image,
             f"Explored: {pct:.0f}%",
             (5, image.shape[0] - 5),
             cv.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)
""")

print("=" * 50)
print("[완료] 5회차 완성본 실행 완료")
print("=" * 50)
print("""
이제 src/ 코드의 핵심 패턴을 모두 이해했습니다.

도전 과제:

1. [쉬움] map_visualizer.py 에 탐색률(%) 텍스트 오버레이 추가
   → cv.putText() 사용

2. [중간] rescue_robot.py 에 auto_return() 메서드 추가
   → remaining_time < threshold 이면 go_to_start() 호출
   → is_at_start 가 True 이면 finish_mission() 호출

3. [어려움] executor.py 에 DEBUG_LEVEL 기반 로그 레벨 추가
   → flags.py 에 DEBUG_LEVEL = 2 추가 (0=없음, 1=중요, 2=전체)
   → print 를 def log(msg, level=1): if level <= DEBUG_LEVEL: print(msg) 로 교체

4. [도전] study/complete/03_complete.py 의 SpiralSearchAgent 를
   실제 src/agent/agent.py 에 추가하고 SubagentPriorityCombiner 에 넣어보기
""")
