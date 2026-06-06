"""
5회차 실습 — NumPy 배열, OpenCV
실행: python study/05_practice.py
필요: pip install numpy opencv-python

이 실습은 Webots 없이 순수 파이썬으로 실행됩니다.
map_visualizer.py 와 동일한 원리로 미로 지도를 시각화합니다.
"""

import numpy as np
import math

try:
    import cv2 as cv
    OPENCV_AVAILABLE = True
except ImportError:
    print("OpenCV 가 설치되지 않았습니다.")
    print("설치: pip install opencv-python")
    print("OpenCV 없이 NumPy 실습만 진행합니다.\n")
    OPENCV_AVAILABLE = False


print("=" * 50)
print("실습 1: NumPy 배열 기초")
print("=" * 50)

# -------------------------------------------------------
# [개념] NumPy 배열 생성과 조작
# -------------------------------------------------------

# 간단한 10x10 미로 (0=통로, 1=벽)
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
], dtype=bool)   # True=벽, False=통로

print(f"미로 크기: {maze.shape}")   # (10, 10)
print(f"벽 개수: {np.sum(maze)}")
print(f"통로 개수: {np.sum(~maze)}")  # ~ 는 NOT 연산

# 특정 위치 확인
print(f"[1,1] 은 벽인가?: {maze[1, 1]}")   # False (통로)
print(f"[0,0] 은 벽인가?: {maze[0, 0]}")   # True (벽)

# 벽의 위치 목록
wall_positions = np.argwhere(maze)
print(f"\n벽 위치 (처음 5개):")
for pos in wall_positions[:5]:
    print(f"  행={pos[0]}, 열={pos[1]}")

print()

# -------------------------------------------------------
# TODO 5-1: 아래 작업을 완성하세요.
# -------------------------------------------------------

# 1) discovered 레이어 만들기 (처음에는 전부 미탐색 = False)
discovered = np.zeros(maze.shape, dtype=bool)

# 2) 로봇이 (1,1) 에서 (1,8) 까지 이동했다고 가정하고
#    해당 경로를 discovered = True 로 표시하세요
#    힌트: discovered[1, 1:9] = True
# TODO: 경로 표시

# 3) 탐색된 통로 비율 계산
total_passages = np.sum(~maze)
discovered_passages = np.sum(discovered & ~maze)   # 탐색된 + 통로인 곳
if total_passages > 0:
    ratio = discovered_passages / total_passages * 100
    # TODO: f-string 으로 "탐색률: XX.X%" 출력

print()

# -------------------------------------------------------
# victims 레이어 만들기
victims = np.zeros(maze.shape, dtype=bool)
victims[3, 5] = True    # 조난자 1
victims[7, 3] = True    # 조난자 2

victim_positions = np.argwhere(victims)
print(f"발견된 조난자 위치:")
for pos in victim_positions:
    print(f"  행={pos[0]}, 열={pos[1]}")


print()
print("=" * 50)
print("실습 2: OpenCV 로 미로 시각화하기")
print("=" * 50)

if not OPENCV_AVAILABLE:
    print("OpenCV 가 없어서 이 실습을 건너뜁니다.")
else:
    # -------------------------------------------------------
    # [개념] 불리언 레이어 → 색상 이미지
    #
    # 1. 배경 이미지 생성
    # 2. 각 레이어에 해당하는 픽셀을 해당 색상으로 덮어씀
    # 3. OpenCV 도형으로 로봇, 조난자 등 특수 표시 추가
    # -------------------------------------------------------

    SCALE = 50    # 한 칸을 몇 픽셀로 표시할지
    H, W = maze.shape
    IMG_H, IMG_W = H * SCALE, W * SCALE

    # 색상표 (BGR)
    COLORS = {
        "undiscovered": (30,  30,  30),   # 미탐색 — 매우 어두운 회색
        "discovered":   (60,  60,  60),   # 탐색됨 — 어두운 회색
        "wall":         (220, 220, 220),  # 벽 — 밝은 흰색
        "victim":       (  0, 165, 255),  # 조난자 — 주황색
        "robot":        (  0,   0, 255),  # 로봇 — 빨간색
        "start":        (255,   0, 255),  # 출발점 — 마젠타
    }

    def render_maze(maze_layer, discovered_layer, victims_layer,
                    robot_pos, start_pos):
        """
        여러 레이어를 합성해서 미로 이미지를 반환합니다.
        map_visualizer.py 의 _render 메서드와 동일한 원리입니다.
        """
        # 1. 배경 이미지 생성
        image = np.full((H, W, 3), COLORS["undiscovered"], dtype=np.uint8)

        # 2. 탐색된 영역 표시 (불리언 마스킹)
        image[discovered_layer] = COLORS["discovered"]

        # 3. 벽 표시
        image[maze_layer] = COLORS["wall"]

        # 4. SCALE 배로 확대 (픽셀 경계 선명하게)
        big = cv.resize(image, (IMG_W, IMG_H), interpolation=cv.INTER_NEAREST)

        # 5. 조난자 표시 (원 + 흰색 테두리)
        for pos in np.argwhere(victims_layer):
            # NumPy: (row, col) → OpenCV: (x=col*SCALE, y=row*SCALE)
            cx = int(pos[1] * SCALE + SCALE // 2)
            cy = int(pos[0] * SCALE + SCALE // 2)
            cv.circle(big, (cx, cy), SCALE // 3, COLORS["victim"], -1)
            cv.circle(big, (cx, cy), SCALE // 3, (255, 255, 255), 2)

        # 6. 출발점 표시 (마젠타 별)
        sx = int(start_pos[1] * SCALE + SCALE // 2)
        sy = int(start_pos[0] * SCALE + SCALE // 2)
        cv.drawMarker(big, (sx, sy), COLORS["start"],
                      cv.MARKER_STAR, SCALE // 2, 2)

        # 7. 로봇 위치 표시 (빨간 원)
        rx = int(robot_pos[1] * SCALE + SCALE // 2)
        ry = int(robot_pos[0] * SCALE + SCALE // 2)
        cv.circle(big, (rx, ry), SCALE // 3, COLORS["robot"], -1)

        return big

    # 시뮬레이션: 로봇이 이동하면서 지도 탐색
    robot_position = [1, 1]   # [행, 열]
    start_position = [1, 1]
    sim_discovered = np.zeros(maze.shape, dtype=bool)

    # 이동 경로 정의
    path = [
        [1,1],[1,2],[1,3],[1,4],   # 상단 왼쪽 이동 실패 (벽)
        [1,3],[2,3],[3,3],[3,4],[3,5],  # 아래로 이동
        [4,5],[5,5],[5,4],[5,3],[5,2],[5,1],  # 계속 이동
        [6,3],[7,3],[7,4],[7,5],[7,6],[7,7],[7,8],  # 하단 이동
    ]

    print("로봇 이동 시뮬레이션 시작 (아무 키나 누르면 다음 단계)")
    print("'q' 키를 누르면 종료합니다.")

    for step, pos in enumerate(path):
        row, col = pos

        # 벽이면 건너뜀
        if maze[row, col]:
            continue

        robot_position = pos

        # 로봇 주변 3×3 영역을 탐색됨으로 표시
        r_min = max(0, row - 1)
        r_max = min(H - 1, row + 1)
        c_min = max(0, col - 1)
        c_max = min(W - 1, col + 1)
        sim_discovered[r_min:r_max+1, c_min:c_max+1] = True

        # 렌더링
        frame = render_maze(maze, sim_discovered, victims,
                            robot_position, start_position)

        # 탐색률 표시
        total_p = np.sum(~maze)
        found_p = np.sum(sim_discovered & ~maze)
        pct = found_p / total_p * 100 if total_p > 0 else 0
        cv.putText(frame,
                   f"Step {step+1}/{len(path)}  Explored: {pct:.0f}%",
                   (10, IMG_H - 10),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv.imshow("미로 탐색 시뮬레이션", frame)
        key = cv.waitKey(300)   # 300ms 대기
        if key == ord('q'):
            break

    cv.waitKey(0)
    cv.destroyAllWindows()


print()
print("=" * 50)
print("실습 3: map_visualizer.py 수정해보기")
print("=" * 50)

print("""
실제 소스 파일을 수정하는 실습입니다.

1. src/map_visualizer.py 를 열어보세요.

2. _COLORS 딕셔너리에서 색상을 변경해보세요:
   - "traversed" 색상을 파란색 (255, 100, 0) 으로 변경
   - "path" 색상을 노란색 (0, 255, 255) 으로 변경

3. 색상 추가 실습 (도전!):
   - 로봇이 지나간 경로를 그라데이션 효과로 표시하려면
     _render 메서드의 어느 부분을 수정해야 할까요?

4. BGR vs RGB 확인:
   다음 색상의 BGR 값을 구해보세요:
   - 하늘색 (RGB: 135, 206, 235) → BGR: ???
   - 금색 (RGB: 255, 215, 0)    → BGR: ???

   힌트: BGR 은 RGB 를 거꾸로 쓰면 됩니다.
   하늘색 = (235, 206, 135)
   금색   = (0, 215, 255)
""")


print("=" * 50)
print("[완료] 5회차 전체 완료! 다음 단계")
print("=" * 50)
print("""
이제 src/ 코드의 핵심 패턴을 모두 배웠습니다.

다음에 도전해볼 것들:

1. [쉬움] map_visualizer.py 에 현재 탐색률(%)을 화면에 표시하기
   힌트: cv.putText() 사용

2. [중간] rescue_robot.py 에 go_home() 메서드 추가하기
   - 남은 시간이 X초 이하면 자동으로 출발점으로 복귀
   - is_time_almost_up 프로퍼티를 활용

3. [어려움] flags.py 에 DEBUG_LEVEL = 0~3 을 추가하고
   executor.py 의 print 문을 레벨별로 제어하기

4. [도전] 새로운 SubAgent 를 상속으로 구현하기
   - SubAgent 인터페이스를 상속
   - update() 메서드에 새로운 탐색 전략 구현
   - agent.py 의 SubagentPriorityCombiner 에 추가
""")
