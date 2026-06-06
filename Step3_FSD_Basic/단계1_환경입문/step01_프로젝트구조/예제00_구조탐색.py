"""
[예제 00] 프로젝트 구조 탐색기
---------------------------------
터미널에서 실행: python 예제00_구조탐색.py

src/ 폴더를 분석해서 각 모듈의 역할을 시각적으로 출력합니다.
Webots 없이 실행 가능합니다.
"""

import os
import sys

# ── 경로 설정 ─────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR    = os.path.normpath(os.path.join(SCRIPT_DIR, "../../../src"))

# ── 모듈 역할 설명 ────────────────────────────────────────────
MODULE_ROLES = {
    "main.py":           "프로그램 시작점 - Robot, Mapper, Executor를 만들고 실행",
    "flags.py":          "전역 옵션 - SHOW_DEBUG, DO_SLOW_DOWN 등 ON/OFF 스위치",
    "utilities.py":      "공통 유틸 - ColorFilterTuner 등",
    "robot":             "로봇 하드웨어 추상화 계층",
    "robot/robot.py":    "Robot 클래스 - 모든 센서/바퀴를 하나로 묶음",
    "robot/drive_base.py": "DriveBase - 바퀴 속도 제어 + 목표 이동/회전",
    "robot/pose_manager.py": "PoseManager - GPS + 자이로로 위치/방향 계산",
    "robot/devices":     "개별 센서 클래스 (GPS, 카메라, 라이다, 자이로, 통신기)",
    "mapping":           "지도 생성 시스템",
    "mapping/mapper.py": "Mapper - 지도 시스템 전체 조율",
    "mapping/wall_mapper.py":    "WallMapper - 라이다로 벽 위치 기록",
    "mapping/floor_mapper.py":   "FloorMapper - 바닥 색(늪/체크포인트) 감지",
    "mapping/fixture_mapper.py": "FixtureMapper - 발견 피해자 위치 기록",
    "executor":          "전체 실행 흐름 관리 (두뇌)",
    "executor/executor.py":      "Executor - 상태기계 + 매 스텝 update 순서 관리",
    "executor/stuck_detector.py": "StuckDetector - 바퀴 멈춤 감지",
    "agent":             "목적지 결정 (어디로 갈지)",
    "agent/agent.py":    "Agent - 서브에이전트들을 우선순위로 조합",
    "agent/subagents":   "개별 전략: 피해자추격 / 벽따라가기 / 미발견구역탐색",
    "algorithms":        "경로 탐색 알고리즘",
    "algorithms/np_bool_array/efficient_a_star.py": "A* - 최단경로 탐색",
    "fixture_detection": "피해자(H/S/U) 감지 시스템",
    "fixture_detection/fixture_clasification.py": "카메라 이미지에서 피해자 분류",
    "fixture_detection/color_filter.py": "HSV 색상 필터링",
    "flow_control":      "실행 흐름 제어 유틸",
    "flow_control/state_machine.py": "StateMachine - 상태 등록/전환/실행",
    "flow_control/sequencer.py":     "Sequencer - 비동기 순차 실행",
    "flow_control/delay.py":         "DelayManager - 시간 기반 대기",
    "data_structures":   "공통 자료형",
    "data_structures/angle.py":      "Angle - 각도 (라디안/도 변환, 정규화)",
    "data_structures/vectors.py":    "Position2D, Vector2D - 2D 위치/벡터",
}

# ── 실제 파일 목록 출력 ───────────────────────────────────────
print("=" * 65)
print("  프로젝트 소스 구조 분석")
print("=" * 65)

if not os.path.isdir(SRC_DIR):
    print(f"\n  [오류] src/ 폴더를 찾을 수 없습니다: {SRC_DIR}")
    sys.exit(1)

print(f"\n  경로: {SRC_DIR}\n")

def count_lines(filepath):
    try:
        with open(filepath, encoding="utf-8", errors="ignore") as f:
            return sum(1 for _ in f)
    except Exception:
        return 0

total_files = 0
total_lines = 0

for root, dirs, files in sorted(os.walk(SRC_DIR)):
    dirs.sort()
    rel_root = os.path.relpath(root, SRC_DIR)
    depth = 0 if rel_root == "." else rel_root.count(os.sep) + 1

    if depth > 2:
        continue

    indent = "  " + "  " * depth
    folder = os.path.basename(root)

    if rel_root != ".":
        folder_key = rel_root.replace("\\", "/")
        role = MODULE_ROLES.get(folder_key, "")
        role_str = f"  <- {role}" if role else ""
        print(f"{indent}[폴더] {folder}/{role_str}")

    py_files = [f for f in sorted(files) if f.endswith(".py")]
    for fname in py_files:
        fpath     = os.path.join(root, fname)
        lines     = count_lines(fpath)
        rel_key   = os.path.join(rel_root, fname).replace("\\", "/")
        if rel_key.startswith("./"):
            rel_key = rel_key[2:]
        role      = MODULE_ROLES.get(rel_key, "")
        role_str  = f"  <- {role}" if role else ""
        fsize_str = f"({lines}줄)"

        print(f"{indent}  [파일] {fname} {fsize_str}{role_str}")
        total_files += 1
        total_lines += lines

print()
print(f"  총 {total_files}개 파일 / {total_lines:,}줄")

# ── 핵심 클래스 관계도 ────────────────────────────────────────
print()
print("=" * 65)
print("  핵심 클래스 관계도")
print("=" * 65)
print("""
  main.py
    │
    ├── Robot(time_step=32)        <- 감각 + 움직임
    │     ├── GPS                  <- 위치 (x, y)
    │     ├── Gyroscope            <- 방향 (각도)
    │     ├── Lidar                <- 거리 센서 (360°)
    │     ├── Camera × 3          <- 이미지 (앞/좌/우)
    │     └── DriveBase            <- 바퀴 속도 제어
    │
    ├── Mapper(tile_size=0.12)     <- 지도 생성
    │     ├── WallMapper           <- 벽 기록
    │     ├── FloorMapper          <- 바닥 기록
    │     └── FixtureMapper        <- 피해자 기록
    │
    └── Executor(mapper, robot)    <- 두뇌 (전체 조율)
          ├── StateMachine         <- 현재 상태 관리
          │     ├── "init"         -> 초기화/보정
          │     ├── "explore"      -> 탐색 (A*로 이동)
          │     ├── "report_fixture" -> 피해자 신고
          │     ├── "send_map"     -> 지도 전송
          │     ├── "stuck"        -> 갇힘 탈출
          │     └── "end"          -> 임무 종료
          │
          ├── Agent                <- 목표 좌표 결정
          │     ├── GoToFixturesAgent    (우선순위 1)
          │     ├── FollowWallsAgent     (우선순위 2)
          │     └── GoToNonDiscoveredAgent (우선순위 3)
          │
          └── FixtureClassifier    <- 피해자 분류 (H/S/U/위험)
""")

# ── 데이터 흐름 시뮬레이션 ────────────────────────────────────
print("=" * 65)
print("  한 스텝(32ms)의 데이터 흐름")
print("=" * 65)

steps = [
    ("robot.update()",            "GPS/자이로/라이다/카메라 값 최신화"),
    ("mapper.update(point_cloud)", "라이다 포인트 -> 격자 지도에 벽 표시"),
    ("agent.update()",            "지도 분석 -> 다음 목표 좌표 계산"),
    ("robot.move_to_coords(pos)", "목표 좌표로 바퀴 속도 결정 & 이동"),
    ("fixture_detector.find(img)","카메라 이미지 분석 -> 피해자 감지 여부"),
    ("state_machine.run()",       "현재 상태 실행 + 필요시 상태 전환"),
]

print()
for i, (func, desc) in enumerate(steps, 1):
    arrow = "↓" if i < len(steps) else "↺"
    print(f"  {i}. {func:<35} {desc}")
    if i < len(steps):
        print(f"     {arrow}")
print()

print("  ※ 이 과정이 8분간 약 15,000번 반복됩니다!")
print()

# ── 자주 묻는 질문 ────────────────────────────────────────────
print("=" * 65)
print("  FAQ")
print("=" * 65)
faqs = [
    ("compiled.py를 수정해도 되나요?",
     "절대 안 됩니다! compiled.py는 src/를 번들링한 대회용 파일입니다.\n"
     "     개발은 src/ 안에서 하고 테스트 후 재컴파일합니다."),
    ("어디서부터 코드를 읽어야 하나요?",
     "main.py -> executor/executor.py -> agent/agent.py 순서로 읽으세요.\n"
     "     robot.py는 '블랙박스'로 먼저 사용법만 익혀도 됩니다."),
    ("버그 수정 후 점수가 낮아질 수 있나요?",
     "재컴파일할 때 src/ 코드가 compiled.py보다 오래된 경우 그렇습니다.\n"
     "     항상 버전을 확인하고 테스트하세요."),
]
print()
for q, a in faqs:
    print(f"  Q: {q}")
    print(f"     A: {a}")
    print()
