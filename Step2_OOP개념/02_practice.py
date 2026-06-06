"""
2회차 실습 — 모듈, 임포트, f-string
실행: python study/02_practice.py

이 파일을 실행하기 전에 02_robot_config.py 를 먼저 만들어야 합니다.
(아래 TODO 1 참고)
"""

print("=" * 50)
print("실습 1: 설정 파일(모듈) 만들기")
print("=" * 50)

# -------------------------------------------------------
# TODO 2-1: study 폴더 안에 02_robot_config.py 파일을 새로 만드세요.
#
# 파일 내용:
#   TILE_SIZE = 0.12          # 타일 크기 (미터)
#   MAX_SPEED = 1.0           # 최대 속도
#   TIME_LIMIT_SEC = 8 * 60  # 대회 제한 시간 (초)
#   SHOW_LOG = True           # 로그 출력 여부
#
#   def format_position(x, y):
#       """좌표를 '(x, y)m' 형식 문자열로 반환"""
#       return f"({x:.3f}, {y:.3f})m"
#
# 파일을 만든 후 아래 import 주석을 해제하세요:
# -------------------------------------------------------

# import sys, os
# sys.path.insert(0, os.path.dirname(__file__))   # study 폴더를 경로에 추가
# from robot_config_02 import TILE_SIZE, MAX_SPEED, TIME_LIMIT_SEC, format_position

# print(f"타일 크기: {TILE_SIZE}m")
# print(f"최대 속도: {MAX_SPEED}")
# print(f"제한 시간: {TIME_LIMIT_SEC}초 ({TIME_LIMIT_SEC // 60}분)")
# print(f"현재 위치: {format_position(0.12, 0.48)}")


print()
print("=" * 50)
print("실습 2: f-string 연습")
print("=" * 50)

# -------------------------------------------------------
# [개념] f-string 형식 지정자
#   {값:.2f}   → 소수점 2자리
#   {값:6.2f}  → 전체 6자리, 소수점 2자리 (우측 정렬)
#   {값:03d}   → 정수를 3자리로, 빈 자리는 0으로
#   {값:>10}   → 10자리 오른쪽 정렬
#   {값:<10}   → 10자리 왼쪽 정렬
# -------------------------------------------------------

# 예시 데이터
robot_x = 0.2345
robot_y = -0.1078
robot_angle = 92.37
elapsed = 47.3
remaining = 432.7
fixture_letter = "H"

# 완성 예시: 소스 코드에서 볼 수 있는 로그 형식
print(f"[탐색:state_explore] [*] fixture 감지!"
      f" 글자='{fixture_letter}',"
      f" 방향={robot_angle:.1f}°,"
      f" 위치=({robot_x:.3f},{robot_y:.3f})m")

print(f"[타임아웃] 경과={elapsed:.1f}s / 남은시간={remaining:.1f}s")

print()

# -------------------------------------------------------
# TODO 2-2: 아래 형식에 맞는 f-string 출력을 작성하세요.
#
# 1) 위치 리포트 (소수점 2자리):
#    출력 예: "현재 위치: x= 0.23m, y=-0.11m"
#    힌트: {값:6.2f} 를 사용하면 자릿수가 맞춰집니다.

x_values = [0.2345, -0.1078, 1.234, -0.004]
y_values = [-0.1078, 0.5612, -0.98, 0.1]

print("--- 위치 리포트 ---")
for x, y in zip(x_values, y_values):
    # TODO: f-string 으로 "현재 위치: x= 0.23m, y=-0.11m" 형식 출력
    pass  # 이 줄을 지우고 print(f"...") 로 교체하세요

print()

# 2) 진행률 바 만들기:
#    출력 예: "탐색률: [████████░░] 80%"
#    힌트: "█" * filled + "░" * empty

def progress_bar(percent, width=10):
    # TODO: percent (0~100) 를 받아서 진행률 바 문자열을 반환하세요
    pass

# 테스트
for pct in [0, 25, 50, 75, 100]:
    bar = progress_bar(pct)
    if bar:
        print(f"탐색률: {bar} {pct}%")

print()

# -------------------------------------------------------
# TODO 2-3: 로봇 상태 요약 함수를 작성하세요.
#
# def status_report(x, y, angle, elapsed, remaining, victims_found):
#     여러 줄의 f-string 으로 아래 형식을 출력:
#     ┌─────────────────────────┐
#     │ 로봇 상태 리포트         │
#     │ 위치: (0.234, -0.107)m  │
#     │ 방향: 92.4°             │
#     │ 경과: 47.3초 / 남은: 432.7초 │
#     │ 발견 조난자: 3명         │
#     └─────────────────────────┘
# -------------------------------------------------------

def status_report(x, y, angle, elapsed, remaining, victims_found):
    # TODO: 위 형식으로 출력하세요
    pass

status_report(0.234, -0.107, 92.4, 47.3, 432.7, 3)


print("=" * 50)
print("실습 3: src 모듈 구조 파악")
print("=" * 50)

# -------------------------------------------------------
# 아래 코드는 src/ 폴더의 .py 파일 목록을 출력합니다.
# 실행해서 실제 모듈 구조를 확인해보세요.
# -------------------------------------------------------

import os

src_dir = os.path.join(os.path.dirname(__file__), "..", "src")
src_dir = os.path.abspath(src_dir)

if os.path.exists(src_dir):
    print(f"\nsrc/ 폴더 구조:")
    for root, dirs, files in os.walk(src_dir):
        # __pycache__ 폴더 제외
        dirs[:] = [d for d in dirs if d != "__pycache__"]

        level = root.replace(src_dir, "").count(os.sep)
        indent = "  " * level
        folder_name = os.path.basename(root)
        if level == 0:
            print(f"{indent}src/")
        else:
            print(f"{indent}{folder_name}/")

        for f in sorted(files):
            if f.endswith(".py"):
                subindent = "  " * (level + 1)
                print(f"{subindent}{f}")
else:
    print("src 폴더를 찾을 수 없습니다. 경로를 확인하세요.")

print()
print("""
위 구조를 보고 확인하세요:

Q1. 'from mapping.mapper import Mapper' 는
    어느 폴더의 어느 파일에서 무엇을 가져오나요?
    답: mapping 폴더 / mapper.py 파일 / Mapper 클래스

Q2. 'from flags import SHOW_DEBUG' 는?
    답: ___________

Q3. 'from agent.pathfinding.pathfinder import PathFinder' 는?
    답: ___________
""")
