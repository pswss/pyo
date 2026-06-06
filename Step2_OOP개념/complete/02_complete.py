"""
2회차 완성본 — 모듈, 임포트, f-string
실행: python study/complete/02_complete.py
"""

import sys
import os

# study/complete 폴더를 파이썬 경로에 추가해야 같은 폴더의 모듈을 임포트할 수 있다.
# 실제 프로젝트에서는 PYTHONPATH 환경 변수나 패키지 구조로 처리한다.
sys.path.insert(0, os.path.dirname(__file__))

print("=" * 50)
print("실습 1: 설정 파일(모듈) 만들기")
print("=" * 50)

# -------------------------------------------------------
# [완성] TODO 2-1: 설정 모듈 임포트
#
# from 모듈이름 import 이름1, 이름2  형태로 필요한 것만 가져온다.
# src/executor/executor.py 의 첫 부분과 같은 패턴이다:
#   from flags import SHOW_DEBUG, DO_SLOW_DOWN, SLOW_DOWN_S
# -------------------------------------------------------

from robot_config_02 import (
    TILE_SIZE,
    MAX_SPEED,
    TIME_LIMIT_SEC,
    SHOW_LOG,
    format_position,
    format_time,
    clamp,
)

print(f"타일 크기: {TILE_SIZE}m")
print(f"최대 속도: {MAX_SPEED}")
print(f"제한 시간: {TIME_LIMIT_SEC}초 ({TIME_LIMIT_SEC // 60}분)")
print(f"로그 표시: {SHOW_LOG}")
print(f"현재 위치: {format_position(0.12, 0.48)}")
print(f"경과 시간: {format_time(94.5)}")
print(f"속도 클리핑 (2.0 → {clamp(2.0, 0.0, 1.0)})")


print()
print("=" * 50)
print("실습 2: f-string 연습")
print("=" * 50)

# -------------------------------------------------------
# [개념] f-string 형식 지정자
#   {값:.2f}   → 소수점 2자리
#   {값:6.2f}  → 전체 6자리, 소수점 2자리 (오른쪽 정렬, 앞에 공백)
#   {값:+.2f}  → 부호 항상 표시 (+0.23, -0.11)
#   {값:03d}   → 정수를 3자리로, 빈 자리는 0으로
# -------------------------------------------------------

robot_x = 0.2345
robot_y = -0.1078
robot_angle = 92.37
elapsed = 47.3
remaining = 432.7
fixture_letter = "H"

# 소스 코드와 동일한 로그 형식
print(f"[탐색:state_explore] [*] fixture 감지!"
      f" 글자='{fixture_letter}',"
      f" 방향={robot_angle:.1f}도,"
      f" 위치=({robot_x:.3f},{robot_y:.3f})m")

print(f"[타임아웃] 경과={elapsed:.1f}s / 남은시간={remaining:.1f}s")
print()

# -------------------------------------------------------
# [완성] TODO 2-2 (1): 위치 리포트 — 자릿수 맞춰 출력
# -------------------------------------------------------

x_values = [0.2345, -0.1078, 1.234, -0.004]
y_values = [-0.1078, 0.5612, -0.98, 0.1]

print("--- 위치 리포트 ---")
for x, y in zip(x_values, y_values):
    # {:6.2f}: 전체 너비 6, 소수점 2자리, 우측 정렬
    # 예: 0.2345 → " 0.23", -0.1078 → "-0.11"
    # 너비를 맞추면 여러 줄이 세로로 정렬되어 읽기 쉽다.
    print(f"현재 위치: x={x:6.2f}m, y={y:6.2f}m")

print()

# -------------------------------------------------------
# [완성] TODO 2-2 (2): 진행률 바 만들기
# -------------------------------------------------------

def progress_bar(percent, width=10):
    """
    percent (0~100) 를 받아서 '[████████░░] 80%' 같은 문자열을 반환한다.

    filled = 채워진 칸 수 = width * (percent / 100) 를 반올림한 정수
    empty  = 빈 칸 수     = width - filled

    문자열 반복: "█" * 3 → "███"
    """
    # round() 로 반올림해서 정수로 변환
    filled = round(width * percent / 100)
    empty = width - filled
    bar = "[" + "##" * filled + ".." * empty + "]"
    return bar

for pct in [0, 25, 50, 75, 100]:
    bar = progress_bar(pct)
    print(f"탐색률: {bar} {pct:3d}%")

print()

# -------------------------------------------------------
# [완성] TODO 2-3: 로봇 상태 요약 함수
# -------------------------------------------------------

def status_report(x, y, angle, elapsed, remaining, victims_found):
    """
    로봇 현재 상태를 보기 좋은 박스 형태로 출력한다.

    f-string 안에서 줄바꿈(\n)과 탭(\t) 도 그대로 사용할 수 있다.
    여러 줄 f-string 을 만들 때는 ''' 또는 "" 세 개를 사용한다.

    src/executor/executor.py 의 print 로그를 조합하면
    이런 리포트를 직접 만들 수 있다.
    """
    # 시간 남은 비율로 긴급도 표시
    urgency = "[!] 시간 부족!" if remaining < 60 else "정상"

    # 줄마다 f-string 으로 만들어서 join 으로 합치는 방법
    lines = [
        "+" + "-" * 32 + "+",
        f"|  {'로봇 상태 리포트':^28}  |",
        "+" + "-" * 32 + "+",
        f"|  위치   : {format_position(x, y):<22}|",
        f"|  방향   : {angle:.1f}도{'':<23}|",
        f"|  경과   : {elapsed:.1f}초  /  남은: {remaining:.1f}초{'':<7}|",
        f"|  상태   : {urgency:<22}|",
        f"|  조난자 : {victims_found}명 발견{'':<21}|",
        "+" + "-" * 32 + "+",
    ]
    for line in lines:
        print(line)

status_report(0.234, -0.107, 92.4, 47.3, 432.7, 3)
print()
status_report(0.960, 0.720, 180.0, 452.1, 27.9, 5)   # 시간 부족 케이스


print()
print("=" * 50)
print("실습 3: src 모듈 구조 파악")
print("=" * 50)

# 현재 파일 기준으로 src 폴더 경로를 계산
src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "src"))

if os.path.exists(src_dir):
    print(f"\nsrc/ 폴더 구조:")
    for root, dirs, files in os.walk(src_dir):
        dirs[:] = sorted([d for d in dirs if d != "__pycache__"])
        level = root.replace(src_dir, "").count(os.sep)
        indent = "  " * level
        folder_name = os.path.basename(root)
        if level == 0:
            print(f"{indent}src/")
        else:
            print(f"{indent}{folder_name}/")
        for f in sorted(files):
            if f.endswith(".py"):
                print(f"{'  ' * (level + 1)}{f}")

print()
print("""
[정답]
Q1. 'from mapping.mapper import Mapper'
    → mapping 폴더 / mapper.py 파일 / Mapper 클래스

Q2. 'from flags import SHOW_DEBUG'
    → src 폴더 직속 / flags.py 파일 / SHOW_DEBUG 변수

Q3. 'from agent.pathfinding.pathfinder import PathFinder'
    → agent 폴더 / pathfinding 하위 폴더 / pathfinder.py / PathFinder 클래스
""")
