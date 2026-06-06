"""
02_robot_config.py — 로봇 설정값 모듈
src/flags.py 와 동일한 역할을 하는 예제 모듈입니다.

다른 파일에서 사용하는 법:
    from 02_robot_config import TILE_SIZE, format_position
    # 참고: 파일명이 숫자로 시작하면 import 가 어렵습니다.
    # 실제 프로젝트에서는 robot_config.py 처럼 영문자로 시작하세요.
"""

# -------------------------------------------------------
# 상수 (Constants)
# 값이 바뀌지 않는 설정값들을 모아두면 관리가 쉽다.
# flags.py 의 SHOW_LIVE_MAP, DO_SLOW_DOWN 등도 이런 패턴이다.
# -------------------------------------------------------

TILE_SIZE = 0.12          # 타일 크기 (미터) — 미로 한 칸의 실제 크기
MAX_SPEED = 1.0           # 최대 속도 (0.0 ~ 1.0 정규화된 값)
TIME_LIMIT_SEC = 8 * 60   # 대회 제한 시간: 8분 = 480초
SHOW_LOG = True           # 로그 출력 여부

# 조난자 글자 목록 — 대회 규정상 7가지
VALID_LETTERS = ["H", "S", "U", "P", "F", "C", "O"]


def format_position(x, y):
    """
    좌표를 '(x, y)m' 형식 문자열로 반환한다.

    executor.py 의 f-string 로그와 동일한 형식:
      f"위치=({self.robot.position.x:.3f},{self.robot.position.y:.3f})m"
    """
    return f"({x:.3f}, {y:.3f})m"


def format_time(seconds):
    """
    초를 'M분 SS초' 형식으로 반환한다.
    예: 94.5 → '1분 34초'
    """
    minutes = int(seconds) // 60
    secs = int(seconds) % 60
    return f"{minutes}분 {secs:02d}초"


def clamp(value, lo, hi):
    """
    value 를 [lo, hi] 범위로 제한(클리핑)해서 반환한다.

    max(lo, min(hi, value)) 패턴을 함수로 감싼 것.
    rescue_robot.py 의 speed 처리 등에 자주 쓰이는 패턴이다.
    """
    return max(lo, min(hi, value))
