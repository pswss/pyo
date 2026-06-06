"""
4회차 실습 — 타입 힌트, 람다, None 패턴
실행: python study/04_practice.py
"""

print("=" * 50)
print("실습 1: 타입 힌트 읽기")
print("=" * 50)

# -------------------------------------------------------
# [개념] 타입 힌트는 실행에 영향 없이 코드 의도를 전달합니다.
#   def func(x: float, y: float) -> bool:
#       ↑ x 는 float, y 는 float, 반환값은 bool
# -------------------------------------------------------

# 아래 함수들을 보고 어떤 타입인지 확인하세요.

def calculate_distance(x1: float, y1: float,
                       x2: float, y2: float) -> float:
    """두 점 사이의 거리를 계산합니다."""
    import math
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def is_in_range(position: tuple, center: tuple, radius: float) -> bool:
    """position 이 center 로부터 radius 이내인지 확인합니다."""
    x, y = position
    cx, cy = center
    return calculate_distance(x, y, cx, cy) <= radius

def find_closest(positions: list, target: tuple):
    """positions 중 target 에 가장 가까운 위치를 반환합니다.
    positions 가 비어있으면 None 을 반환합니다."""
    if not positions:
        return None
    return min(positions, key=lambda p: calculate_distance(p[0], p[1], target[0], target[1]))

# 테스트
p1 = (0.0, 0.0)
p2 = (0.3, 0.4)
print(f"거리: {calculate_distance(*p1, *p2):.3f}m")   # 0.500

candidates = [(0.1, 0.1), (0.5, 0.5), (0.9, 0.2)]
closest = find_closest(candidates, (0.2, 0.2))
print(f"가장 가까운 위치: {closest}")

empty_result = find_closest([], (0.0, 0.0))
print(f"빈 리스트 결과: {empty_result}")   # None

print()

# -------------------------------------------------------
# TODO 4-1: 아래 함수에 타입 힌트를 추가하세요.
#
# 힌트:
#   - 각도는 float
#   - 이름은 str
#   - bool 반환
#   - Optional[str]: str 또는 None 이 될 수 있음
# -------------------------------------------------------

# from typing import Optional   ← 이 줄의 주석을 해제하세요

def normalize_angle(angle):   # TODO: 타입 힌트 추가
    """각도를 0~360 범위로 정규화합니다."""
    return angle % 360

def classify_victim(letter):  # TODO: 타입 힌트 추가
    """
    letter 가 유효한 조난자 글자면 설명 반환, 아니면 None 반환.
    유효: H(열), S(연기), U(갇힘), P(독성), F(가연성), C(부식), O(과산화물)
    """
    descriptions = {
        "H": "열",
        "S": "연기",
        "U": "갇힘",
        "P": "독성",
        "F": "가연성",
        "C": "부식성",
        "O": "유기 과산화물",
    }
    return descriptions.get(letter, None)


print(f"각도 정규화: {normalize_angle(450)}")          # 90
print(f"조난자 분류 'H': {classify_victim('H')}")      # 열
print(f"조난자 분류 'Z': {classify_victim('Z')}")      # None


print()
print("=" * 50)
print("실습 2: 람다 활용")
print("=" * 50)

# -------------------------------------------------------
# [개념] lambda 매개변수: 반환값
#   sorted, filter, map 에 함수를 전달할 때 주로 사용
# -------------------------------------------------------

# 센서 측정 데이터
readings = [
    {"sensor": "lidar_front",  "value": 0.45, "valid": True},
    {"sensor": "lidar_left",   "value": 0.12, "valid": True},
    {"sensor": "lidar_right",  "value": 2.30, "valid": False},  # 범위 초과
    {"sensor": "lidar_back",   "value": 0.88, "valid": True},
    {"sensor": "camera_front", "value": 0.60, "valid": True},
]

# 완성 예시: 유효한 측정값만 필터링
valid_only = list(filter(lambda r: r["valid"], readings))
print(f"유효한 측정값 수: {len(valid_only)}")

# 완성 예시: 거리값만 추출
distances = list(map(lambda r: r["value"], valid_only))
print(f"거리 목록: {distances}")

# 완성 예시: 가까운 것부터 정렬
sorted_readings = sorted(valid_only, key=lambda r: r["value"])
print(f"가장 가까운 센서: {sorted_readings[0]['sensor']} ({sorted_readings[0]['value']}m)")


print()

# -------------------------------------------------------
# TODO 4-2: 람다를 사용해서 아래 작업을 완성하세요.
# -------------------------------------------------------

# 조난자 목록
victims = [
    {"letter": "H", "x": 0.36, "y": 0.12, "reported": False},
    {"letter": "S", "x": 0.12, "y": 0.48, "reported": True},
    {"letter": "U", "x": 0.60, "y": 0.36, "reported": False},
    {"letter": "F", "x": 0.24, "y": 0.60, "reported": True},
]

# TODO: 아직 보고 안 된(reported=False) 조난자만 필터링
unreported = None  # list(filter(lambda ..., victims))
# print(f"미보고 조난자: {[v['letter'] for v in unreported]}")

# TODO: y 좌표 기준으로 오름차순 정렬
by_y = None  # sorted(victims, key=lambda ...)
# print(f"y순 정렬: {[f\"{v['letter']}@{v['y']}\" for v in by_y]}")

# TODO: 로봇 위치 (0.3, 0.3) 에서 가장 가까운 미보고 조난자 찾기
import math
robot_pos = (0.3, 0.3)
# closest_unreported = min(unreported, key=lambda v: ...)
# if closest_unreported:
#     print(f"가장 가까운 미보고 조난자: {closest_unreported['letter']}")


print()
print("=" * 50)
print("실습 3: None 패턴 완전 정복")
print("=" * 50)

# -------------------------------------------------------
# [개념]
#   None = 값이 없음을 나타내는 특별한 값
#   체크: if x is None / if x is not None
#   Short-circuit: None 이면 즉시 return
# -------------------------------------------------------

# 완성 예시: None 을 반환할 수 있는 함수를 안전하게 사용
class SensorManager:
    def __init__(self):
        self._lidar_value = None     # 아직 측정 안 됨
        self._camera_image = None    # 아직 캡처 안 됨
        self._start_pos = None       # 아직 시작 위치 미등록

    def update_lidar(self, value: float):
        self._lidar_value = value

    def register_start(self, x: float, y: float):
        self._start_pos = (x, y)

    def get_distance(self):
        """라이다 값 반환. 아직 측정 안 됐으면 None."""
        return self._lidar_value

    def is_close_to_wall(self, threshold=0.15):
        """장애물이 가까우면 True. 측정값이 없으면 False."""
        dist = self.get_distance()
        if dist is None:         # None 체크 먼저!
            return False
        return dist < threshold

    def distance_to_start(self, current_x, current_y):
        """시작점까지 거리. 시작점 미등록 시 None."""
        if self._start_pos is None:    # short-circuit
            return None
        sx, sy = self._start_pos
        return math.sqrt((current_x - sx)**2 + (current_y - sy)**2)

sm = SensorManager()

# 아직 측정값 없는 상태
print(f"측정값: {sm.get_distance()}")           # None
print(f"벽 근접: {sm.is_close_to_wall()}")      # False (None-safe)
print(f"시작점 거리: {sm.distance_to_start(0.3, 0.3)}")  # None

sm.update_lidar(0.08)
sm.register_start(0.0, 0.0)

print(f"측정값: {sm.get_distance()}")           # 0.08
print(f"벽 근접: {sm.is_close_to_wall()}")      # True
print(f"시작점 거리: {sm.distance_to_start(0.3, 0.3):.3f}m")


print()

# -------------------------------------------------------
# TODO 4-3: 아래 PathPlanner 클래스의 None 처리를 완성하세요.
# -------------------------------------------------------

class PathPlanner:
    def __init__(self):
        self._current_path = None    # A* 경로 (없을 수 있음)
        self._target = None          # 목표 위치 (없을 수 있음)

    def set_target(self, x: float, y: float):
        self._target = (x, y)
        self._current_path = None    # 목표 바뀌면 경로 초기화

    def compute_path(self, start_x: float, start_y: float):
        """단순화된 경로 계산 (직선 경로 생성)."""
        if self._target is None:     # TODO: 목표 없으면 바로 return
            pass
        # 직선으로 10단계 경로 생성
        tx, ty = self._target
        steps = 10
        self._current_path = [
            (start_x + (tx - start_x) * i / steps,
             start_y + (ty - start_y) * i / steps)
            for i in range(steps + 1)
        ]

    def get_next_waypoint(self):
        """다음 경유점 반환. 경로가 없거나 완료되면 None."""
        if self._current_path is None:    # TODO: None 체크
            return None
        if len(self._current_path) == 0:  # TODO: 빈 리스트 체크
            return None
        return self._current_path.pop(0)

    def has_path(self) -> bool:
        """경로가 존재하고 아직 남아있으면 True."""
        # TODO: self._current_path 가 None 이 아니고
        #       길이가 0보다 클 때 True 반환
        pass


# 테스트
planner = PathPlanner()

# 목표 없는 상태
planner.compute_path(0.0, 0.0)   # 목표 없으므로 아무것도 안 함
print(f"경로 있음: {planner.has_path()}")  # False

planner.set_target(0.6, 0.6)
planner.compute_path(0.0, 0.0)
print(f"경로 있음: {planner.has_path()}")  # True

# 경유점 이동
for _ in range(3):
    wp = planner.get_next_waypoint()
    if wp is not None:
        print(f"  → 경유점: ({wp[0]:.2f}, {wp[1]:.2f})")


print()
print("=" * 50)
print("실습 4: 소스 코드 분석")
print("=" * 50)

print("""
src/executor/executor.py 에서 None 과 람다 패턴을 찾아보세요:

1. state_explore 메서드에서 None 체크:
   - 감지된 letter 가 None 이면 어떻게 처리하나요?
   - (힌트: detected_letter is None → continue)

2. state_report_fixture 에서:
   - letter_to_report 가 None 일 때 if 문이 어떻게 동작하나요?
   - (힌트: if self.letter_to_report is not None: 블록 안)

3. src/agent/subagents/go_to_non_discovered/go_to_non_discovered_position_finder.py
   에서 람다를 찾아보세요:
   - found_function 의 람다는 무엇을 확인하나요?
   - traversable_function 의 람다는 무엇을 확인하나요?
""")
