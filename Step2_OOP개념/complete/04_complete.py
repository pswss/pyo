"""
4회차 완성본 — 타입 힌트, 람다, None 패턴
실행: python study/complete/04_complete.py
"""

import math
from typing import Optional, List, Tuple

print("=" * 50)
print("실습 1: 타입 힌트 읽기")
print("=" * 50)

def calculate_distance(x1: float, y1: float,
                       x2: float, y2: float) -> float:
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def is_in_range(position: tuple, center: tuple, radius: float) -> bool:
    x, y = position
    cx, cy = center
    return calculate_distance(x, y, cx, cy) <= radius

def find_closest(positions: list, target: tuple):
    if not positions:
        return None
    return min(positions, key=lambda p: calculate_distance(p[0], p[1], target[0], target[1]))

p1 = (0.0, 0.0)
p2 = (0.3, 0.4)
print(f"거리: {calculate_distance(*p1, *p2):.3f}m")

candidates = [(0.1, 0.1), (0.5, 0.5), (0.9, 0.2)]
print(f"가장 가까운 위치: {find_closest(candidates, (0.2, 0.2))}")
print(f"빈 리스트 결과:   {find_closest([], (0.0, 0.0))}")

print()

# -------------------------------------------------------
# [완성] TODO 4-1: 타입 힌트 추가
#
# 타입 힌트는 실행에 영향을 주지 않는다.
# 코드를 읽는 사람에게 "이 매개변수는 float 를 기대해요" 라고 알려주는 문서 역할이다.
# Optional[str] = str 또는 None 이 올 수 있다는 의미.
# -------------------------------------------------------

def normalize_angle(angle: float) -> float:
    """
    각도를 0~360 범위로 정규화한다.

    % (나머지 연산):  450 % 360 = 90,  -30 % 360 = 330
    파이썬의 % 는 결과가 항상 양수이므로 음수 각도도 올바르게 처리된다.
    """
    return angle % 360

def classify_victim(letter: str) -> Optional[str]:
    """
    letter 가 유효한 조난자 글자면 한글 설명을 반환, 아니면 None 반환.

    반환 타입이 Optional[str] 인 이유:
    - 유효한 글자면 str ("열", "연기" 등)
    - 유효하지 않은 글자면 None
    → 둘 다 될 수 있으므로 Optional[str]

    src/fixture_clasification.py 의 classify_fixture() 도
    already_detected 일 때 None 을 반환하는 같은 패턴이다.
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
    # dict.get(key, default): key 없으면 default 반환
    # default 를 생략하면 None 이 기본값 → None 을 명시하지 않아도 됨
    return descriptions.get(letter)

print(f"각도 정규화 450도: {normalize_angle(450)}")
print(f"각도 정규화 -30도: {normalize_angle(-30)}")
print(f"조난자 분류 'H':   {classify_victim('H')}")
print(f"조난자 분류 'Z':   {classify_victim('Z')}")


print()
print("=" * 50)
print("실습 2: 람다 활용")
print("=" * 50)

readings = [
    {"sensor": "lidar_front",  "value": 0.45, "valid": True},
    {"sensor": "lidar_left",   "value": 0.12, "valid": True},
    {"sensor": "lidar_right",  "value": 2.30, "valid": False},
    {"sensor": "lidar_back",   "value": 0.88, "valid": True},
    {"sensor": "camera_front", "value": 0.60, "valid": True},
]

valid_only = list(filter(lambda r: r["valid"], readings))
distances = list(map(lambda r: r["value"], valid_only))
sorted_readings = sorted(valid_only, key=lambda r: r["value"])
print(f"유효 측정값 수: {len(valid_only)}")
print(f"거리 목록: {distances}")
print(f"가장 가까운 센서: {sorted_readings[0]['sensor']} ({sorted_readings[0]['value']}m)")

print()

# -------------------------------------------------------
# [완성] TODO 4-2: 람다를 사용한 조난자 데이터 처리
# -------------------------------------------------------

victims = [
    {"letter": "H", "x": 0.36, "y": 0.12, "reported": False},
    {"letter": "S", "x": 0.12, "y": 0.48, "reported": True},
    {"letter": "U", "x": 0.60, "y": 0.36, "reported": False},
    {"letter": "F", "x": 0.24, "y": 0.60, "reported": True},
]

# 1) 미보고 조난자만 필터링
# lambda v: not v["reported"] → reported 가 False 인 것만 통과
unreported = list(filter(lambda v: not v["reported"], victims))
print(f"미보고 조난자: {[v['letter'] for v in unreported]}")
# 출력: ['H', 'U']

# 2) y 좌표 기준 오름차순 정렬
# key=lambda v: v["y"] → 딕셔너리에서 "y" 값을 정렬 기준으로 사용
by_y = sorted(victims, key=lambda v: v["y"])
labels_by_y = [f"{v['letter']}@y={v['y']:.2f}" for v in by_y]
print(f"y 좌표 순: {labels_by_y}")
# 출력: H@y=0.12, U@y=0.36, S@y=0.48, F@y=0.60

# 3) 로봇 위치에서 가장 가까운 미보고 조난자
robot_pos = (0.3, 0.3)

# min() 에 key 람다를 전달하면 각 항목의 거리를 계산해서 가장 작은 것을 반환
# unreported 가 비어있으면 min() 이 ValueError 를 발생시키므로 먼저 확인
if unreported:
    closest = min(
        unreported,
        key=lambda v: math.sqrt((v["x"] - robot_pos[0])**2
                                + (v["y"] - robot_pos[1])**2)
    )
    dist = math.sqrt((closest["x"] - robot_pos[0])**2
                     + (closest["y"] - robot_pos[1])**2)
    print(f"가장 가까운 미보고 조난자: {closest['letter']}"
          f" @ ({closest['x']:.2f},{closest['y']:.2f}), 거리={dist:.3f}m")


print()
print("=" * 50)
print("실습 3: None 패턴 완전 정복")
print("=" * 50)

class SensorManager:
    def __init__(self):
        self._lidar_value = None
        self._camera_image = None
        self._start_pos = None

    def update_lidar(self, value: float):
        self._lidar_value = value

    def register_start(self, x: float, y: float):
        self._start_pos = (x, y)

    def get_distance(self) -> Optional[float]:
        return self._lidar_value

    def is_close_to_wall(self, threshold=0.15) -> bool:
        dist = self.get_distance()
        if dist is None:
            return False
        return dist < threshold

    def distance_to_start(self, current_x: float,
                          current_y: float) -> Optional[float]:
        if self._start_pos is None:
            return None
        sx, sy = self._start_pos
        return math.sqrt((current_x - sx)**2 + (current_y - sy)**2)

sm = SensorManager()
print(f"측정값: {sm.get_distance()}")
print(f"벽 근접: {sm.is_close_to_wall()}")
print(f"시작점 거리: {sm.distance_to_start(0.3, 0.3)}")

sm.update_lidar(0.08)
sm.register_start(0.0, 0.0)
print(f"측정값: {sm.get_distance()}")
print(f"벽 근접: {sm.is_close_to_wall()}")
print(f"시작점 거리: {sm.distance_to_start(0.3, 0.3):.3f}m")


print()

# -------------------------------------------------------
# [완성] TODO 4-3: PathPlanner None 처리 완성
# -------------------------------------------------------

class PathPlanner:
    """
    A* 경로 계획기 단순화 버전.

    src/agent/pathfinding/pathfinder.py 의 PathFinder 와 같은 역할이다.
    이 클래스는 직선 경로만 생성하는 단순한 버전이다.
    """

    def __init__(self):
        self._current_path: Optional[List[Tuple]] = None  # 경로 없음 = None
        self._target: Optional[Tuple[float, float]] = None

    def set_target(self, x: float, y: float):
        self._target = (x, y)
        self._current_path = None   # 목표가 바뀌면 기존 경로를 초기화

    def compute_path(self, start_x: float, start_y: float):
        """
        목표까지 직선 경로를 10단계로 생성한다.
        목표가 없으면 아무것도 하지 않는다.

        Short-circuit(조기 반환) 패턴:
          if ... is None: return
        → 이후 코드를 실행하지 않고 바로 함수를 빠져나간다.
          rescue_robot.py 의 go_to_start() 에서도 동일하게 사용된다.
        """
        # 목표가 없으면 경로를 만들 수 없다 → 즉시 반환
        if self._target is None:
            print("  [!] compute_path: 목표가 없어서 경로를 만들 수 없습니다.")
            return

        tx, ty = self._target
        steps = 10
        # 리스트 컴프리헨션으로 10개의 (x, y) 좌표 생성
        self._current_path = [
            (start_x + (tx - start_x) * i / steps,
             start_y + (ty - start_y) * i / steps)
            for i in range(steps + 1)
        ]
        print(f"  경로 생성 완료: {len(self._current_path)}개 경유점")

    def get_next_waypoint(self) -> Optional[Tuple[float, float]]:
        """
        다음 경유점을 꺼내서 반환한다.
        경로가 없거나 끝났으면 None 반환.

        list.pop(0): 리스트의 첫 번째 원소를 꺼내면서 제거한다.
        매번 pop(0) 하면 경로를 앞에서부터 하나씩 소비하는 효과다.
        """
        # None 체크: 경로 자체가 없는 경우
        if self._current_path is None:
            return None
        # 빈 리스트 체크: 경로를 다 소비한 경우
        if len(self._current_path) == 0:
            return None
        return self._current_path.pop(0)   # 앞에서 꺼냄

    def has_path(self) -> bool:
        """
        경로가 존재하고 아직 남아있으면 True.

        두 가지를 모두 확인해야 한다:
        1. _current_path 가 None 이 아닌가? (경로가 만들어졌는가)
        2. len > 0 인가? (아직 남은 경유점이 있는가)
        """
        # None 체크를 먼저 해야 한다.
        # _current_path 가 None 일 때 len() 을 호출하면 TypeError 가 난다.
        return self._current_path is not None and len(self._current_path) > 0

print("[PathPlanner 테스트]")
planner = PathPlanner()

# 목표 없는 상태
planner.compute_path(0.0, 0.0)   # → 경고 메시지
print(f"경로 있음: {planner.has_path()}")

planner.set_target(0.6, 0.6)
planner.compute_path(0.0, 0.0)
print(f"경로 있음: {planner.has_path()}")

for i in range(4):
    wp = planner.get_next_waypoint()
    if wp is not None:
        print(f"  경유점 {i+1}: ({wp[0]:.2f}, {wp[1]:.2f})")

print(f"남은 경유점: {len(planner._current_path)}개")
# 나머지 전부 소비
while planner.has_path():
    planner.get_next_waypoint()
print(f"경로 소진 후 has_path: {planner.has_path()}")
print(f"소진 후 get_next_waypoint: {planner.get_next_waypoint()}")  # None


print()
print("=" * 50)
print("[참고] 소스 코드와 연결")
print("=" * 50)
print("""
src/executor/executor.py 의 None 과 람다 패턴:

1. state_explore 의 None 체크 (이번 버그 수정 내용):
   detected_letter = self.fixture_detector.classify_fixture(fixtures[0])
   if detected_letter is None:
       continue   ← already_detected 이면 다음 카메라로 넘어감

2. state_report_fixture 의 None 체크:
   if self.letter_to_report is not None:
       ...   ← 글자가 있을 때만 보고 시퀀스 실행

3. go_to_non_discovered_position_finder.py 의 람다:
   NavigatingBFSAlgorithm(
       found_function=lambda x: x == False,   ← 미탐색 칸 찾기
       traversable_function=lambda x: x == False   ← 통과 가능 칸
   )
   → BFS 알고리즘에 "어느 칸이 목표인지", "어디를 통과할 수 있는지" 를
     람다로 전달하여 알고리즘 코드를 수정하지 않고 동작을 교체한다.
""")
