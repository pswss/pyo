"""
3회차 실습 — @property, 상속, 이름 규칙
실행: python study/03_practice.py
"""

print("=" * 50)
print("실습 1: 이름 규칙 확인")
print("=" * 50)

# -------------------------------------------------------
# [개념]
#   self.변수    → 공개: 외부에서 자유롭게 접근
#   self._변수   → 비공개 관례: "직접 쓰지 마세요" 신호
#   self.__변수  → 강제 비공개: 파이썬이 이름을 바꿔서 차단
# -------------------------------------------------------

class RobotInternal:
    def __init__(self):
        self.status = "ready"          # 공개
        self._internal_log = []        # 관례적 비공개
        self.__secret_key = "XK-47"   # 강제 비공개

    def add_log(self, msg):
        self._internal_log.append(msg)

    def get_status(self):
        return f"상태: {self.status}, 로그 수: {len(self._internal_log)}"

r = RobotInternal()

# 공개 변수 — 직접 접근 가능
print(f"공개 접근: {r.status}")

# 관례적 비공개 — 파이썬이 막지는 않지만 관례상 쓰지 말 것
r._internal_log.append("직접 접근 (권장 안 함)")
print(f"관례적 비공개 접근: {r._internal_log}")

# 강제 비공개 — 직접 접근 시 오류
try:
    print(r.__secret_key)
except AttributeError as e:
    print(f"강제 비공개 접근 실패: {e}")

# 내부 이름 변환 후에는 접근 가능하지만 절대 하지 말 것
print(f"변환된 이름으로: {r._RobotInternal__secret_key}  ← 이렇게 하면 안 됨!")


print()
print("=" * 50)
print("실습 2: @property 구현")
print("=" * 50)

# -------------------------------------------------------
# [개념]
#   @property        → 읽기 (robot.x)
#   @x.setter        → 쓰기 (robot.x = 0.5) + 유효성 검사
#   setter 없으면     → 읽기 전용 (쓰기 시도하면 AttributeError)
# -------------------------------------------------------

# 완성 예시: 속도 제한이 있는 로봇
class SafeRobot:
    """속도와 위치에 유효성 검사가 있는 로봇"""

    FIELD_SIZE = 1.2   # 경기장 크기 (미터)

    def __init__(self, name):
        self.name = name
        self._x = 0.0
        self._y = 0.0
        self._speed = 0.0

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        if not (-self.FIELD_SIZE <= value <= self.FIELD_SIZE):
            print(f"  [!] x={value:.2f} 는 경기장 밖입니다. 무시됨.")
            return
        self._x = value

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        if not (-self.FIELD_SIZE <= value <= self.FIELD_SIZE):
            print(f"  [!] y={value:.2f} 는 경기장 밖입니다. 무시됨.")
            return
        self._y = value

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, value):
        self._speed = max(0.0, min(1.0, value))   # 0~1 범위로 클리핑

    @property
    def position(self):
        """(x, y) 튜플 — 읽기 전용 (setter 없음)"""
        return (self._x, self._y)

    @property
    def distance_from_origin(self):
        """출발점까지 거리 — 계산값 (읽기 전용)"""
        import math
        return math.sqrt(self._x ** 2 + self._y ** 2)

bot = SafeRobot("테스트봇")

bot.x = 0.5
bot.y = 0.3
bot.speed = 0.75
print(f"위치: {bot.position}, 속도: {bot.speed:.2f}")
print(f"출발점까지 거리: {bot.distance_from_origin:.3f}m")

bot.x = 5.0       # 범위 초과 → 경고, 변경 안 됨
bot.speed = 2.0   # 범위 초과 → 1.0 으로 클리핑
print(f"클리핑 후 속도: {bot.speed:.2f}")

# 읽기 전용 property 에 쓰기 시도
try:
    bot.position = (1, 1)
except AttributeError as e:
    print(f"읽기 전용 접근 실패: {e}")


print()

# -------------------------------------------------------
# TODO 3-1: BatteryRobot 클래스를 완성하세요.
#
# 요구사항:
#   - __init__(self, capacity_wh): 배터리 용량(Wh) 초기화
#     self._charge = capacity_wh  (현재 충전량)
#     self._capacity = capacity_wh
#
#   - charge (property): 현재 충전량 반환
#   - charge (setter): 0 ~ capacity 범위로 클리핑
#
#   - percent (property, 읽기 전용):
#     현재 충전량을 백분율로 반환 (0 ~ 100)
#
#   - is_low (property, 읽기 전용):
#     충전량이 20% 미만이면 True
#
#   - consume(wh) 메서드:
#     충전량에서 wh 를 빼되, 0 미만이 되면 0으로 처리
#     배터리 방전 시 "배터리 방전!" 출력
# -------------------------------------------------------

class BatteryRobot:
    def __init__(self, capacity_wh):
        pass  # TODO

    # TODO: 나머지 property 와 메서드를 구현하세요


# 구현 후 아래 주석을 해제해서 테스트하세요:
# battery = BatteryRobot(100)
# print(f"초기 충전: {battery.percent:.0f}%")
# battery.consume(30)
# print(f"30Wh 소비 후: {battery.percent:.0f}%, 낮음={battery.is_low}")
# battery.consume(60)   # 방전 → "배터리 방전!" 출력


print("=" * 50)
print("실습 3: 상속 구현")
print("=" * 50)

# -------------------------------------------------------
# [개념]
#   class 자식(부모):
#       def __init__(self, ...):
#           super().__init__(...)  ← 부모 초기화 먼저
#
#   오버라이드: 자식이 부모와 같은 이름의 메서드를 재정의
# -------------------------------------------------------

# 공통 베이스 클래스
class SubAgent:
    """탐색 서브에이전트의 공통 인터페이스 (src/agent/agent_interface.py 참고)"""

    def __init__(self, name):
        self.name = name
        self._target = None

    def update(self):
        """목표 위치를 재계산합니다 (자식 클래스에서 구현)."""
        raise NotImplementedError(f"{self.name}.update() 가 구현되지 않았습니다")

    def get_target(self):
        return self._target

    def has_target(self):
        return self._target is not None

    def describe(self):
        status = f"목표=({self._target[0]:.2f},{self._target[1]:.2f})" \
                 if self.has_target() else "목표=없음"
        return f"[{self.name}] {status}"


# 완성 예시: 직선 이동 에이전트
class GoStraightAgent(SubAgent):
    def __init__(self, direction_x, direction_y, step=0.12):
        super().__init__("직선이동")     # 부모 __init__ 에 이름 전달
        self._dir_x = direction_x
        self._dir_y = direction_y
        self._step = step
        self._current_x = 0.0
        self._current_y = 0.0

    def update(self):
        self._current_x += self._dir_x * self._step
        self._current_y += self._dir_y * self._step
        self._target = (self._current_x, self._current_y)

# 테스트
go = GoStraightAgent(1, 0)  # x 방향으로 직선
for _ in range(3):
    go.update()
    print(f"  {go.describe()}")


print()

# -------------------------------------------------------
# TODO 3-2: 아래 두 클래스를 SubAgent 를 상속받아 구현하세요.
#
# 1) ReturnToStartAgent
#    - __init__(self): super().__init__("귀환") 호출
#    - update(self): self._target = (0.0, 0.0) 으로 설정
#      (출발점으로 돌아가는 가장 단순한 구현)
#
# 2) SpiralSearchAgent
#    - __init__(self, radius_step=0.12): 나선형 탐색
#    - update(self): 매 호출마다 반경을 radius_step 씩 늘리면서
#      원 위의 점을 목표로 설정 (각도도 30도씩 증가)
#      힌트: import math, math.cos(각도_rad), math.sin(각도_rad)
# -------------------------------------------------------

# 여기에 코드를 작성하세요 ↓


# 구현 후 테스트:
# agents = [GoStraightAgent(0, 1), ReturnToStartAgent()]
# for agent in agents:
#     for _ in range(3):
#         agent.update()
#     print(agent.describe())


print()
print("=" * 50)
print("실습 4: rescue_robot.py property 분석")
print("=" * 50)

print("""
src/rescue_robot.py 를 열고 아래 표를 채워보세요:

property 이름        | 반환 타입 | 내부에서 호출하는 것
---------------------|-----------|---------------------
x                   | float     | self._robot.position.x
y                   | ?         | ?
direction           | ?         | ?
elapsed_time        | ?         | ?
remaining_time      | ?         | ?
is_time_almost_up   | ?         | ?
is_at_start         | ?         | ?
victim_visible      | ?         | ?
exploration_complete| ?         | ?

읽기 전용(setter 없음)인 것: 전부 다 (rescue_robot.py 는 모두 읽기 전용)
""")
