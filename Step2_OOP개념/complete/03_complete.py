"""
3회차 완성본 — @property, 상속, 이름 규칙
실행: python study/complete/03_complete.py
"""

import math

print("=" * 50)
print("실습 1: 이름 규칙 확인")
print("=" * 50)

# -------------------------------------------------------
# [개념]
#   self.변수    → 공개: 외부에서 자유롭게 접근 가능
#   self._변수   → 관례적 비공개: "내부용"이라는 신호, 강제로 막지는 않음
#   self.__변수  → 강제 비공개: 파이썬이 이름을 바꿔서 외부 직접 접근 차단
# -------------------------------------------------------

class RobotInternal:
    def __init__(self):
        self.status = "ready"          # 공개 — 외부에서 자유롭게 읽고 쓰기 가능
        self._internal_log = []        # 관례적 비공개 — 외부에서 쓰지 않기로 약속
        self.__secret_key = "XK-47"   # 강제 비공개 — 파이썬이 실제로 차단

    def add_log(self, msg):
        self._internal_log.append(msg)

    def get_status(self):
        return f"상태: {self.status}, 로그 수: {len(self._internal_log)}"

r = RobotInternal()

print(f"공개 접근: {r.status}")
r._internal_log.append("직접 접근 (권장 안 함)")
print(f"관례적 비공개 접근: {r._internal_log}")

try:
    print(r.__secret_key)
except AttributeError as e:
    print(f"강제 비공개 접근 실패: {e}")

# 파이썬은 __변수를 _클래스명__변수 로 이름을 바꾸므로 이렇게는 접근됨
# 하지만 절대 이렇게 쓰면 안 된다!
print(f"변환된 이름으로: {r._RobotInternal__secret_key}  ← 이렇게 하면 안 됨!")


print()
print("=" * 50)
print("실습 2: @property 구현")
print("=" * 50)

class SafeRobot:
    """속도와 위치에 유효성 검사가 있는 로봇"""

    FIELD_SIZE = 1.2

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
        self._speed = max(0.0, min(1.0, value))

    @property
    def position(self):
        return (self._x, self._y)

    @property
    def distance_from_origin(self):
        return math.sqrt(self._x ** 2 + self._y ** 2)

bot = SafeRobot("테스트봇")
bot.x = 0.5
bot.y = 0.3
bot.speed = 0.75
print(f"위치: {bot.position}, 속도: {bot.speed:.2f}")
print(f"출발점까지 거리: {bot.distance_from_origin:.3f}m")
bot.x = 5.0
bot.speed = 2.0
print(f"클리핑 후 속도: {bot.speed:.2f}")

try:
    bot.position = (1, 1)
except AttributeError as e:
    print(f"읽기 전용 접근 실패: {e}")


print()

# -------------------------------------------------------
# [완성] TODO 3-1: BatteryRobot 클래스
# -------------------------------------------------------

class BatteryRobot:
    """
    배터리 상태를 관리하는 로봇 클래스.

    실제 Webots 로봇에는 배터리 센서가 있다.
    이 클래스는 그 동작을 시뮬레이션한다.
    """

    def __init__(self, capacity_wh):
        # 용량과 현재 충전량을 별도로 관리
        # → 현재 충전량이 바뀌어도 용량은 항상 고정값으로 남는다
        self._capacity = capacity_wh   # 최대 용량 (변하지 않음)
        self._charge = capacity_wh     # 현재 충전량 (처음엔 꽉 참)

    @property
    def charge(self):
        """현재 충전량(Wh)을 반환한다."""
        return self._charge

    @charge.setter
    def charge(self, value):
        """
        충전량을 설정한다. 0 ~ capacity 범위를 벗어나면 클리핑.

        max(0, min(capacity, value)):
          - capacity 초과하면 capacity 로 낮춤
          - 0 미만이면 0 으로 올림
        """
        self._charge = max(0.0, min(self._capacity, value))

    @property
    def percent(self):
        """
        현재 충전량을 0~100 백분율로 반환한다. (읽기 전용)

        나눗셈 전에 capacity 가 0인지 확인해야 ZeroDivisionError 를 막을 수 있다.
        capacity 가 0 일 일은 없겠지만, 방어적으로 처리하는 게 좋은 습관이다.
        """
        if self._capacity == 0:
            return 0.0
        return (self._charge / self._capacity) * 100

    @property
    def is_low(self):
        """충전량이 20% 미만이면 True. (읽기 전용)"""
        return self.percent < 20   # percent property 를 재활용

    def consume(self, wh):
        """
        충전량에서 wh 를 소비한다.
        방전(0Wh 도달)시 경고 메시지를 출력한다.

        self.charge = self._charge - wh 로 setter 를 호출하면
        setter 안의 클리핑이 자동으로 적용된다.
        즉, 0 미만으로 내려가려 해도 setter 가 0 으로 막아준다.
        """
        new_charge = self._charge - wh
        self.charge = new_charge   # setter 호출 → 0 미만 클리핑 적용
        if self._charge == 0:
            print(f"  [!] 배터리 방전! 로봇이 멈춥니다.")

# 테스트
print("[BatteryRobot 테스트]")
battery = BatteryRobot(100)
print(f"초기 충전: {battery.percent:.0f}%,  낮음={battery.is_low}")

battery.consume(30)
print(f"30Wh 소비: {battery.percent:.0f}%,  낮음={battery.is_low}")

battery.consume(45)
print(f"45Wh 추가: {battery.percent:.0f}%,  낮음={battery.is_low}")

battery.consume(60)   # 나머지 25Wh 소비 후 방전
print(f"방전 후:   {battery.percent:.0f}%,  충전량={battery.charge}Wh")


print()
print("=" * 50)
print("실습 3: 상속 구현")
print("=" * 50)

# 공통 베이스 클래스
class SubAgent:
    """
    탐색 서브에이전트의 공통 인터페이스.
    src/agent/agent_interface.py 의 SubagentInterface 를 단순화한 버전.
    """

    def __init__(self, name):
        self.name = name
        self._target = None

    def update(self):
        """목표 위치를 재계산한다. 반드시 자식 클래스에서 구현해야 한다."""
        # NotImplementedError 를 발생시키면 "이 메서드는 자식이 구현해야 해" 라는
        # 명시적인 신호가 된다. 구현 없이 호출하면 즉시 에러로 알려준다.
        raise NotImplementedError(f"{self.name}.update() 가 구현되지 않았습니다")

    def get_target(self):
        return self._target

    def has_target(self):
        return self._target is not None

    def describe(self):
        if self.has_target():
            status = f"목표=({self._target[0]:.2f},{self._target[1]:.2f})"
        else:
            status = "목표=없음"
        return f"[{self.name}] {status}"


# 완성 예시
class GoStraightAgent(SubAgent):
    def __init__(self, direction_x, direction_y, step=0.12):
        super().__init__("직선이동")
        self._dir_x = direction_x
        self._dir_y = direction_y
        self._step = step
        self._current_x = 0.0
        self._current_y = 0.0

    def update(self):
        self._current_x += self._dir_x * self._step
        self._current_y += self._dir_y * self._step
        self._target = (self._current_x, self._current_y)


# -------------------------------------------------------
# [완성] TODO 3-2 (1): ReturnToStartAgent
# -------------------------------------------------------

class ReturnToStartAgent(SubAgent):
    """
    출발점(0, 0)으로 돌아가는 가장 단순한 에이전트.

    src/agent/subagents/return_to_start/return_to_start_subagent.py 에서
    실제 A* 경로 탐색으로 출발점을 향하는 것과 같은 개념이다.
    여기서는 단순히 고정 좌표 (0, 0) 을 목표로 설정한다.
    """

    def __init__(self):
        # super().__init__("이름") 으로 부모의 __init__ 을 먼저 실행한다.
        # 이것을 빠뜨리면 self.name, self._target 이 없어서 오류가 난다.
        super().__init__("귀환")

    def update(self):
        # 항상 출발점 (0, 0) 이 목표
        self._target = (0.0, 0.0)


# -------------------------------------------------------
# [완성] TODO 3-2 (2): SpiralSearchAgent
# -------------------------------------------------------

class SpiralSearchAgent(SubAgent):
    """
    나선형(spiral)으로 탐색하는 에이전트.

    매 update() 호출마다:
    - 각도를 30도씩 증가
    - 반경을 radius_step 만큼 증가
    → 점점 바깥쪽 원을 따라 목표를 설정한다.

    원 위의 점 좌표:
      x = radius * cos(angle_rad)
      y = radius * sin(angle_rad)
    """

    def __init__(self, radius_step=0.12):
        super().__init__("나선탐색")
        self._angle_deg = 0.0        # 현재 각도 (도)
        self._radius = 0.0           # 현재 반경 (미터)
        self._radius_step = radius_step   # 매 호출마다 반경 증가량
        self._angle_step = 30.0      # 매 호출마다 각도 증가량

    def update(self):
        # 반경과 각도를 증가시키면서 나선형으로 이동
        self._radius += self._radius_step
        self._angle_deg += self._angle_step

        # 각도를 라디안으로 변환 (math.cos, math.sin 은 라디안을 받는다)
        angle_rad = math.radians(self._angle_deg)

        # 원 위의 점 계산
        x = self._radius * math.cos(angle_rad)
        y = self._radius * math.sin(angle_rad)
        self._target = (x, y)


# 전체 에이전트 테스트
print("[에이전트 동작 테스트]")
agents = [
    GoStraightAgent(1, 0),       # x+ 방향 직선
    ReturnToStartAgent(),         # 항상 (0, 0)
    SpiralSearchAgent(0.12),      # 나선형
]

for agent in agents:
    print(f"\n{agent.name} 에이전트:")
    for step in range(4):
        agent.update()
        print(f"  step {step + 1}: {agent.describe()}")


print()
print("=" * 50)
print("실습 4: rescue_robot.py property 분석 [정답]")
print("=" * 50)
print("""
property 이름         | 반환 타입 | 내부에서 호출하는 것
----------------------|-----------|----------------------------------
x                    | float     | self._robot.position.x
y                    | float     | self._robot.position.y
direction            | float     | self._robot.orientation.degrees
elapsed_time         | float     | self._robot.time
remaining_time       | float     | max(0.0, max_time - elapsed_time)
is_time_almost_up    | bool      | self.remaining_time < 30
is_at_start          | bool      | self.distance_to_start < 0.04
victim_visible       | bool      | fixture_detector.find_fixtures(...)
exploration_complete | bool      | self._executor.agent.do_end()

모두 setter 없이 @property 만 있으므로 전부 읽기 전용이다.
robot.x = 0.5 처럼 쓰면 AttributeError 가 발생한다.
""")
