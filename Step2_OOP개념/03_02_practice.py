"""
3회차 보충 실습 — 캡슐화, 오버라이딩 심화, 오버로딩, 추상화, 다형성
실행: python study/03_02_practice.py

03_practice.py 에서 다룬 것:
  - 이름 규칙 (public / _protected / __private)
  - @property / @setter
  - 상속 기초, super().__init__()

이 파일에서 다루는 것:
  - 캡슐화   : 데이터 + 동작을 하나의 단위로 묶기
  - 오버라이딩 심화: super() 로 부모 동작 유지하면서 확장
  - 오버로딩  : Python 방식 (기본값, isinstance, *args)
  - 추상화   : abc.ABC, @abstractmethod
  - 다형성   : 같은 인터페이스 → 클래스마다 다른 동작
"""

print("=" * 50)
print("실습 1: 캡슐화 (Encapsulation)")
print("=" * 50)

# -------------------------------------------------------
# [개념] 캡슐화
#   데이터(속성)와 그 데이터를 다루는 동작(메서드)을
#   하나의 클래스 안에 묶는 것.
#
#   캡슐화 없음 → 데이터와 함수가 흩어져 있음
#   캡슐화 있음 → 연관된 것들이 한 클래스 안에
#
#   프로젝트 예: src/robot/robot.py
#     self.time_step, self.position, self.orientation 같은 데이터와
#     update(), move_wheels(), rotate_to_angle() 같은 동작이
#     Robot 클래스 안에 함께 묶여 있음.
# -------------------------------------------------------

# ❌ 캡슐화 없음: 데이터와 처리가 파일 어디에나 흩어짐
battery_level = 100.0
motor_speed   = 0.0

def consume_battery(wh):          # 이 함수가 battery_level 을 바꾸는지
    global battery_level          # 코드를 다 읽기 전까지 알 수 없음
    battery_level -= wh

def set_speed(v):
    global motor_speed
    motor_speed = max(0.0, min(1.0, v))


# ✅ 캡슐화 있음: 데이터와 동작이 한 클래스 안에 묶임
class DriveUnit:
    """배터리와 모터를 하나로 묶은 구동 유닛."""

    def __init__(self, capacity_wh: float):
        self._capacity   = capacity_wh   # 총 용량
        self._charge     = capacity_wh   # 현재 충전량
        self._speed      = 0.0           # 현재 속도 (0~1)

    # ── 배터리 관련 동작 ──────────────────────────────
    def consume(self, wh: float) -> bool:
        """wh 만큼 소모. 방전되면 False 반환."""
        self._charge = max(0.0, self._charge - wh)
        if self._charge == 0.0:
            print("  [DriveUnit] 배터리 방전!")
            return False
        return True

    @property
    def battery_percent(self) -> float:
        return self._charge / self._capacity * 100

    # ── 모터 관련 동작 ────────────────────────────────
    def set_speed(self, v: float):
        self._speed = max(0.0, min(1.0, v))

    @property
    def speed(self) -> float:
        return self._speed

    def status(self) -> str:
        return (f"배터리 {self.battery_percent:.0f}% | "
                f"속도 {self.speed:.2f}")


unit = DriveUnit(100)
unit.set_speed(0.8)
unit.consume(30)
print(f"  {unit.status()}")   # 배터리 70% | 속도 0.80
unit.consume(80)              # → 방전 메시지
print(f"  {unit.status()}")

print()

# -------------------------------------------------------
# TODO 3-02-1: SensorPack 클래스를 작성하세요.
#
# 요구사항:
#   - __init__(self, max_range_m: float)
#     self._max_range  = max_range_m
#     self._readings   = []          # 측정값 목록
#
#   - record(self, value: float)
#     readings 에 value 추가 (max_range 초과값은 max_range 로 클리핑)
#
#   - average (property, 읽기 전용)
#     readings 의 평균. readings 가 비어있으면 0.0
#
#   - clear(self)
#     readings 초기화
#
#   - summary(self) -> str
#     "측정 횟수: N, 평균: X.XXm, 최대범위: Xm" 반환
# -------------------------------------------------------

class SensorPack:
    pass  # TODO


# 구현 후 주석 해제:
# sensor = SensorPack(max_range_m=2.0)
# for v in [0.5, 1.2, 3.0, 0.8]:   # 3.0 은 2.0 으로 클리핑
#     sensor.record(v)
# print(f"  {sensor.summary()}")    # 측정 횟수: 4, 평균: 1.13m, 최대범위: 2.0m
# sensor.clear()
# print(f"  평균(초기화 후): {sensor.average}")   # 0.0


print("=" * 50)
print("실습 2: 오버라이딩 심화 (super())")
print("=" * 50)

# -------------------------------------------------------
# [개념] 오버라이딩 + super()
#
#   자식 클래스가 부모 메서드를 같은 이름으로 재정의하는 것.
#   super().메서드() 로 부모 동작을 유지하면서 기능을 추가할 수 있음.
#
#   프로젝트 예: src/robot/devices/camera.py
#     class Camera(Sensor):
#         def __init__(self, ...):
#             super().__init__(device, time_step, step_counter)  ← 부모 초기화
#             self.image = CameraImage(...)  ← 카메라 전용 추가
# -------------------------------------------------------

class Logger:
    """로그를 기록하는 기본 클래스."""

    def __init__(self, name: str):
        self.name   = name
        self._logs  = []

    def log(self, msg: str):
        entry = f"[{self.name}] {msg}"
        self._logs.append(entry)
        print(f"  {entry}")

    def summary(self) -> str:
        return f"{self.name}: 총 {len(self._logs)}건"


class TimestampLogger(Logger):
    """타임스탬프를 붙이는 로거 — Logger 오버라이딩."""

    def __init__(self, name: str):
        super().__init__(name)    # ← 부모 __init__ 실행 (name, _logs 초기화)
        self._tick = 0

    def log(self, msg: str):      # ← 오버라이딩
        self._tick += 1
        super().log(f"[t={self._tick:04d}] {msg}")   # 부모 log 에 타임스탬프 추가


class FilteredLogger(TimestampLogger):
    """특정 키워드를 무시하는 로거 — 2단계 오버라이딩."""

    def __init__(self, name: str, ignore_keyword: str):
        super().__init__(name)
        self._ignore = ignore_keyword

    def log(self, msg: str):      # ← 또 오버라이딩 (2단계)
        if self._ignore.lower() in msg.lower():
            return                # 키워드 포함 시 기록 안 함
        super().log(msg)          # 그 외엔 TimestampLogger.log() 실행


print("── 기본 Logger ──")
base = Logger("BASE")
base.log("시작")
base.log("탐색 중")

print("── TimestampLogger ──")
ts = TimestampLogger("TS")
ts.log("시작")
ts.log("탐색 중")

print("── FilteredLogger (DEBUG 무시) ──")
fl = FilteredLogger("FL", ignore_keyword="DEBUG")
fl.log("시작")
fl.log("DEBUG: 내부 값=0.12")   # 무시됨
fl.log("조난자 발견")
print(f"  {fl.summary()}")      # 2건 (DEBUG 제외)

print()

# -------------------------------------------------------
# TODO 3-02-2: CountingLogger 를 작성하세요.
#
#   TimestampLogger 를 상속받아:
#   - __init__(self, name, warn_after: int)
#     warn_after 건 이상 로그가 쌓이면 경고
#
#   - log(self, msg)
#     super().log(msg) 를 먼저 실행한 뒤,
#     누적 건수가 warn_after 의 배수에 도달할 때마다
#     "  ⚠ 경고: N건 누적" 을 출력
#
#   예시:
#     cl = CountingLogger("CL", warn_after=3)
#     for msg in ["a", "b", "c", "d", "e", "f"]:
#         cl.log(msg)
#     → "c" 기록 후, "f" 기록 후 경고 출력
# -------------------------------------------------------

class CountingLogger:
    pass  # TODO: TimestampLogger 를 상속받아 구현


# 구현 후 주석 해제:
# cl = CountingLogger("CL", warn_after=3)
# for msg in ["a", "b", "c", "d", "e", "f"]:
#     cl.log(msg)


print("=" * 50)
print("실습 3: 오버로딩 (Overloading)")
print("=" * 50)

# -------------------------------------------------------
# [개념] 오버로딩
#
#   Java·C++ 은 매개변수 타입/개수가 다른 같은 이름의 메서드를
#   여러 개 정의할 수 있음. Python 은 이를 직접 지원하지 않음.
#   대신 세 가지 방법으로 유사하게 구현:
#
#   방법 A: 기본값 (Default Arguments)
#   방법 B: isinstance() 로 타입 분기
#   방법 C: *args 로 가변 인자 처리
# -------------------------------------------------------

import math

# ── 방법 A: 기본값 ──────────────────────────────────────
class Motor:
    """기본값으로 오버로딩을 흉내 내는 예."""

    def move(self, speed: float = 1.0, duration: float = None):
        """
        move(0.5)         → 속도 0.5 로 무한 이동
        move(0.5, 2.0)    → 속도 0.5 로 2초 이동
        """
        if duration is None:
            print(f"  [Motor] 속도 {speed:.1f} 로 계속 이동")
        else:
            print(f"  [Motor] 속도 {speed:.1f} 로 {duration:.1f}초 이동")

print("── 방법 A: 기본값 ──")
m = Motor()
m.move(0.5)
m.move(0.5, 2.0)

print()

# ── 방법 B: isinstance() 타입 분기 ─────────────────────
class Position:
    """isinstance 로 인자 타입에 따라 다르게 동작."""

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def distance_to(self, other) -> float:
        """
        distance_to((0.3, 0.4))    → tuple 을 받으면
        distance_to(Position(...)) → Position 을 받으면
        """
        if isinstance(other, tuple):
            ox, oy = other
        elif isinstance(other, Position):
            ox, oy = other.x, other.y
        else:
            raise TypeError(f"지원하지 않는 타입: {type(other)}")
        return math.sqrt((self.x - ox) ** 2 + (self.y - oy) ** 2)

    def __repr__(self):
        return f"Position({self.x:.2f}, {self.y:.2f})"


print("── 방법 B: isinstance 타입 분기 ──")
p1 = Position(0.0, 0.0)
p2 = Position(0.3, 0.4)
print(f"  tuple  까지 거리: {p1.distance_to((0.3, 0.4)):.3f}m")
print(f"  Position 까지 거리: {p1.distance_to(p2):.3f}m")

print()

# ── 방법 C: *args 가변 인자 ─────────────────────────────
class Vector2D:
    """
    Vector2D(1.0, 0.0)      → x, y 로 생성
    Vector2D((1.0, 0.0))    → 튜플로 생성
    Vector2D(angle_deg=90)  → 각도로 생성 (단위 벡터)
    """

    def __init__(self, *args, angle_deg: float = None):
        if angle_deg is not None:                    # 각도로 생성
            rad = math.radians(angle_deg)
            self.x = math.cos(rad)
            self.y = math.sin(rad)
        elif len(args) == 2:                         # x, y 두 값
            self.x, self.y = float(args[0]), float(args[1])
        elif len(args) == 1 and isinstance(args[0], (tuple, list)):
            self.x, self.y = float(args[0][0]), float(args[0][1])
        else:
            raise ValueError(f"인자 형식 오류: {args}")

    def __repr__(self):
        return f"Vector2D({self.x:.3f}, {self.y:.3f})"


print("── 방법 C: *args 가변 인자 ──")
v1 = Vector2D(1.0, 0.0)
v2 = Vector2D((0.5, 0.5))
v3 = Vector2D(angle_deg=90)
print(f"  x,y 로: {v1}")
print(f"  튜플로: {v2}")
print(f"  90도 단위벡터: {v3}")

print()

# -------------------------------------------------------
# TODO 3-02-3: Rectangle 클래스를 작성하세요.
#
#   생성자가 세 가지 방법으로 동작해야 합니다:
#     Rectangle(3.0, 2.0)          → 가로 3.0, 세로 2.0
#     Rectangle((3.0, 2.0))        → 튜플로 (가로, 세로)
#     Rectangle(side=2.5)          → 정사각형 (가로=세로=2.5)
#
#   property:
#     area (읽기 전용): 가로 × 세로
#     perimeter (읽기 전용): (가로 + 세로) × 2
#     is_square (읽기 전용): 가로 == 세로 이면 True
#
#   메서드:
#     __repr__(self) → "Rectangle(가로=X.XX, 세로=X.XX)" 반환
# -------------------------------------------------------

class Rectangle:
    pass  # TODO


# 구현 후 주석 해제:
# r1 = Rectangle(3.0, 2.0)
# r2 = Rectangle((4.0, 4.0))
# r3 = Rectangle(side=5.0)
# for r in [r1, r2, r3]:
#     print(f"  {r}  넓이={r.area:.2f}  둘레={r.perimeter:.2f}  정사각형={r.is_square}")


print("=" * 50)
print("실습 4: 추상화 (Abstraction)")
print("=" * 50)

# -------------------------------------------------------
# [개념] 추상화
#
#   공통 인터페이스(메서드 이름과 역할)만 정의하고
#   구체적인 구현은 자식 클래스에게 맡기는 것.
#
#   Python 표준 방법: abc 모듈
#     - ABC 를 상속
#     - @abstractmethod 데코레이터
#     → 추상 메서드를 구현하지 않은 자식 클래스는
#       인스턴스화 시 TypeError 발생
#
#   프로젝트 예: src/agent/agent_interface.py
#     class PositionFinderInterface:
#         def update(self): raise NotImplementedError
#         def get_target_position(self): raise NotImplementedError
# -------------------------------------------------------

from abc import ABC, abstractmethod

class Sensor(ABC):
    """모든 센서의 공통 인터페이스."""

    def __init__(self, name: str):
        self.name = name
        self._enabled = False

    def enable(self):
        self._enabled = True
        print(f"  [{self.name}] 활성화")

    def disable(self):
        self._enabled = False

    @abstractmethod
    def read(self) -> float:
        """현재 측정값을 반환. 자식 클래스에서 반드시 구현."""
        pass

    @abstractmethod
    def unit(self) -> str:
        """측정 단위 문자열 반환. 자식 클래스에서 반드시 구현."""
        pass

    def status(self) -> str:
        """공통 상태 문자열 (자식이 구현한 read/unit 사용)."""
        if not self._enabled:
            return f"[{self.name}] 비활성화"
        return f"[{self.name}] {self.read():.3f} {self.unit()}"


class DistanceSensor(Sensor):
    def __init__(self):
        super().__init__("거리센서")
        self._distance = 0.0

    def set_distance(self, d: float):
        self._distance = d

    def read(self) -> float:        # 추상 메서드 구현 (필수)
        return self._distance

    def unit(self) -> str:          # 추상 메서드 구현 (필수)
        return "m"


class TemperatureSensor(Sensor):
    def __init__(self, base_temp: float = 25.0):
        super().__init__("온도센서")
        self._temp = base_temp

    def read(self) -> float:
        return self._temp

    def unit(self) -> str:
        return "°C"


print("── 추상 클래스 사용 ──")

# 추상 클래스 직접 생성 시 TypeError
try:
    s = Sensor("직접생성")
except TypeError as e:
    print(f"  추상 클래스 직접 생성 불가: {e}")

ds = DistanceSensor()
ts = TemperatureSensor(36.5)

ds.enable()
ds.set_distance(0.245)
ts.enable()

print(f"  {ds.status()}")
print(f"  {ts.status()}")

print()

# -------------------------------------------------------
# TODO 3-02-4: BatterySensor 를 작성하세요.
#
#   Sensor 를 상속받아 구현:
#   - __init__(self, capacity_wh: float)
#     super().__init__("배터리센서")
#     self._capacity = capacity_wh
#     self._charge   = capacity_wh
#
#   - drain(self, wh: float)
#     _charge 를 wh 만큼 줄임 (0 미만은 0 으로 처리)
#
#   - read(self) -> float   ← 추상 메서드 구현
#     현재 충전량(Wh)을 반환
#
#   - unit(self) -> str     ← 추상 메서드 구현
#     "Wh" 반환
#
#   - percent (property, 읽기 전용)
#     충전량 백분율 (0 ~ 100)
# -------------------------------------------------------

class BatterySensor:
    pass  # TODO: Sensor 를 상속받아 구현


# 구현 후 주석 해제:
# bs = BatterySensor(100)
# bs.enable()
# bs.drain(40)
# print(f"  {bs.status()}")          # [배터리센서] 60.000 Wh
# print(f"  충전율: {bs.percent:.0f}%")  # 60%


print("=" * 50)
print("실습 5: 다형성 (Polymorphism)")
print("=" * 50)

# -------------------------------------------------------
# [개념] 다형성
#
#   같은 인터페이스(메서드 이름)를 호출했을 때
#   객체의 실제 타입에 따라 다른 동작이 실행되는 것.
#
#   오버라이딩 + 상속이 만들어내는 결과.
#   "같은 코드 → 다른 결과"
#
#   프로젝트 예: src/agent/agent.py
#     self.__navigation_agent = SubagentPriorityCombiner([
#         GoToFixturesAgent(mapper),       # Priority 1
#         FollowWallsAgent(mapper),        # Priority 2
#         GoToNonDiscoveredAgent(mapper),  # Priority 3
#     ])
#     → 모두 update() / get_target_position() 을 가지지만
#       클래스마다 다른 방식으로 목표를 계산
# -------------------------------------------------------

class ReportStrategy(ABC):
    """조난자 보고 전략의 공통 인터페이스."""

    @abstractmethod
    def report(self, letter: str, position: tuple) -> str:
        pass


class ConsoleReport(ReportStrategy):
    def report(self, letter: str, position: tuple) -> str:
        msg = f"[콘솔] 조난자 '{letter}' @ ({position[0]:.2f}, {position[1]:.2f})"
        print(f"  {msg}")
        return msg


class FileReport(ReportStrategy):
    def __init__(self, filename: str):
        self._filename = filename
        self._entries  = []

    def report(self, letter: str, position: tuple) -> str:
        entry = f"{letter},{position[0]:.3f},{position[1]:.3f}"
        self._entries.append(entry)
        msg = f"[파일:{self._filename}] '{entry}' 기록"
        print(f"  {msg}")
        return msg


class NetworkReport(ReportStrategy):
    def __init__(self, endpoint: str):
        self._endpoint = endpoint

    def report(self, letter: str, position: tuple) -> str:
        msg = f"[네트워크:{self._endpoint}] POST letter={letter} x={position[0]:.3f} y={position[1]:.3f}"
        print(f"  {msg}")
        return msg


def broadcast_victim(strategies: list, letter: str, position: tuple):
    """다형성 핵심: strategies 안의 실제 타입이 무엇이든 report() 호출."""
    print(f"  ── 조난자 '{letter}' 브로드캐스트 ──")
    for strategy in strategies:
        strategy.report(letter, position)   # ← 같은 호출, 다른 동작


strategies = [
    ConsoleReport(),
    FileReport("victims.csv"),
    NetworkReport("192.168.1.10:8080"),
]

broadcast_victim(strategies, "H", (0.24, -0.12))
print()
broadcast_victim(strategies, "S", (0.48,  0.36))

print()

# -------------------------------------------------------
# TODO 3-02-5: ScoringStrategy 계층을 작성하세요.
#
# 1) ScoringStrategy (ABC)
#    - calculate(self, letter: str, distance_m: float) -> float  (추상)
#    - describe(self) -> str  (추상)
#
# 2) BasicScoring(ScoringStrategy)
#    - 점수 = letter 가 'H' 이면 10, 'S' 이면 6, 그 외 4
#
# 3) DistanceBonus(ScoringStrategy)
#    - 점수 = 기본 5점 + (1.0 - distance_m) * 10
#      (distance_m 가 작을수록 보너스 높음, 최솟값 0)
#
# 4) ComboScoring(ScoringStrategy)
#    - __init__(self, strategies: list)
#    - calculate: strategies 리스트 각 전략의 점수를 합산
#    - describe: 각 전략의 describe() 를 " + ".join
#
# 다형성 확인:
#   scorers = [BasicScoring(), DistanceBonus(), ComboScoring([BasicScoring(), DistanceBonus()])]
#   for sc in scorers:
#       score = sc.calculate("H", 0.3)
#       print(f"  {sc.describe()} → {score:.1f}점")
# -------------------------------------------------------

# 여기에 코드를 작성하세요 ↓


# 구현 후 주석 해제:
# scorers = [
#     BasicScoring(),
#     DistanceBonus(),
#     ComboScoring([BasicScoring(), DistanceBonus()]),
# ]
# for sc in scorers:
#     score = sc.calculate("H", 0.3)
#     print(f"  {sc.describe()} → {score:.1f}점")


print("=" * 50)
print("실습 6: 프로젝트 코드 분석")
print("=" * 50)

print("""
아래 질문에 답하면서 프로젝트 코드에서 개념을 직접 확인하세요.

Q1. 캡슐화
    src/robot/robot.py 의 Robot 클래스에서
    '데이터(속성)' 에 해당하는 것 3가지와
    '동작(메서드)' 에 해당하는 것 3가지를 찾아보세요.
    답: 데이터 → ?, ?, ?
        동작   → ?, ?, ?

Q2. 오버라이딩
    src/robot/devices/camera.py 의 Camera 는
    어떤 클래스를 상속받나요?
    부모 클래스와 비교해서 추가된 것은 무엇인가요?
    답: 상속 대상 → ?
        추가된 것  → ?

Q3. 오버로딩 (isinstance 방식)
    src/data_structures/compound_pixel_grid.py 에서
    expand_to_grid_index() 메서드를 찾아보세요.
    인자로 np.ndarray 와 tuple 을 모두 받을 수 있나요?
    어떻게 처리하고 있나요?
    답: ?

Q4. 추상화
    src/agent/agent_interface.py 를 열어보세요.
    어떤 메서드가 추상 메서드처럼 정의되어 있나요?
    (NotImplementedError 또는 pass 로 구현된 것)
    답: ?

Q5. 다형성
    src/agent/agent.py 에서 SubagentPriorityCombiner 에
    들어있는 에이전트 3개의 이름을 찾아보세요.
    모두 공통으로 가지는 메서드 이름은 무엇인가요?
    답: 에이전트 → ?, ?, ?
        공통 메서드 → ?
""")
