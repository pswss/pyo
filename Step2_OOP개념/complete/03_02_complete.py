"""
3회차 보충 완성본 - 캡슐화, 오버라이딩 심화, 오버로딩, 추상화, 다형성
실행: python study/complete/03_02_complete.py
"""

import math
from abc import ABC, abstractmethod

# ======================================================
print("=" * 50)
print("실습 1: 캡슐화 (Encapsulation)")
print("=" * 50)
# ======================================================

class DriveUnit:
    """배터리와 모터를 하나로 묶은 구동 유닛."""

    def __init__(self, capacity_wh: float):
        self._capacity = capacity_wh
        self._charge   = capacity_wh
        self._speed    = 0.0

    def consume(self, wh: float) -> bool:
        self._charge = max(0.0, self._charge - wh)
        if self._charge == 0.0:
            print("  [DriveUnit] 배터리 방전!")
            return False
        return True

    @property
    def battery_percent(self) -> float:
        return self._charge / self._capacity * 100

    def set_speed(self, v: float):
        self._speed = max(0.0, min(1.0, v))

    @property
    def speed(self) -> float:
        return self._speed

    def status(self) -> str:
        return f"배터리 {self.battery_percent:.0f}% | 속도 {self.speed:.2f}"


unit = DriveUnit(100)
unit.set_speed(0.8)
unit.consume(30)
print(f"  {unit.status()}")
unit.consume(80)
print(f"  {unit.status()}")

print()

# -------------------------------------------------------
# [완성] TODO 3-02-1: SensorPack
#
# 핵심 포인트:
#   - record() 에서 max_range 로 클리핑 → setter 없이 메서드로 처리
#   - average 는 읽기 전용 property (readings 의 평균)
#   - readings 가 비어있으면 ZeroDivisionError 방지
# -------------------------------------------------------

class SensorPack:
    """여러 번의 측정값을 누적·요약하는 센서 팩."""

    def __init__(self, max_range_m: float):
        self._max_range = max_range_m
        self._readings  = []

    def record(self, value: float):
        """
        측정값을 추가한다. max_range 를 초과하면 max_range 로 클리핑.

        max(0, min(max_range, value)) 대신 min 만 쓰는 이유:
        거리 센서 값은 음수가 없어서 하한(0) 클리핑은 생략한다.
        """
        clipped = min(self._max_range, value)
        self._readings.append(clipped)

    @property
    def average(self) -> float:
        """현재까지 기록된 측정값의 평균. 데이터가 없으면 0.0."""
        if not self._readings:          # 빈 리스트는 False → ZeroDivisionError 방지
            return 0.0
        return sum(self._readings) / len(self._readings)

    def clear(self):
        """측정값 목록을 초기화한다."""
        self._readings = []

    def summary(self) -> str:
        return (f"측정 횟수: {len(self._readings)}, "
                f"평균: {self.average:.2f}m, "
                f"최대범위: {self._max_range}m")


print("[SensorPack 테스트]")
sensor = SensorPack(max_range_m=2.0)
for v in [0.5, 1.2, 3.0, 0.8]:   # 3.0 은 2.0 으로 클리핑
    sensor.record(v)
print(f"  {sensor.summary()}")    # 측정 횟수: 4, 평균: 1.13m, 최대범위: 2.0m
sensor.clear()
print(f"  평균(초기화 후): {sensor.average}")   # 0.0


# ======================================================
print()
print("=" * 50)
print("실습 2: 오버라이딩 심화 (super())")
print("=" * 50)
# ======================================================

class Logger:
    def __init__(self, name: str):
        self.name  = name
        self._logs = []

    def log(self, msg: str):
        entry = f"[{self.name}] {msg}"
        self._logs.append(entry)
        print(f"  {entry}")

    def summary(self) -> str:
        return f"{self.name}: 총 {len(self._logs)}건"


class TimestampLogger(Logger):
    def __init__(self, name: str):
        super().__init__(name)
        self._tick = 0

    def log(self, msg: str):
        self._tick += 1
        super().log(f"[t={self._tick:04d}] {msg}")


class FilteredLogger(TimestampLogger):
    def __init__(self, name: str, ignore_keyword: str):
        super().__init__(name)
        self._ignore = ignore_keyword

    def log(self, msg: str):
        if self._ignore.lower() in msg.lower():
            return
        super().log(msg)


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
fl.log("DEBUG: 내부 값=0.12")
fl.log("조난자 발견")
print(f"  {fl.summary()}")

print()

# -------------------------------------------------------
# [완성] TODO 3-02-2: CountingLogger
#
# 핵심 포인트:
#   - TimestampLogger 를 상속 → super().log() 로 타임스탬프 기능 유지
#   - _logs 는 Logger 에 있지만 super().log() 를 타고 거기까지 추가됨
#   - 경고 조건: len(self._logs) % warn_after == 0
#     → warn_after 의 배수에 도달할 때마다 출력
# -------------------------------------------------------

class CountingLogger(TimestampLogger):
    """
    누적 건수가 warn_after 의 배수에 도달하면 경고를 출력하는 로거.

    호출 순서:
      CountingLogger.log()
        └─ super().log()  →  TimestampLogger.log()
              └─ super().log()  →  Logger.log()  (실제 출력 + _logs 추가)
        └─ 경고 체크 (Logger._logs 의 길이 확인)
    """

    def __init__(self, name: str, warn_after: int):
        super().__init__(name)
        self._warn_after = warn_after

    def log(self, msg: str):
        super().log(msg)                        # 부모(TimestampLogger) 실행
        if len(self._logs) % self._warn_after == 0:   # 배수 도달 시 경고
            print(f"  [경고] {len(self._logs)}건 누적")


print("[CountingLogger 테스트 (warn_after=3)]")
cl = CountingLogger("CL", warn_after=3)
for msg in ["a", "b", "c", "d", "e", "f"]:
    cl.log(msg)


# ======================================================
print()
print("=" * 50)
print("실습 3: 오버로딩 (Overloading)")
print("=" * 50)
# ======================================================

class Motor:
    def move(self, speed: float = 1.0, duration: float = None):
        if duration is None:
            print(f"  [Motor] 속도 {speed:.1f} 로 계속 이동")
        else:
            print(f"  [Motor] 속도 {speed:.1f} 로 {duration:.1f}초 이동")

print("── 방법 A: 기본값 ──")
m = Motor()
m.move(0.5)
m.move(0.5, 2.0)
print()

class Position:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def distance_to(self, other) -> float:
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

class Vector2D:
    def __init__(self, *args, angle_deg: float = None):
        if angle_deg is not None:
            rad = math.radians(angle_deg)
            self.x = math.cos(rad)
            self.y = math.sin(rad)
        elif len(args) == 2:
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
# [완성] TODO 3-02-3: Rectangle
#
# 핵심 포인트:
#   - *args 로 (w, h) 또는 ((w, h),) 받기
#   - keyword-only 인자 side= 로 정사각형 처리
#   - is_square 비교 시 부동소수점 오차 주의 → round() 또는 math.isclose()
# -------------------------------------------------------

class Rectangle:
    """
    세 가지 방식으로 생성할 수 있는 직사각형.

      Rectangle(3.0, 2.0)       → 가로·세로
      Rectangle((3.0, 2.0))     → 튜플
      Rectangle(side=2.5)       → 정사각형
    """

    def __init__(self, *args, side: float = None):
        if side is not None:                                    # 정사각형
            self._w = side
            self._h = side
        elif len(args) == 2:                                    # 가로, 세로
            self._w, self._h = float(args[0]), float(args[1])
        elif len(args) == 1 and isinstance(args[0], (tuple, list)):  # 튜플
            self._w, self._h = float(args[0][0]), float(args[0][1])
        else:
            raise ValueError(f"사용법: Rectangle(w, h) / Rectangle((w,h)) / Rectangle(side=s)")

    @property
    def area(self) -> float:
        return self._w * self._h

    @property
    def perimeter(self) -> float:
        return (self._w + self._h) * 2

    @property
    def is_square(self) -> bool:
        return math.isclose(self._w, self._h)   # 부동소수점 비교는 isclose 가 안전

    def __repr__(self) -> str:
        return f"Rectangle(가로={self._w:.2f}, 세로={self._h:.2f})"


print("[Rectangle 테스트]")
r1 = Rectangle(3.0, 2.0)
r2 = Rectangle((4.0, 4.0))
r3 = Rectangle(side=5.0)
for r in [r1, r2, r3]:
    print(f"  {r}  넓이={r.area:.2f}  둘레={r.perimeter:.2f}  정사각형={r.is_square}")


# ======================================================
print()
print("=" * 50)
print("실습 4: 추상화 (Abstraction)")
print("=" * 50)
# ======================================================

class Sensor(ABC):
    def __init__(self, name: str):
        self.name     = name
        self._enabled = False

    def enable(self):
        self._enabled = True
        print(f"  [{self.name}] 활성화")

    def disable(self):
        self._enabled = False

    @abstractmethod
    def read(self) -> float:
        pass

    @abstractmethod
    def unit(self) -> str:
        pass

    def status(self) -> str:
        if not self._enabled:
            return f"[{self.name}] 비활성화"
        return f"[{self.name}] {self.read():.3f} {self.unit()}"


class DistanceSensor(Sensor):
    def __init__(self):
        super().__init__("거리센서")
        self._distance = 0.0

    def set_distance(self, d: float):
        self._distance = d

    def read(self) -> float:
        return self._distance

    def unit(self) -> str:
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
# [완성] TODO 3-02-4: BatterySensor
#
# 핵심 포인트:
#   - Sensor(ABC) 를 상속 → read(), unit() 반드시 구현
#   - drain() 에서 0 미만 방지 → max(0.0, ...)
#   - percent 는 읽기 전용 property (setter 없음)
#   - capacity 가 0 이면 ZeroDivisionError 방지
# -------------------------------------------------------

class BatterySensor(Sensor):
    """
    배터리 잔량을 측정하는 센서.

    read() → 현재 충전량(Wh) 반환
    unit() → "Wh" 반환
    percent → 현재 충전량을 백분율로 반환
    drain() → 충전량 소모
    """

    def __init__(self, capacity_wh: float):
        super().__init__("배터리센서")
        self._capacity = capacity_wh
        self._charge   = capacity_wh

    def drain(self, wh: float):
        """충전량을 wh 만큼 줄인다. 0 미만은 0 으로 처리."""
        self._charge = max(0.0, self._charge - wh)

    def read(self) -> float:
        """현재 충전량(Wh) 반환. Sensor.status() 가 이 값을 사용한다."""
        return self._charge

    def unit(self) -> str:
        return "Wh"

    @property
    def percent(self) -> float:
        """충전량 백분율 (0 ~ 100). 읽기 전용."""
        if self._capacity == 0:
            return 0.0
        return self._charge / self._capacity * 100


print("[BatterySensor 테스트]")
bs = BatterySensor(100)
bs.enable()
bs.drain(40)
print(f"  {bs.status()}")               # [배터리센서] 60.000 Wh
print(f"  충전율: {bs.percent:.0f}%")   # 60%
bs.drain(200)                           # 200Wh 소모 → 0으로 클리핑
print(f"  방전 후: {bs.status()}")


# ======================================================
print()
print("=" * 50)
print("실습 5: 다형성 (Polymorphism)")
print("=" * 50)
# ======================================================

class ReportStrategy(ABC):
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
    print(f"  ── 조난자 '{letter}' 브로드캐스트 ──")
    for strategy in strategies:
        strategy.report(letter, position)


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
# [완성] TODO 3-02-5: ScoringStrategy 계층
#
# 핵심 포인트:
#   - BasicScoring: letter 에 따라 고정 점수 딕셔너리 사용
#   - DistanceBonus: 거리가 가까울수록 보너스 높음, 음수 방지
#   - ComboScoring: 다형성의 정수 - 자신도 ScoringStrategy 이므로
#     ComboScoring 안에 ComboScoring 을 넣을 수도 있음 (재귀 가능)
# -------------------------------------------------------

class ScoringStrategy(ABC):
    @abstractmethod
    def calculate(self, letter: str, distance_m: float) -> float:
        pass

    @abstractmethod
    def describe(self) -> str:
        pass


class BasicScoring(ScoringStrategy):
    """
    조난자 글자 종류에 따른 고정 점수.

    딕셔너리를 쓰면 if-elif 없이 간결하게 처리 가능.
    get(letter, 기본값) 으로 없는 글자도 안전하게 처리.
    """

    _SCORES = {"H": 10, "S": 6}   # H: Hazardous, S: Stable

    def calculate(self, letter: str, distance_m: float) -> float:
        return float(self._SCORES.get(letter, 4))   # 그 외 글자는 4점

    def describe(self) -> str:
        return "기본점수(H=10, S=6, 기타=4)"


class DistanceBonus(ScoringStrategy):
    """
    거리가 가까울수록 보너스 점수가 높은 전략.

    distance_m 이 0이면 최대 보너스(10점), 1m 이면 0점, 1m 이상이면 음수이지만 max(0) 로 처리.
    """

    def calculate(self, letter: str, distance_m: float) -> float:
        bonus = (1.0 - distance_m) * 10
        return 5.0 + max(0.0, bonus)   # 기본 5점 + 거리 보너스

    def describe(self) -> str:
        return "거리보너스(기본5 + (1-dist)*10)"


class ComboScoring(ScoringStrategy):
    """
    여러 전략의 점수를 합산하는 복합 전략.

    다형성 활용: strategies 안에 어떤 ScoringStrategy 가 들어와도
    calculate() / describe() 를 같은 방식으로 호출 가능.
    ComboScoring 자체도 ScoringStrategy 이므로 중첩 가능.
    """

    def __init__(self, strategies: list):
        self._strategies = strategies

    def calculate(self, letter: str, distance_m: float) -> float:
        return sum(s.calculate(letter, distance_m) for s in self._strategies)

    def describe(self) -> str:
        return " + ".join(s.describe() for s in self._strategies)


print("[ScoringStrategy 다형성 테스트]")
scorers = [
    BasicScoring(),
    DistanceBonus(),
    ComboScoring([BasicScoring(), DistanceBonus()]),
]
for sc in scorers:
    score = sc.calculate("H", 0.3)
    print(f"  {sc.describe()}")
    print(f"    → {score:.1f}점")


# ======================================================
print()
print("=" * 50)
print("실습 6: 프로젝트 코드 분석 [정답]")
print("=" * 50)
# ======================================================

print("""
Q1. 캡슐화 - src/robot/robot.py Robot 클래스
    데이터: time_step, position, orientation (+ __time, __start_time 등)
    동작:   update(), move_wheels(), rotate_to_angle(), do_loop(), set_start_time()

Q2. 오버라이딩 - src/robot/devices/camera.py
    Camera 는 Sensor 를 상속.
    부모(Sensor)와 비교해서 추가된 것:
      - self.image = CameraImage(...)    카메라 이미지 객체
      - update() 오버라이딩: 이미지 캡처 + 방향 정보 처리

Q3. 오버로딩 (isinstance 방식) - src/data_structures/compound_pixel_grid.py
    expand_to_grid_index(grid_index) 는 np.ndarray 와 tuple 을 모두 받을 수 있음.
    내부에서 np.array(grid_index) 로 감싸서 통일하거나,
    isinstance 로 분기 없이 numpy 가 자동 처리.

Q4. 추상화 - src/agent/agent_interface.py
    SubagentInterface, PositionFinderInterface 등에서
    raise NotImplementedError 로 정의된 추상 메서드:
      - update()
      - get_target_position()
      - target_position_exists()

Q5. 다형성 - src/agent/agent.py SubagentPriorityCombiner
    에이전트 3개:
      GoToFixturesAgent, FollowWallsAgent, GoToNonDiscoveredAgent
    공통 메서드:
      update(), get_target_position(), target_position_exists()
    → SubagentPriorityCombiner 는 세 클래스 중 어느 것이 들어있는지 몰라도
      같은 방식으로 호출 가능. (다형성)
""")
