# 3회차 — @property · 이름 규칙 · 상속

> 목표: @property 로 안전한 속성을 만들고, 상속으로 코드를 재사용할 수 있다.
>       소스 코드의 `_변수` 와 `__변수` 의 의미를 안다.
> 실습 파일: `03_practice.py`

---

## 1. 이름 규칙 — `_` 와 `__`

파이썬에는 변수/메서드 접근 수준을 이름으로 표시하는 관례가 있습니다.

### 공개 (Public)
```python
self.x = 0.0         # 누구나 접근 가능
self.speed = 1.0
```

### 비공개 관례 (Convention Private) — 밑줄 하나
```python
self._mapper = Mapper()    # "내부용이니 외부에서 직접 쓰지 마세요" 라는 신호
self._path = []
```
파이썬이 강제로 막지는 않지만, 개발자들 사이의 약속입니다.
`rescue_robot.py` 에서는 모든 내부 객체가 `_robot`, `_mapper`, `_executor` 처럼 `_` 로 시작합니다.

### 강제 비공개 (Name Mangling) — 밑줄 둘
```python
self.__navigation_agent = SubagentPriorityCombiner(...)
```
파이썬이 이름을 `_클래스명__변수명` 으로 바꿔서 외부 접근을 실질적으로 차단합니다.

```python
class Agent:
    def __init__(self):
        self.__secret = 42

a = Agent()
# a.__secret      → AttributeError! 접근 불가
# a._Agent__secret → 42  (파이썬 내부 이름 변환 후엔 접근 가능하지만 하지 말 것)
```

`src/agent/agent.py` 에서:
```python
self.__navigation_agent    # Agent 클래스 내부에서만 사용
self.__stage_machine       # 외부(Executor)에서 직접 접근 불가
self.__target_position     # get_target_position() 메서드로만 접근
```

---

## 2. @property — 메서드를 변수처럼

### 문제 상황

```python
class Robot:
    def __init__(self):
        self.x = 0.0

robot = Robot()
robot.x = -999   # 유효하지 않은 값을 막을 수 없음
```

### @property 로 해결

```python
class Robot:
    def __init__(self):
        self._x = 0.0        # 실제 값은 _x 에 저장

    @property
    def x(self):             # 읽기: robot.x
        return self._x

    @x.setter
    def x(self, value):      # 쓰기: robot.x = 0.5 (유효성 검사 가능)
        if value < -10 or value > 10:
            raise ValueError(f"x 좌표 범위 초과: {value}")
        self._x = value
```

사용:
```python
robot = Robot()
robot.x = 0.5      # setter 호출 → 유효성 검사 통과
print(robot.x)     # getter 호출 → 0.5 반환

robot.x = 999      # → ValueError: x 좌표 범위 초과: 999
```

### 읽기 전용 property (setter 없음)

```python
class Robot:
    def __init__(self, width, height):
        self._width = width
        self._height = height

    @property
    def area(self):             # 계산된 값을 변수처럼 제공
        return self._width * self._height

    @property
    def is_square(self):
        return self._width == self._height
```

사용:
```python
r = Robot(0.18, 0.18)
print(r.area)        # 0.0324  (메서드지만 () 없이 호출)
print(r.is_square)   # True
r.area = 1.0         # → AttributeError: can't set attribute
```

### rescue_robot.py 의 property 들

```python
@property
def x(self) -> float:
    return self._robot.position.x      # Robot 객체의 값을 읽어서 반환

@property
def direction(self) -> float:
    return self._robot.orientation.degrees

@property
def is_time_almost_up(self) -> bool:
    return self.remaining_time < 30    # 다른 property를 사용한 계산

@property
def remaining_time(self) -> float:
    return max(0.0, self._executor.max_time_in_run - self.elapsed_time)
```

호출 방법:
```python
robot = RescueRobot()
print(robot.x)              # 메서드지만 () 없이 사용
print(robot.is_time_almost_up)   # True / False
```

---

## 3. 상속 — 부모의 기능을 물려받기

### 기본 상속

```python
class 자식클래스(부모클래스):
    ...
```

```python
class Sensor:                        # 부모 클래스
    def __init__(self, name):
        self.name = name

    def read(self):
        return 0.0                   # 기본 동작

    def describe(self):
        return f"센서: {self.name}"


class LidarSensor(Sensor):           # Sensor 를 상속
    def __init__(self, name, max_range):
        super().__init__(name)       # 부모의 __init__ 실행
        self.max_range = max_range

    def read(self):                  # 부모의 read 를 덮어씀 (오버라이드)
        import random
        return random.uniform(0, self.max_range)


class CameraSensor(Sensor):          # 다른 자식 클래스
    def __init__(self, name, resolution):
        super().__init__(name)
        self.resolution = resolution

    def read(self):
        return "image_data"          # 전혀 다른 동작
```

사용:
```python
lidar = LidarSensor("전방 라이다", 2.0)
cam = CameraSensor("중앙 카메라", (64, 64))

print(lidar.describe())    # "센서: 전방 라이다" (부모 메서드 그대로 사용)
print(lidar.read())        # 0.0 ~ 2.0 사이 랜덤값 (오버라이드된 메서드)
print(cam.read())          # "image_data"
```

### `super()` — 부모 메서드 호출

```python
class AdvancedLidar(LidarSensor):
    def __init__(self, name, max_range, noise_level):
        super().__init__(name, max_range)    # 부모(LidarSensor)의 __init__
        self.noise_level = noise_level

    def read(self):
        base = super().read()               # 부모의 read() 결과를 가져와서
        import random
        noise = random.uniform(-self.noise_level, self.noise_level)
        return base + noise                  # 노이즈를 추가
```

### 소스 코드에서의 인터페이스 패턴

`src/agent/agent_interface.py` 에는 추상적인 베이스 클래스가 있습니다:

```python
class SubagentInterface:
    def update(self, force_calculation=False) -> None:
        raise NotImplementedError

    def get_target_position(self):
        raise NotImplementedError

    def target_position_exists(self) -> bool:
        raise NotImplementedError
```

이것을 상속받는 클래스들:
```python
class GoToNonDiscoveredAgent(SubagentInterface):    # 미탐색 지역으로
    def update(self, ...): ...
    def get_target_position(self): ...
    def target_position_exists(self): ...

class FollowWallsAgent(SubagentInterface):           # 벽 따라가기
    def update(self, ...): ...
    def get_target_position(self): ...
    def target_position_exists(self): ...
```

`SubagentPriorityCombiner` 는 이 공통 인터페이스 덕분에
어떤 서브에이전트인지 모르고도 `.update()`, `.get_target_position()` 을 호출할 수 있습니다.

---

## 4. isinstance — 어떤 클래스인지 확인

```python
lidar = LidarSensor("라이다", 2.0)

print(isinstance(lidar, LidarSensor))  # True
print(isinstance(lidar, Sensor))       # True  (부모 클래스도 포함)
print(isinstance(lidar, CameraSensor)) # False
```

---

## ✏️ 실습

`03_practice.py` 를 열고 순서대로 진행하세요.

### 실습 1 — @property 로 좌표 관리
이동 범위를 제한하는 Robot 클래스를 property 로 구현합니다.

### 실습 2 — 상속으로 센서 계층 만들기
공통 Sensor 클래스를 만들고 LidarSensor, CameraSensor 를 상속으로 구현합니다.

### 실습 3 — rescue_robot.py 분석
property 목록을 정리하고 각각의 역할을 설명합니다.

---

## ✅ 이번 회차 체크리스트

- [ ] `_변수` 와 `__변수` 의 차이를 설명할 수 있다
- [ ] `@property` 를 사용해 읽기 전용 속성을 만들 수 있다
- [ ] `@setter` 로 값 검증을 추가할 수 있다
- [ ] `class 자식(부모):` 로 상속을 구현할 수 있다
- [ ] `super().__init__()` 이 언제 필요한지 안다
- [ ] `rescue_robot.py` 의 property 중 3개의 내부 동작을 설명할 수 있다
