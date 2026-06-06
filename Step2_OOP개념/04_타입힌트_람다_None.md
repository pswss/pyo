# 4회차 — 타입 힌트 · 람다 · None 패턴

> 목표: 타입 힌트가 붙은 코드를 읽을 수 있다.
>       lambda 로 간단한 익명 함수를 만들고 다른 함수에 전달할 수 있다.
>       None 을 안전하게 다루는 패턴을 안다.
> 실습 파일: `04_practice.py`

---

## 1. 타입 힌트 (Type Hints)

### 타입 힌트란?

변수와 함수의 매개변수·반환값에 **예상 타입을 명시**하는 문법입니다.
파이썬 실행에는 영향을 주지 않지만, 코드를 읽는 사람에게 중요한 정보를 줍니다.

```python
# 타입 힌트 없음 (무엇을 넣어야 하는지 불명확)
def move(target, speed):
    ...

# 타입 힌트 있음 (명확!)
def move(target: tuple, speed: float) -> bool:
    ...
```

### 기본 타입 힌트

```python
x: int = 3
name: str = "로봇"
speed: float = 0.75
active: bool = True

def greet(name: str) -> str:
    return f"안녕, {name}!"

def add(a: int, b: int) -> int:
    return a + b
```

### 컬렉션 타입

```python
from typing import List, Dict, Tuple, Optional

positions: List[float] = [0.1, 0.2, 0.3]
config: Dict[str, int] = {"speed": 1, "timeout": 30}
coord: Tuple[float, float] = (0.12, 0.48)

def find_path(start: Tuple, end: Tuple) -> List[Tuple]:
    ...
```

### None 이 될 수 있는 타입: Optional

```python
from typing import Optional

# Optional[float] = float 또는 None
def measure_distance() -> Optional[float]:
    if sensor_ready:
        return 1.5
    return None   # 측정 불가 시 None 반환
```

### 소스 코드에서 읽는 법

```python
# src/rescue_robot.py
def go_to(self, x: float, y: float) -> bool:
#                ↑타입    ↑타입     ↑반환타입
#         float 두 개를 받아서 bool 을 반환한다는 뜻

def get_camera_image(self, camera: str = "center"):
#                          ↑str 타입  ↑기본값 "center"
```

```python
# src/agent/agent.py
def get_target_position(self) -> Position2D:
#                                ↑ 사용자 정의 클래스도 타입이 될 수 있음

def __init__(self, mapper: Mapper) -> None:
#                          ↑Mapper 클래스  ↑반환 없음
```

---

## 2. 람다 (Lambda) — 이름 없는 함수

### 기본 문법

```python
# 일반 함수
def double(x):
    return x * 2

# 람다로 같은 기능
double = lambda x: x * 2

print(double(5))   # 10
```

문법: `lambda 매개변수: 반환값`

조건문도 가능:
```python
is_positive = lambda x: x > 0
clamp = lambda x, lo, hi: max(lo, min(hi, x))

print(is_positive(-3))       # False
print(clamp(1.5, 0.0, 1.0))  # 1.0
```

### 함수를 다른 함수에 전달할 때

람다의 주된 용도는 **다른 함수에 동작을 전달**하는 것입니다.

```python
# sorted 에 정렬 기준 함수를 전달
victims = [("H", 0.5), ("S", 0.2), ("U", 0.8)]

# x 좌표(인덱스 1) 기준으로 정렬
sorted_victims = sorted(victims, key=lambda v: v[1])
print(sorted_victims)   # [("S", 0.2), ("H", 0.5), ("U", 0.8)]
```

```python
# filter: 조건에 맞는 것만 골라내기
distances = [0.5, 0.1, 1.8, 0.08, 2.1]
close_ones = list(filter(lambda d: d < 0.2, distances))
print(close_ones)   # [0.1, 0.08]

# map: 각 요소를 변환
in_cm = list(map(lambda d: d * 100, distances))
print(in_cm)        # [50.0, 10.0, 180.0, 8.0, 210.0]
```

### 소스 코드에서의 람다

`src/agent/subagents/go_to_non_discovered/go_to_non_discovered_position_finder.py`:
```python
self.closest_unseen_finder = NavigatingBFSAlgorithm(
    found_function=lambda x: x == False,     # discovered=False 인 곳을 목표로
    traversable_function=lambda x: x == False,  # traversable=False 인 곳만 통과
    max_result_number=1
)
```

BFS 알고리즘에 **"어느 칸이 목표인지"** 와 **"어느 칸을 통과할 수 있는지"**
를 람다로 전달합니다. BFS 코드 자체는 바꾸지 않고 동작만 교체할 수 있습니다.

`follow_walls_position_finder.py`:
```python
self.__next_position_finder = NavigatingBFSAlgorithm(
    lambda x: x,        # fixture_distance_margin=True 인 곳을 목표로
    lambda x: not x     # traversable=False (not False = True) 인 곳만 통과
)
```

---

## 3. None 패턴

### None 이란?

**값이 없음** 을 나타내는 특별한 값입니다.

```python
result = None    # 아직 계산하지 않음
target = None    # 목표 없음
image = None     # 아직 캡처하지 않음
```

### None 체크 — `is None` vs `== None`

```python
# 올바른 방법
if value is None:      # is 로 비교 (권장)
    ...

if value is not None:  # None 이 아닌 경우
    ...

# 나쁜 방법 (동작은 하지만 권장 안 함)
if value == None:      # == 로 비교하면 안 됨
    ...
```

### 소스 코드의 None 패턴들

**초기화**: 나중에 설정될 값을 None 으로 초기화
```python
# src/map_visualizer.py
self._path: list = []
self._target = None      # 아직 목표 없음

# src/agent/agent.py
self.__target_position = None   # 아직 계산 안 됨
```

**체크 후 사용**:
```python
# src/map_visualizer.py
if self._mapper.robot_position is None:   # 위치가 없으면 렌더링 안 함
    return

if self._target is not None:              # 목표가 있을 때만 그리기
    tgt = (int(self._target[1]), int(self._target[0]))
    cv.circle(image, tgt, 5, _COLORS["target"], 1)
```

**None 반환 — "결과 없음" 신호**:
```python
# src/fixture_detection/fixture_clasification.py
def classify_fixture(self, fixture) -> str:
    ...
    elif final_fixture_filter.fixture_type == "already_detected":
        letter = None    # None 반환 = "이미 보고된 곳, 무시하세요"
    ...
    return letter
```

호출하는 쪽:
```python
# src/executor/executor.py
detected_letter = self.fixture_detector.classify_fixture(fixtures[0])

if detected_letter is None:    # None 이면 이미 보고된 것 → 건너뜀
    continue

self.letter_to_report = detected_letter
```

**Short-circuit 패턴** — None 이면 즉시 반환:
```python
# src/rescue_robot.py
def go_to_start(self) -> bool:
    sp = self._mapper.start_position
    if sp is None:          # 시작 위치가 등록 안 됐으면
        return False        # 바로 False 반환 (이후 코드 실행 안 함)
    return self._robot.move_to_coords((sp.x, sp.y))
```

---

## 4. 딕셔너리 고급 활용

소스에서 딕셔너리를 설정·상태 관리에 적극 활용합니다.

### get() — 키가 없어도 안전하게

```python
config = {"speed": 0.8, "timeout": 30}

# 키가 없으면 오류
# config["missing"]   → KeyError

# 키가 없으면 기본값 반환
val = config.get("missing", 0)   # 0
val = config.get("speed", 1.0)   # 0.8 (있으므로 해당 값)
```

### 색상표로 쓰는 패턴

```python
# src/map_visualizer.py
_COLORS = {
    "wall":     (220, 220, 220),
    "robot":    (0, 0, 255),
    "victim":   (0, 165, 255),
}

# 색상 이름으로 접근
cv.circle(image, pt, 4, _COLORS["robot"], -1)
```

---

## ✏️ 실습

`04_practice.py` 를 열고 순서대로 진행하세요.

### 실습 1 — 타입 힌트 읽기
소스 코드의 함수 시그니처를 보고 매개변수와 반환값을 설명합니다.

### 실습 2 — 람다 활용
센서 데이터를 필터링하고 변환하는 람다를 작성합니다.

### 실습 3 — None 안전 처리
None 이 올 수 있는 함수에서 안전하게 처리하는 패턴을 구현합니다.

---

## ✅ 이번 회차 체크리스트

- [ ] 타입 힌트가 붙은 함수 시그니처를 읽고 의미를 설명할 수 있다
- [ ] `Optional[X]` 가 무엇을 뜻하는지 안다
- [ ] `lambda x: x > 0` 같은 간단한 람다를 작성할 수 있다
- [ ] `sorted(..., key=lambda ...)` 패턴을 사용할 수 있다
- [ ] `if value is None:` 과 `if value is not None:` 을 적절히 사용한다
- [ ] 소스의 `classify_fixture` 가 None 을 반환하는 이유를 설명할 수 있다
