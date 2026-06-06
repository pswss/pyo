# 2회차 — 모듈, 임포트, f-string

> 목표: 코드를 파일로 나누고, 다른 파일에서 불러와 쓸 수 있다.
>       f-string 으로 원하는 형식의 출력을 만들 수 있다.
> 실습 파일: `02_practice.py`

---

## 1. 모듈이란?

**모듈** = 파이썬 코드가 담긴 `.py` 파일 하나

소스 코드를 한 파일에 모두 쓰면:
- 수천 줄이 되어 찾기 어려움
- 여러 사람이 동시에 작업하기 어려움
- 같은 코드를 다른 프로젝트에서 재사용하기 어려움

`src/` 폴더를 보면 기능별로 파일이 나뉘어 있습니다:
```
src/
  rescue_robot.py        ← 학생용 API
  flags.py               ← 설정값 모음
  map_visualizer.py      ← 시각화
  mapping/
    mapper.py            ← 지도 관리
    fixture_mapper.py    ← 조난자 위치
  agent/
    agent.py             ← 탐색 AI
    pathfinding/
      pathfinder.py      ← A* 경로 찾기
```

---

## 2. import 기본

### 방법 1: 모듈 전체 임포트

```python
import math              # math 모듈 전체를 가져옴
import time

result = math.sqrt(4)    # math. 을 앞에 붙여서 사용
print(math.pi)           # 3.14159...
time.sleep(0.1)
```

### 방법 2: 특정 이름만 임포트

```python
from math import sqrt, pi   # sqrt 와 pi 만 가져옴

result = sqrt(4)            # math. 없이 바로 사용
print(pi)
```

### 방법 3: 별칭(alias) 붙이기

```python
import numpy as np          # numpy 를 np 라는 짧은 이름으로
import cv2 as cv            # cv2 를 cv 로

arr = np.array([1, 2, 3])   # np.array 로 사용
```

---

## 3. 패키지 — 폴더를 모듈로

폴더에 `__init__.py` 파일이 있으면 그 폴더가 **패키지** 가 됩니다.

```
src/
  mapping/
    __init__.py          ← 이 파일이 있어야 패키지로 인식
    mapper.py
    fixture_mapper.py
```

임포트할 때:
```python
from mapping.mapper import Mapper
#    ↑폴더    ↑파일    ↑클래스이름
```

소스 코드 실제 예시 — `src/executor/executor.py`:
```python
from data_structures.angle import Angle          # data_structures/angle.py 의 Angle 클래스
from mapping.mapper import Mapper                # mapping/mapper.py 의 Mapper 클래스
from agent.agent import Agent                    # agent/agent.py 의 Agent 클래스
from flags import SHOW_DEBUG, DO_SLOW_DOWN       # flags.py 의 변수 두 개
```

---

## 4. 내가 만든 파일을 임포트하기

`robot_config.py` 라는 파일을 만들고:
```python
# robot_config.py
MAX_SPEED = 1.0
TILE_SIZE = 0.12      # 12cm
TIME_LIMIT = 8 * 60  # 480초

def get_speed_ratio(desired, maximum=MAX_SPEED):
    return min(desired / maximum, 1.0)
```

다른 파일에서:
```python
# main.py (같은 폴더에 있을 때)
from robot_config import MAX_SPEED, TILE_SIZE, get_speed_ratio

print(TILE_SIZE)                  # 0.12
print(get_speed_ratio(0.5))       # 0.5
```

---

## 5. flags.py 패턴

`src/flags.py` 는 프로그램 전체의 설정값을 한 곳에 모아둔 파일입니다:

```python
# flags.py
SHOW_LIVE_MAP = 1       # 0=끔, 1=켬
SHOW_DEBUG = 0
DO_SLOW_DOWN = 0
```

다른 파일들이 이 값을 가져와서 if 문으로 분기합니다:

```python
# map_visualizer.py
from flags import SHOW_LIVE_MAP   # 설정값 가져오기

if SHOW_LIVE_MAP:                 # 1이면 True
    visualizer = MapVisualizer()
```

**장점**: 설정을 바꾸고 싶을 때 `flags.py` 한 파일만 수정하면 됩니다.

---

## 6. f-string 완전 정복

### 기본: 변수 삽입

```python
name = "로봇1"
x = 0.123456
print(f"이름: {name}, 위치: {x}")
# → 이름: 로봇1, 위치: 0.123456
```

### 소수점 자릿수 지정

```python
x = 0.123456
print(f"{x:.1f}")   # 0.1    (소수점 1자리)
print(f"{x:.2f}")   # 0.12   (소수점 2자리)
print(f"{x:.4f}")   # 0.1235 (소수점 4자리, 반올림)
```

### 정수 자릿수(패딩)

```python
n = 7
print(f"{n:3d}")    # "  7"  (3자리, 앞에 공백)
print(f"{n:03d}")   # "007"  (3자리, 앞에 0)
```

### 계산식도 가능

```python
speed = 0.75
print(f"속도: {speed * 100:.0f}%")   # 속도: 75%
```

### 실제 소스에서의 사용

`src/executor/executor.py`:
```python
print(f"[탐색:executor.state_explore] ★ fixture 감지! "
      f"글자='{self.letter_to_report}', "
      f"방향={self.report_orientation.degrees:.1f}°, "
      f"위치=({self.robot.position.x:.3f},{self.robot.position.y:.3f})m "
      f"→ report_fixture 전환")
```

출력 예:
```
[탐색:executor.state_explore] ★ fixture 감지! 글자='H', 방향=92.3°, 위치=(0.240,0.360)m → report_fixture 전환
```

---

## 7. 순환 임포트 주의

A가 B를 임포트하고, B도 A를 임포트하면 오류가 납니다:
```
# 나쁜 예
# a.py: from b import B
# b.py: from a import A   ← 순환!
```

소스에서는 이를 피하기 위해 `flags.py` 처럼 공통 설정을 별도 파일로 분리합니다.

---

## ✏️ 실습

`02_practice.py` 파일에서 순서대로 진행하세요.

### 실습 1 — 설정 파일 만들기
`flags.py` 처럼 로봇 설정값을 담은 `02_robot_config.py` 를 만들고 임포트합니다.

### 실습 2 — f-string 로그 출력
실제 소스처럼 `[모듈명:함수명]` 형식의 로그를 f-string 으로 작성합니다.

### 실습 3 — 모듈 구조 파악
`src/` 폴더를 보며 어떤 폴더/파일이 있는지 지도를 그려봅니다.

---

## ✅ 이번 회차 체크리스트

- [ ] `import 모듈` 과 `from 모듈 import 이름` 의 차이를 안다
- [ ] 폴더가 패키지가 되려면 무엇이 필요한지 안다
- [ ] `from mapping.mapper import Mapper` 를 읽고 어느 파일의 무엇을 가져오는지 알 수 있다
- [ ] f-string 으로 소수점 자릿수를 지정해 출력할 수 있다
- [ ] `src/flags.py` 의 역할을 설명할 수 있다
