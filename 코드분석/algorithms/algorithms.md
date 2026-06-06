# `algorithms/np_bool_array` 모듈 분석

## 1. 폴더/모듈 개요

`src/algorithms/np_bool_array`는 **numpy 불리언(bool) 배열로 표현된 픽셀 그리드 위에서 동작하는 그래프 탐색 알고리즘 모음**이다. 여기서 그리드의 한 셀이 `True`면 "통과 불가(traversable)"(벽/장애물 등), `False`면 "통과 가능"을 의미한다(파일들이 `not grid[...]`, `lambda x: not x` 형태로 통과 가능성을 판정한다 — `efficient_a_star.py:85`, `pathfinder.py:29`). 즉 이 모듈은 Mapper가 만든 픽셀 단위 지도(`pixel_grid.arrays["traversable"]`) 위에서 로봇이 어디로, 어떻게 이동할지 계산하는 **저수준 경로탐색 엔진**이다.

폴더는 두 종류의 알고리즘을 제공한다. (1) **A\*** — 시작점에서 목표점까지의 최단(가중) 경로를 구한다. `a_star.py`(리스트 기반, 느린 참조 구현)와 `efficient_a_star.py`(힙 기반, 실사용 구현) 두 버전이 있고 실제 시스템은 후자만 사용한다. (2) **BFS** — 목표 *좌표가 미리 정해지지 않은* 탐색, 즉 "조건을 만족하는 가장 가까운 셀"을 찾을 때 쓴다(가장 가까운 빈 공간, 가장 가까운 미탐색 영역, 특정 위치가 아직 도달 가능한지 등). 이 모듈은 규정상 "사전 매핑/데드레커닝 금지, 자율 탐색 필수"(규정 §LoP/제어)를 충족하는 자율 주행의 핵심으로, 상위의 `agent/pathfinding`과 `agent/subagents`가 직접 호출한다.

---

## 2. 파일별 상세 분석

### 2.1 `a_star.py` — 리스트 기반 A\* (참조용, 비권장)

#### 목적
가장 기본적인 형태의 A\* 구현. 코드 주석 자체가 "느림, `efficient_a_star.py`를 권장"이라고 명시한다(`a_star.py:25-26`). 실제 런타임에서는 import되지 않으며(상위 모듈은 모두 `efficient_a_star`만 import), 교육/참조 목적으로 남아 있는 구현으로 보인다.

#### 핵심 클래스/함수와 시그니처
- `class aStarNode` (`a_star.py:7`)
  - `__init__(self, parent=None, position=None)` — 필드: `parent`(역추적용 부모), `position`, `g`(시작점→현재 실제 비용), `h`(휴리스틱 추정), `p`(선호도 패널티), `f = g + h + p`(총 비용). (`a_star.py:8-14`)
  - `__eq__` — `position`이 같으면 동일 노드로 간주(`a_star.py:16-17`).
- `class aStarAlgorithm` (`a_star.py:22`)
  - `__init__` — `self.adjacents`는 4방향 이동 벡터 `[[0,1],[0,-1],[-1,0],[1,0]]`(대각선은 주석 처리), `self.preference_weight = 50`(`a_star.py:36-37`).
  - `get_preference(self, preference_grid, position)` — 해당 위치의 선호도 값을 반환, 배열 범위 밖이면 0(`a_star.py:39-46`).
  - `a_star(self, grid, start, end, preference_grid=None)` — start→end 경로를 좌표 리스트로 반환(`a_star.py:49`).

#### 사용 알고리즘 (구체적으로)
- **A\* 탐색**. open list / closed list를 **일반 파이썬 리스트**로 관리.
  - **노드 선택**: open list 전체를 선형 순회하며 `f`가 최소인 노드를 찾는다(`a_star.py:77-82`). 이 부분이 O(n) 이고, 매 반복마다 일어나므로 전체가 O(n²)이 된다(주석 `a_star.py:25`).
  - **휴리스틱 h**: **유클리드 거리의 제곱** — `(Δrow)² + (Δcol)²` (`a_star.py:120-121`). 주석은 "맨해튼 거리 제곱"이라 적었으나 실제 식은 두 축 차이의 제곱합(=유클리드 제곱)이다. 거리의 제곱은 admissible(과대평가 없음)이 아니므로 — 단위 이동 비용이 1인데 h는 제곱으로 부풀려진다 — 이 휴리스틱은 **최단 경로를 보장하지 않는다**(빠르지만 비최적). 이것이 `efficient_a_star.py`가 옥타일 휴리스틱으로 바꾼 이유이다.
  - **비용 g**: 한 칸 이동마다 +1 (`a_star.py:118`).
  - **선호도 패널티 p**: `get_preference(...) * preference_weight(=50)` (`a_star.py:124`). 벽 근처 셀일수록 높은 선호도 값을 가지게 만들어, 경로가 벽에서 멀어지도록 유도한다(중앙 주행 유도).
  - **중복 처리**: 자식이 closed list에 있으면 스킵, open list에 동일 위치가 있고 새 비용(`p+g`)이 더 크면 스킵(`a_star.py:113-132`).
- **4방향 이웃 확장**(상하좌우), 대각선 없음. 배열 범위/통과 가능 검사 후 자식 생성(`a_star.py:98-107`).

#### 사용 기능/라이브러리
- `numpy` — 그리드 배열.
- `cv2`(OpenCV) — **디버그 시각화**. 매 반복마다 open list 노드를 파란 점으로 찍고 `cv.imshow("debug", ...)` + `cv.waitKey(1)`을 호출한다(`a_star.py:139-145`). 이 코드가 항상 실행되므로(플래그 없음) 이 파일을 실사용하면 매 루프마다 창이 열려 **성능이 크게 저하**된다 — 비권장의 또 다른 이유.
- `math` — import되어 있으나 실제로 사용되지 않음.

#### 입력/출력 데이터 흐름
- 입력: `grid`(bool 통과불가 배열), `start`/`end`(배열 인덱스 `[row, col]`), `preference_grid`(선택, 선호도 값 배열).
- 출력: 시작→목표 좌표 리스트(`reverse`된 경로). 목표가 통과 불가이면 빈 리스트 반환(`a_star.py:62-64`). 경로 미발견 시 빈 리스트(`a_star.py:147`).

---

### 2.2 `efficient_a_star.py` — 힙 기반 A\* (실사용 구현)

#### 목적
`heapq`(이진 힙 우선순위 큐) 기반의 효율적 A\*. `PathFinder`(`pathfinder.py:8,27`)와 `PathTimeCalculator`(`path_time_calculator.py:8,23`)가 실제로 사용하는 핵심 경로탐색기이다.

#### 핵심 클래스/함수와 시그니처
- `class aStarNode` (`efficient_a_star.py:7`)
  - `__init__(self, location)` — `location`, `parent=None`, `g=float('inf')`(첫 방문 시 무조건 갱신되도록), `p=0`, `f=0` (`efficient_a_star.py:13-18`).
  - `__gt__(self, other)` — `self.f > other.f`. heapq가 노드끼리 비교할 때 `f`값 기준 정렬을 하도록 해준다(`efficient_a_star.py:20-22`).
- `class aStarAlgorithm` (`efficient_a_star.py:28`)
  - `__init__` — `adjacents` 4방향, `preference_weight = 2`(`efficient_a_star.py:44-45`).
  - `@staticmethod reconstructpath(node)` — 부모 체인을 따라 역추적 후 `reverse`(`efficient_a_star.py:47-55`).
  - `@staticmethod heuristic(start, target)` — 옥타일(Octile) 거리(`efficient_a_star.py:57-66`).
  - `@staticmethod get_preference(preference_grid, position)` — 선호도 값(범위 밖 0), `int()` 캐스팅(`efficient_a_star.py:68-76`).
  - `@staticmethod is_traversable(grid, position)` — 범위 내이면 `not grid[...]`, **범위 밖은 True(통과 가능) 처리**(`efficient_a_star.py:78-87`).
  - `a_star(self, grid, start, end, preference_grid=None, search_limit=float('inf'))` (`efficient_a_star.py:91`).

#### 사용 알고리즘 (구체적으로)
- **A\* + 이진 힙 우선순위 큐(heapq)**.
  - **노드 선택**: `heappop(openList)`으로 `f` 최소 노드를 O(log n)에 꺼낸다(`efficient_a_star.py:120`). 리스트 버전의 O(n) 선형 탐색 대비 본질적 개선.
  - **휴리스틱 h — 옥타일(Octile) 거리**: `min(dx,dy)*15 + |dx-dy|*10` (`efficient_a_star.py:64-66`). 직선 이동 비용 10, 대각선 비용 15(≈10·√2 근사)를 가정한 추정. 단, `adjacents`는 4방향뿐이라 실제 대각선 이동은 없다 — 주석도 이를 인정한다(`efficient_a_star.py:62`). 스케일을 10단위로 잡았으므로 g(한 칸당 +1)와 단위가 맞지 않아(h가 10배 스케일) 휴리스틱이 비용을 과대평가하여 **엄밀히는 admissible하지 않다**. 이는 최적성을 약간 희생하는 대신 목표 방향으로 강하게 끌어 탐색 노드 수를 줄이는(=속도 우선) **weighted/greedy A\*** 성격을 띤다. 그래도 `a_star.py`의 거리 제곱보다는 훨씬 완만해 경로 품질이 더 좋다.
  - **비용 g**: 한 칸 +1 (`efficient_a_star.py:140`).
  - **선호도 패널티 p**: `get_preference(...) * 2` (`efficient_a_star.py:144`). 가중치가 2로, `a_star.py`의 50보다 훨씬 낮다(주석 `efficient_a_star.py:38,45`). 즉 벽 회피 영향이 작아 더 직선적인 경로가 나온다.
  - **f**: `g + h + p` (`efficient_a_star.py:146`).
  - **중복/지연 삭제 처리**: 
    - `closed`(set)으로 이미 확정된 노드 재방문 차단(`efficient_a_star.py:114,122-125`).
    - `best_cost_for_node_lookup`(dict): 각 위치까지의 최적 비용 `g+p`를 추적. 새 자식의 `g+p`가 기존 기록보다 **작을 때만** 힙에 push(`efficient_a_star.py:150-157`). 처음 보는 위치도 push. 더 나쁜 비용의 옛 항목은 힙에 남지만, pop될 때 `closed`에 있으면 스킵되는 **lazy deletion(지연 삭제)** 방식이다(`efficient_a_star.py:122-123`).
- **4방향 이웃 확장**, 대각선 없음(`efficient_a_star.py:131-135`).
- **`search_limit`**: `loop_n`이 한계를 넘으면 탐색을 중단하고 빈 경로 반환(`efficient_a_star.py:159-162`). 시간 제한이 빡빡한 대회에서 한 번의 경로 계산이 메인 루프를 과도하게 점유하지 않도록 방어.

#### 사용 기능/라이브러리
- `numpy` — 그리드.
- `heapq`(`heappop`, `heappush`) — 우선순위 큐(`efficient_a_star.py:3`). 핵심 성능 요소.
- `cv2`, `math` — import되어 있으나 `a_star` 함수 본문에서 실제로는 사용되지 않는다(`debug_grid`만 선언되고 imshow는 호출 안 함, `efficient_a_star.py:92`). 즉 리스트 버전과 달리 디버그 창을 띄우지 않는다.

#### 입력/출력 데이터 흐름
- 입력: `grid`(bool 통과불가), `start`/`end`(튜플로 정규화), `preference_grid`, `search_limit`.
- 출력: start→end 위치 튜플 리스트. 목표가 통과 불가이거나 미발견/limit 초과 시 빈 리스트.
- 호출처(`pathfinder.py:111-114`): `grid=pixel_grid.arrays["traversable"]`, `preference_grid=pixel_grid.arrays["navigation_preference"]`를 넘긴다. 결과 경로는 array index → grid index 변환 → 시작점 제거(`[1:]`) → dither(2칸 1개) → `PathSmoother`로 평활화 후 사용.

---

### 2.3 `bfs.py` — BFS 계열 3종

#### 목적
A\*가 "정해진 좌표"로 가는 길을 찾는다면, BFS 계열은 **"조건을 만족하는 가장 가까운 셀이 어디인가"**를 찾는다(목표 좌표를 모를 때). 미탐색 영역 찾기, 빈 공간으로의 보정, 도달 가능성 판정 등에 쓰인다. BFS는 모든 간선 비용이 동일할 때 시작점에서 가장 가까운 노드부터 방문하므로 "가장 가까운 X"를 자연스럽게 보장한다.

#### 핵심 클래스/함수와 시그니처
세 클래스 모두 `adjacents`는 4방향, `get_neighbours(node)`는 4방향 이웃을 yield하는 제너레이터다.

1. **`class BFSAlgorithm`** (`bfs.py:5`)
   - `__init__(self, found_function)` — 목표 판정 함수만 받음(`bfs.py:13`).
   - `bfs(self, array, start_node)` — `found_function(value)`가 True인 첫 노드 반환(`bfs.py:23`).
   - **특징/주의**: **traversable 제약이 없다**(벽을 무시하고 모든 칸 확장). 더 심각하게 **closed set이 없고**, 중복 방지를 `if not n in open_list`(리스트 선형 탐색)로만 한다(`bfs.py:40`). 또 **배열 경계 검사가 없어** 범위 밖 인덱스 접근 시 예외 위험이 있다(`bfs.py:26` 주석도 명시). 그래서 안전한 시작점/조건에서만 써야 한다.

2. **`class NavigatingBFSAlgorithm`** (`bfs.py:44`) — 실사용 핵심
   - `__init__(self, found_function, traversable_function, max_result_number=1)` (`bfs.py:57`).
   - `bfs(self, found_array, traversable_array, start_node)` (`bfs.py:69`).
   - **특징**: `traversable_function`을 통과하는 셀로만 확장(`bfs.py:93`), `found_function`을 만족하면 결과에 추가, `max_result_number`개를 모으면 조기 반환(`bfs.py:99-102`). **`closed_set`으로 중복 방문 방지**(`bfs.py:80,105-108`), **배열 경계 검사 포함**(`bfs.py:89`). `found_array`와 `traversable_array`를 **분리**해서 받는 점이 핵심 — "통과 가능 영역만 따라가면서, 다른 기준으로 목표를 판정"할 수 있다.

3. **`class NavigatingLimitedBFSAlgorithm`** (`bfs.py:112`)
   - `__init__(self, found_function, traversable_function, max_result_number=1, limit=math.inf)` (`bfs.py:120`).
   - `NavigatingBFSAlgorithm`과 동일하되 `self.loops`가 `limit`을 넘으면 그때까지의 결과만 반환(`bfs.py:147-151`). `GoToFixturesAgent`에서 `limit=1000`으로 사용(주석 `bfs.py:118`, 실제 `go_to_fixtures_position_finder.py:25`). 매우 넓은 영역 탐색 시 프레임 시간 폭주를 막는 안전장치.

#### 사용 알고리즘 (구체적으로)
- **너비 우선 탐색(BFS)**: `open_list.pop(0)`(FIFO 큐) + 4방향 확장. 단위 비용 그래프에서 시작점으로부터의 최소 홉(hop) 순서로 방문 → 발견되는 첫 목표가 "가장 가까운 목표".
- **closed set 기반 중복 제거**(Navigating 계열): 노드를 큐에 넣는 순간 `closed_set`에 등록(`bfs.py:106-108`)해 재삽입을 막는다. `BFSAlgorithm`만 이게 없어 비효율/위험.
- **이중 배열 판정**(Navigating 계열): traversable 따라 확장 + found로 목표 판정을 분리.
- `pop(0)`은 리스트에서 O(n)이라 엄밀히는 deque가 더 빠르나, closed set으로 노드 수를 제한해 실용적으로 동작.

#### 사용 기능/라이브러리
- `numpy` — `found_array`/`traversable_array`.
- `math` — `NavigatingLimitedBFSAlgorithm`의 기본 `limit=math.inf`에 사용(`bfs.py:120`). `BFSAlgorithm`에서는 import만 되고 미사용.

#### 입력/출력 데이터 흐름
- `found_function`/`traversable_function`은 **호출 측이 lambda로 주입**한다. 예:
  - `pathfinder.py:29` — `NavigatingBFSAlgorithm(lambda x: x == 0, lambda x: True)`: 모든 곳을 통과 가능으로 보고 값이 0인(완전 빈) 가장 가까운 셀 찾기 → 시작/목표가 장애물 위일 때 보정(`pathfinder.py:213-223`).
  - `follow_walls_position_finder.py:26` — `NavigatingBFSAlgorithm(lambda x: x, lambda x: not x)`: 통과 가능(`not x`) 영역을 따라가며 True인 셀 탐색.
  - `go_to_non_discovered_position_finder.py:21,27` — 미탐색(`discovered=False`) 영역과 가장 가까운 빈 점 찾기.
- 출력: 위치(튜플) 또는 위치 리스트. `NavigatingBFSAlgorithm`은 항상 리스트(`results`)를 반환하므로 호출 측이 `[0]`으로 첫 결과를 꺼낸다(`pathfinder.py:221`).

---

## 3. 규정(2026) 연관성

- **자율 탐색·매핑(규정 §LoP/제어: 사전 매핑·데드레커닝 금지)**: A\*/BFS가 실시간 픽셀 지도 위에서 매 프레임 경로를 다시 계산해 자율 주행을 구현한다. 사전 정의 경로가 없다.
- **체크포인트/탈출 보너스(CN +10, EB +10%)**: 복귀(`ReturnToStart`) 및 시작 타일/exit 도달은 A\* 경로로 수행된다. `efficient_a_star`가 시작→목표 최단 경로를 제공한다.
- **늪/구멍/장애물 회피(swamp/hole/obstacle, LoP -5 회피)**: `traversable` 배열에서 통과 불가 셀을 회피하도록 4방향 확장 시 `is_traversable`로 차단(`efficient_a_star.py:134`, `bfs.py:93`). 늪 등 코스트가 다른 지형은 `navigation_preference`(선호도 패널티)로 우회를 유도한다(`p` 항).
- **벽 회피·주행 안정성(벽 두께 1cm, 노이즈 강건성)**: `preference_weight`로 벽 근처 경로에 패널티를 줘 통로 중앙으로 주행 → GPS 노이즈(v26 추가) 환경에서 벽 충돌 위험을 낮춘다.
- **시간 제한(시뮬 8분/실시간 10분)**: `efficient_a_star`의 힙 기반 O(log n) 선택과 `search_limit`, `NavigatingLimitedBFSAlgorithm`의 `limit`이 한 번의 계산이 메인 루프를 과점하지 않게 한다 → 시간 예산 보호.
- **매핑 보너스(MB)와의 관계**: 미탐색 영역 탐색(`go_to_non_discovered`)을 BFS로 안내해 맵 커버리지(정확도)를 높여 MB 획득에 기여한다.

> 주의: 이 모듈은 **순수 그래프 탐색**만 담당한다. Area별 격자/쿼터타일 구분, 토큰 식별(글자/해즈맷), 맵 행렬 정렬·90° 회전 비교 같은 규정 항목은 상위 모듈(`mapping`, `fixture_detection`, `FinalMatrixCreator`)의 책임이며 여기서 직접 다루지 않는다.

---

## 4. 다른 모듈과의 상호작용

### 이 모듈이 호출하는 것
- 외부 호출은 사실상 없다. `numpy`, `cv2`, `heapq`, `math` 표준/외부 라이브러리만 사용. `found_function`/`traversable_function`/`preference_grid`는 모두 **호출 측에서 주입**받으므로, 이 모듈은 의존성이 거의 없는 독립적 라이브러리다.

### 이 모듈을 호출하는 것 (호출처)
- **`agent/pathfinding/pathfinder.py`** (`:8-9,27,29`) — `efficient_a_star.aStarAlgorithm`로 주 경로 계산, `NavigatingBFSAlgorithm`으로 시작/목표 장애물 보정. 결과를 dither + `PathSmoother`로 다듬어 로봇 구동에 전달. **A\* 모듈의 가장 중요한 소비자.**
- **`agent/pathfinding/path_time_calculator.py`** (`:8-9,23,25`) — 동일한 A\*/BFS로 경로를 구해 **소요 시간 추정**(서브에이전트 우선순위/탈출 타이밍 판단 자료).
- **`agent/subagents/go_to_fixtures/go_to_fixtures_position_finder.py`** (`:5,25,27`) — `NavigatingLimitedBFSAlgorithm(limit=1000)`로 조난자 접근 후보 위치 탐색, `NavigatingBFSAlgorithm`으로 도달 가능성 확인.
- **`agent/subagents/go_to_non_discovered/go_to_non_discovered_position_finder.py`** (`:6,21,27`) — `NavigatingBFSAlgorithm`으로 가장 가까운 미탐색 영역, `BFSAlgorithm`으로 가장 가까운 빈 점 탐색.
- **`agent/subagents/follow_walls/follow_walls_position_finder.py`** (`:6,26,27`) — `NavigatingBFSAlgorithm`으로 벽 따라가기 다음 위치/도달 가능성 판정.

이들은 모두 `SubagentPriorityCombiner`(GoToFixtures > FollowWalls > GoToNonDiscovered) 및 복귀(ReturnToStart) 로직의 하부에서 동작한다(전체 아키텍처 §Agent).

---

## 5. `a_star.py` vs `efficient_a_star.py` 핵심 비교

| 항목 | `a_star.py` (리스트) | `efficient_a_star.py` (힙) |
|---|---|---|
| open list 자료구조 | 파이썬 리스트 | `heapq` 이진 힙 |
| 최소 f 노드 선택 | O(n) 선형 탐색 | O(log n) `heappop` |
| 전체 복잡도 | ~O(n²) | ~O(E log V) |
| 휴리스틱 h | 유클리드 거리의 제곱 (`Δr²+Δc²`) — 과대평가 큼, 비최적 | 옥타일 `min·15+|차|·10` — 더 완만, 그래도 g와 스케일 불일치로 약간 비admissible |
| 선호도 가중치 | `preference_weight = 50` (벽 회피 강함) | `preference_weight = 2` (직선적 경로) |
| 중복 노드 처리 | open/closed 리스트 선형 검사 | `closed` set + `best_cost_for_node_lookup` dict + lazy deletion |
| 노드 식별 | `__eq__`(위치 비교) | `__gt__`(f 비교, heapq용) |
| 탐색 한계 | 없음 | `search_limit` 지원 |
| 디버그 시각화 | **매 루프 `cv.imshow`(항상 켜짐 → 느림)** | 없음(`debug_grid` 선언만) |
| 실사용 여부 | **사용 안 함**(참조용) | **사용함**(pathfinder, path_time_calculator) |

요약: 두 구현은 같은 A\* 골격이지만 `efficient_a_star.py`는 자료구조(힙+dict+set), 휴리스틱(옥타일), 선호도 가중치(2), 디버그 제거, 탐색 한계 측면에서 대회용으로 최적화된 버전이다.

---

## 6. 발견한 주의점 (코드 품질 메모)

- `a_star.py:120-121`: 주석은 "맨해튼 거리 제곱"이라 적혀 있으나 식은 유클리드 거리의 제곱이다(주석-코드 불일치).
- `a_star.py:143-145`: `cv.imshow`/`cv.waitKey`가 무조건 실행 — 이 파일을 실수로 import해 쓰면 매 루프 창이 떠 큰 성능 저하. 실사용 금지 이유.
- `bfs.py:23-41` `BFSAlgorithm`: closed set·경계 검사 부재 + `n in open_list`(O(n)) 중복 검사로 큰 배열에서 느리고 인덱스 예외 위험. 통과 제약도 없음 → 안전한 입력에서만 사용.
- `efficient_a_star.py`의 휴리스틱은 g(단위 1)와 스케일이 10:1로 불일치 → 엄밀한 최단 경로 보장은 아님(속도 우선의 의도적 트레이드오프로 해석됨).
