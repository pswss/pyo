# `agent` 모듈 상세 분석 — RoboCup Rescue Simulation 2026

> 분석 대상: `src/agent/` 전체 (최상위 + `pathfinding/` + `subagents/*`)
> 대상 독자: RoboCupJunior Rescue Simulation 2026 경쟁자

---

## 1. 폴더/모듈 개요

`agent` 모듈은 로봇의 **"두뇌"(의사결정 계층)** 다. 센서로 만든 지도(`Mapper`)와 실제 모터 제어(`Robot`/`drive_base`) 사이에 위치하며, **"지금 로봇이 어느 좌표로 가야 하는가?"** 라는 단 하나의 질문에 답한다. `Executor`(메인 루프 상태머신)가 매 타임스텝 `Agent.update()`를 호출하고 `Agent.get_target_position()`으로 다음 목표 좌표 1개를 받아 로봇을 그쪽으로 몰고 간다.

이 모듈은 3개 계층으로 구성된다.

1. **최상위 `Agent`** — 2단계(`explore` ↔ `return_to_start`) 상태머신. 탐색 중에는 `SubagentPriorityCombiner`로 여러 전략을 우선순위대로 시도하고, 탐색이 끝나면 시작점 복귀로 전환한다.
2. **서브에이전트(`subagents/*`)** — 각각 하나의 전략을 담당. "어떤 좌표를 목표로 삼을지" 정하는 **PositionFinder**(목표 탐색기, BFS 계열)와 "거기까지 어떻게 갈지" 정하는 **PathFinder**(A* 경로) 쌍으로 구성된다.
3. **경로 탐색(`pathfinding/`)** — A*로 픽셀 그리드 위 경로를 계산하고(`pathfinder.py`), 경로를 부드럽게 다듬으며(`path_smoothing.py`), 경로 길이로 예상 소요를 추정한다(`path_time_calculator.py`).

핵심 데이터 구조는 `Mapper`가 들고 있는 **`pixel_grid`(CompoundExpandablePixelGrid)** 다. 이는 여러 개의 불리언/값 레이어(`traversable`, `navigation_preference`, `discovered`, `traversed`, `robot_center_traversed`, `victims`, `checkpoints`, `fixture_distance_margin`, `swamps`)를 겹쳐 놓은 다층 그리드이며, 에이전트 전체가 이 레이어들을 읽어 결정을 내린다.

---

## 2. 최상위 파일 분석

### 2.1 `agent_interface.py`

**목적**: 에이전트 계층의 추상 인터페이스(ABC) 3종을 정의해 다형성을 보장한다.

**핵심 클래스/시그니처**:
- `AgentInterface(ABC)` — 최상위 에이전트 계약. `update() -> None`, `get_target_position() -> Position2D`, `do_end() -> bool`.
- `SubagentInterface(ABC)` — 전략 서브에이전트 계약. `update(force_calculation=False)`, `get_target_position()`, `target_position_exists() -> bool`.
- `PositionFinderInterface(ABC)` — 목표 위치 탐색 알고리즘 계약. `update(force_calculation=False)`, `get_target_position()`, `target_position_exists()`.

**사용 알고리즘/기능**: 순수 추상 기반 클래스 패턴(`abc.ABC` + `@abstractmethod`). `random`, `Position2D`를 import하지만 인터페이스 정의 자체에는 알고리즘이 없다.

**데이터 흐름**: 구현체(`Agent`, `FollowWallsAgent` 등)가 이 계약을 따르므로 `SubagentPriorityCombiner`가 서브에이전트 종류를 몰라도 동일한 메서드로 호출할 수 있다.

**규정 연관성**: 직접 구현하는 규정 항목은 없음(아키텍처 골격).

**상호작용**: 모든 서브에이전트/PositionFinder가 상속. `Agent`가 `AgentInterface`를 구현.

---

### 2.2 `agent.py`

**목적**: 모듈 전체의 진입점. (1) 탐색 전략 우선순위 결합기 `SubagentPriorityCombiner`, (2) 2단계 상태머신 최상위 `Agent`를 정의한다.

#### `SubagentPriorityCombiner(SubagentInterface)`

**핵심 로직** (`agent.py:34-47`): 생성자에서 받은 에이전트 리스트를 **우선순위 순서대로** 순회하며, `target_position_exists()`가 True인 **첫 번째** 에이전트를 선택하고 즉시 `break`. 우선순위(높은 순):

1. `GoToFixturesAgent` — 조난자(벽 마커)·체크포인트 접근 (최우선)
2. `FollowWallsAgent` — 벽 근처를 따라가 조난자 발견 확률 높이기
3. `GoToNonDiscoveredAgent` — 미탐색 지역으로 이동 (최후 수단)

**알고리즘**: 우선순위 기반 폴백 체인(priority fallback chain). `__agent_changed()`(`agent.py:56-58`)로 선택된 에이전트가 이전 프레임과 바뀌면 `force_calculation=True`를 다음 `update`에 주입해 새 전략이 곧장 새 목표를 재계산하게 한다.

**입출력**: 입력=각 서브에이전트 상태, 출력=현재 선택된 에이전트의 `get_target_position()`.

#### `Agent(AgentInterface)`

**핵심 구성** (`agent.py:71-101`):
- `__navigation_agent`: 위 3개를 담은 `SubagentPriorityCombiner`.
- `__return_to_start_agent`: `ReturnToStartAgent`.
- `__stage_machine`: `StateMachine("explore", ...)` — 상태 `explore`(→`return_to_start` 전환 가능)와 `return_to_start`.
- `end_reached_distance_threshold = 0.04` (m): 시작점 4cm 이내면 복귀 완료.
- `max_time = 8*60` (480초): 규정상 시뮬레이션 8분.
- `__no_target_threshold = 120`: 탐색 목표가 **120프레임 연속** 없으면 탐색 종료로 간주.
- `PathTimeCalculator(mapper, 0.06, 0.01)` + `StepCounter(300)`: 경로 시간 추정기와 300스텝 카운터(생성은 되나 본문에서 직접 활용하는 흐름은 없고, 향후 시간 기반 의사결정 훅으로 보임).

**상태머신 동작**:
- `__stage_explore` (`agent.py:116-128`): 내비게이션 에이전트 갱신 → 목표 없으면 `__no_target_consecutive` 증가, 임계치 도달 시 `return_to_start`로 전환. 목표 있으면 카운터 리셋 후 `__target_position` 갱신.
- `__stage_return_to_start` (`agent.py:130-141`): `ReturnToStartAgent`로 시작점 경로 계산, 목표 갱신. 200스텝마다 잔여 거리 로그.
- `__set_force_calculation` (`agent.py:143-145`): 상태머신이 상태 전환 콜백으로 호출 → 다음 프레임 강제 재계산 플래그 ON.

**`do_end()`** (`agent.py:111-114`): `state == "return_to_start"` **그리고** 로봇~시작점 거리 < 0.04m → True. Executor가 이를 받아 미션 종료(`end`)로 넘어간다.

**사용 알고리즘/기능**: 유한 상태머신(FSM), 우선순위 폴백, 히스테리시스(120프레임 연속 조건으로 깜빡임 방지). `numpy`, `Position2D`, `StateMachine`, `StepCounter`.

**입출력**: 입력=`Mapper`(로봇 위치, 시작 위치, pixel_grid). 출력=`get_target_position()` → 좌표 1개.

**규정 연관성**:
- **탈출 보너스(EB)** 직결: 복귀 단계와 4cm 도달 판정이 "시작 타일 복귀" 조건을 구현.
- **시간 제한 480초**: `max_time`로 명시.
- 조난자 우선 접근 우선순위는 **토큰 식별(TI/TT) 점수** 극대화 전략.

**상호작용**: `Executor`가 호출(`update`/`get_target_position`/`do_end`). 내부적으로 4개 서브에이전트와 `PathTimeCalculator`를 소유. `Mapper`를 읽는다.

---

## 3. `pathfinding/` 섹션 — 경로 탐색

이 하위 폴더는 "목표 좌표가 주어졌을 때 픽셀 그리드 위에서 어떻게 갈지"를 담당한다. 모든 서브에이전트가 공유한다.

### 3.1 `pathfinder.py` — A* 경로 추종기

**목적**: 로봇 현재 위치 → 목표 위치까지의 A* 경로를 계산·관리하고, 매 프레임 "다음에 향할 노드 좌표"를 반환한다.

**핵심 클래스**: `PathFinder` (`pathfinder.py:15`).

**생성자 구성** (`pathfinder.py:26-44`):
- `aStarAlgorithm()` — 힙 기반 A*.
- `NavigatingBFSAlgorithm(lambda x: x==0, lambda x: True)` — 시작/목표가 장애물 위일 때 가장 가까운 빈칸을 찾는 보정용 BFS.
- `PathSmoother(1)` — 평활화 강도 1.
- 상태 변수: `__a_star_path`(원본 grid index 경로), `__smooth_astar_path`(평활화 경로), `__a_star_index`(현재 추종 인덱스), `path_not_found`, `__position_changed`.

**`update()`** (`pathfinder.py:46-90`) — 재계산 트리거 4종:
1. 경로 완료(`is_path_finished()`), 2. 경로 막힘(`__is_path_obstructed()`), 3. 목표 변경(`__position_changed`), 4. 강제(`force_calculation`).
이후 `__calculate_path_index()`로 인덱스를 전진하고, 디버그/시각화(cv.imshow, `visualizer.set_path/set_target`)를 갱신한다. 매 호출 시 `pixel_grid.expand_to_grid_index`로 그리드를 자동 확장한다(맵이 동적으로 커지는 구조).

**`__calculate_path()`** (`pathfinder.py:93-132`):
- 시작/목표 array index를 구하고, 장애물 위면 `__get_closest_traversable_array_index`(BFS)로 보정.
- `self.__a_star.a_star(traversable배열, start, target, navigation_preference배열)` 호출.
- 결과가 2노드 이상이면 grid index로 변환·저장하되 **첫 노드(현재 위치) 제거**(`[1:]`). 실패 시 `path_not_found = True`.
- `__dither_path`로 **2칸당 1노드** 희소화(마지막=목표 노드는 항상 보존, `pathfinder.py:150-169`) → `PathSmoother.smooth`로 평활화.

**`__calculate_path_index()`** (`pathfinder.py:134-148`): 로봇이 현재 목표 노드에 **3픽셀 이내** 접근하면 인덱스 +1(다음 노드로 전진).

**`__is_path_obstructed()`** (`pathfinder.py:185-206`): 경로 노드 중 하나라도 `traversable=True`(=장애물)면 막힘 판정. 새로 발견된 벽/장애물에 대응하는 재계획 트리거.

**`get_next_position()`** (`pathfinder.py:171-183`): 현재 인덱스의 평활화 좌표를 m 단위 `Position2D`로 반환. 경로 없으면 로봇 현재 위치 반환(제자리).

**사용 알고리즘**: **A\*** (휴리스틱 경로 탐색), **BFS**(장애물 위 시작/목표 보정), 경로 디더링(다운샘플링), 경로 평활화(가중 평균).

**사용 라이브러리**: `numpy`, `cv2`(디버그 시각화), `Position2D`, `CompoundExpandablePixelGrid`, `aStarAlgorithm`, `NavigatingBFSAlgorithm`, `PathSmoother`, `flags`.

**입출력**: 입력=목표 좌표(`np.ndarray`) + pixel_grid의 `traversable`/`navigation_preference`. 출력=다음 노드 좌표.

**규정 연관성**: 자율 제어(사전 매핑·데드레커닝 금지) 하에서 **늪/구멍/장애물·벽 회피 주행**을 실현. `navigation_preference`로 벽 근처를 피해 안전 마진 확보(LoP 위험·끼임 감소).

**상호작용**: 4개 서브에이전트 전부가 인스턴스를 소유하고 `update`/`get_next_position`/`is_path_finished`/`path_not_found`를 호출.

#### A* 내부(`algorithms/np_bool_array/efficient_a_star.py`) 보충

PathFinder가 쓰는 A*는 **heapq 우선순위 큐 기반**(`efficient_a_star.py:118-164`)이다. `f = g + h + p`로 정렬하며:
- `g`: 시작점부터 실제 비용(스텝당 +1).
- `h`: **옥타일(Octile) 휴리스틱** (`heuristic`, `efficient_a_star.py:57-66`) — `min(dx,dy)*15 + |dx-dy|*10`. 실제 이동은 4방향(상하좌우, `adjacents`, line 44)뿐이지만 더 타이트한 h를 제공.
- `p`: `navigation_preference` 값 × `preference_weight=2` — **벽 근처 회피 패널티**.
- `best_cost_for_node_lookup` 딕셔너리 + `closed` 집합으로 중복 노드 지연 삭제(lazy deletion). 목표가 장애물 위면 빈 경로 반환(line 104-106), 배열 범위 밖은 통과 가능 처리(line 84-87).

### 3.2 `path_smoothing.py` — 경로 평활화

**목적**: A* 경로의 날카로운 꺾임을 완화해 로봇이 부드럽게 주행하도록 노드 좌표를 재배치한다.

**핵심 클래스**: `PathSmoother(strenght)` (`path_smoothing.py:1`, 변수명 오타 `strenght`).

**알고리즘** (`smooth`, `path_smoothing.py:13-30`): 각 노드를 이전/다음 노드와 가중 평균.
`avg = (node + prior*strength + next*strength) / (1 + strength*2)` (x, y 각각). 첫/마지막 노드는 자기 자신을 이웃으로 써서 끝점이 거의 유지된다. `strength=0`이면 변화 없음, 클수록 더 부드러움. PathFinder는 `strength=1` 사용.

**라이브러리**: 순수 Python(외부 의존 없음).

**입출력**: 입력=노드 리스트 `[[x,y],...]`, 출력=평활화된 `[[avg_x,avg_y],...]`.

**규정 연관성**: 간접적. 부드러운 경로 → 늪/장애물 사이 주행 안정성, 끼임·LoP 위험 감소.

**상호작용**: `PathFinder.__calculate_path`가 호출.

### 3.3 `path_time_calculator.py` — 경로 시간 추정

**목적**: 목표까지 A* 경로 길이를 구해 **비선형 예상 비용**을 산출한다. 에이전트 간 우선순위/시간 예산 판단용 유틸리티.

**핵심 클래스**: `PathTimeCalculator(mapper, factor, exponent)` (`path_time_calculator.py:13`). `Agent`는 `factor=0.06, exponent=0.01`로 생성.

**알고리즘** (`calculate`, line 33-39): `cost = n*factor + n^exponent` (n=경로 노드 수). 경로 길이 계산은 `__calculate_path_lenght`(line 41-65)에서 동일한 A*(navigation_preference 포함)로 수행하며, 시작/목표가 장애물 위면 BFS로 보정한다. PathFinder와 거의 같은 A* 호출이지만 **경로 자체가 아니라 길이(len)만** 반환한다.

**라이브러리**: `numpy`, `cv2`(import만), `aStarAlgorithm`, `NavigatingBFSAlgorithm`.

**입출력**: 입력=목표 좌표, 출력=실수 비용값.

**규정 연관성**: **480초 시간 예산** 관리·우선순위 비교 보조(어느 목표가 시간 대비 가치가 높은지 가늠).

**상호작용**: `Agent`가 인스턴스를 소유. (현재 코드에서 호출 흐름은 생성에 머무는 상태라 향후 확장 훅으로 해석.)

---

## 4. `subagents/` 섹션 — 전략별 서브에이전트

각 서브에이전트는 **PositionFinder(목표 결정) + PathFinder(경로) 쌍** 구조다. `update()`에서 PositionFinder로 목표 좌표를 정한 뒤 PathFinder에 넘겨 경로를 갱신하고, `get_target_position()`은 PathFinder의 다음 노드를 돌려준다.

공통적으로 PositionFinder는 다음 3조건 중 하나라도 만족하면 목표를 **재탐색**한다(`go_to_fixtures`/`follow_walls`):
- 목표가 없음 / 목표가 더 이상 도달 불가(`__is_grid_index_still_reachable`) / 로봇 중심이 이미 지나침(`__already_passed_through_grid_index`) / 강제.

### 4.1 `go_to_fixtures/` — 조난자·체크포인트 접근 (우선순위 1)

#### `go_to_fixtures_subagent.py`
**목적**: 감지된 조난자(victim)·체크포인트 근처로 이동(최우선 전략).
**클래스**: `GoToFixturesAgent(SubagentInterface)`. `PositionFinder` + `PathFinder` 보유. `update`(line 26-32)는 PositionFinder 갱신 후 목표 존재 시 PathFinder에 `np.array(target)` 전달.

#### `go_to_fixtures_position_finder.py`
**목적**: `victims` 레이어 주변의 **도달 가능 + 미방문** 후보를 BFS로 선택.

**핵심 알고리즘**:
- **Zone of influence(영향권)** (`__get_fixtures_zone_of_influence`, line 106-115): `victims` 배열에 **원형 커널**(`cv.circle`, 반경 = 로봇반경/2×해상도 + 3px)을 `cv.filter2D` 컨볼루션 → 각 조난자 주변에 원 영역 생성. 이를 `fixture_distance_margin`(벽 앞 도달 가능 마진)과 **bitwise_and** → "조난자 앞에서 실제 설 수 있는 위치"만 남김.
- `__calculate_position`(line 61-83): 위 영향권 + `checkpoints` 레이어를 후보로 합치고, `robot_center_traversed`(이미 지난 중심 경로)를 제거한 뒤, **루프 한도 1000의 제한 BFS**(`NavigatingLimitedBFSAlgorithm`)로 로봇에서 **가장 가까운** 후보 선택.
- 도달성 확인(`__is_grid_index_still_reachable`, line 86-97): 목표가 `traversable`(장애물)이면 False, 아니면 `traversed` 레이어 위에서 BFS로 연결 여부 판정.

**커널/반경** (line 31-35): `circle_radius = round(robot_diameter/2 * resolution) + 3`.

**라이브러리**: `numpy`, `cv2`(circle/filter2D), `NavigatingLimitedBFSAlgorithm`, `NavigatingBFSAlgorithm`.

**규정 연관성**: 핵심. **토큰 식별(TI)·종류(TT) 점수**와 **체크포인트(CN +10)** 획득을 위해 "로봇 중심이 토큰에서 반 타일 이내" 조건을 만족하는 위치로 접근(영향권·마진이 그 거리 보장). 구역 배율(AM)이 높은 곳일수록 가치 큼.

#### `go_to_fixtures_position_finder.py` 보충 — BFS 한도
조난자가 멀거나 맵이 넓을 때 1000루프 제한으로 프레임 지연을 막는다(`NavigatingLimitedBFSAlgorithm`, `bfs.py:112-177`).

### 4.2 `follow_walls/` — 벽 따라가기 (우선순위 2)

#### `follow_walls_subagent.py`
**목적**: 벽 근처 미방문 위치로 이동해 조난자 발견 확률을 높임. `GoToFixturesAgent` 실패 시 폴백.
**클래스**: `FollowWallsAgent(SubagentInterface)`. 구조는 GoToFixtures와 동일(PositionFinder+PathFinder).

#### `follow_walls_position_finder.py`
**목적**: `fixture_distance_margin`(벽 앞 도달 가능 마진) 레이어에서 미방문 후보를 골라 가장 가까운 곳을 BFS로 선택.

**핵심 알고리즘** (`__calculate_position`, line 63-105):
1. **고립 후보 제거**: `smoother_template`(반경 ~3cm = `0.03*resolution`의 솔리드 디스크, **중심 제외**, line 36-40)를 `cv.filter2D`로 컨볼루션 → 주변 후보 수가 `min_number_to_be_valid=10` 미만이면 고립점으로 제거. 벽 한 점만 튀어나온 곳을 버려 의미 있는 벽 구간만 남김.
2. **디더링**(`__dither_array`, line 126-134): 2칸 간격 격자 마스크로 후보 희소화(연산량 감소).
3. `robot_center_traversed`(지난 중심) 제거, `traversable`(장애물) 위 후보 제거, `swamps`(늪) 위 후보 제거.
4. `NavigatingBFSAlgorithm`으로 로봇에서 가장 가까운 후보 선택.

도달성/통과 판정은 GoToFixtures와 같은 패턴(`traversed` 연결 BFS, `robot_center_traversed`).

**라이브러리**: `numpy`, `cv2`(circle/filter2D), `math`, `NavigatingBFSAlgorithm`.

**규정 연관성**: **벽 토큰(글자 조난자·인지 표적)** 탐지 극대화 전략 — 벽 마커는 벽에 붙어 있으므로 벽을 따라가야 카메라에 잡힌다. 늪 회피(line 88-89)는 시간 5~10배 소모 패널티 회피와 직결.

### 4.3 `go_to_non_discovered/` — 미탐색 지역 이동 (우선순위 3, 최후 수단)

#### `go_to_non_discovered_subagent.py`
**목적**: 위 두 전략이 모두 목표를 못 찾을 때, **아직 본 적 없는(`discovered=False`)** 가장 가까운 곳으로 향함.
**클래스**: `GoToNonDiscoveredAgent`. 특이점: `__do_force_position_finder`(line 42-44) — **경로 완료 또는 경로 탐색 실패** 시 PositionFinder 재탐색을 강제(다른 두 서브에이전트와 달리 PathFinder 상태를 트리거로 사용).

#### `go_to_non_discovered_position_finder.py`
**목적**: `discovered` 레이어가 False인(로봇 전방 ~170도 시야에 한 번도 안 들어온) 가장 가까운 접근 가능 위치 탐색.

**핵심 알고리즘** (`__get_closest_unseen_grid_index`, line 67-88):
- 시작점 보정: 로봇이 `traversable`(장애물) 위면 `BFSAlgorithm`(closed set 없는 단순 BFS, `bfs.py:5-41`)으로 가장 가까운 빈칸으로 출발점 이동.
- `NavigatingBFSAlgorithm(found=discovered==False, traversable=traversable==False, max_result_number=1)`로 **최단 미탐색 위치 1개**를 찾음.
- 재탐색 트리거: 현재 목표가 `traversable`(장애물)로 바뀌었거나 강제 시(`__is_objective_untraversable`, line 57-65).

**라이브러리**: `numpy`, `cv2`(디버그 시각화), `BFSAlgorithm`, `NavigatingBFSAlgorithm`, `flags`.

**규정 연관성**: **매핑 보너스(MB)** 직결 — 맵 정확도를 위해 미탐색 영역을 빠짐없이 훑는 프론티어 기반 탐색(frontier exploration). 새 구역(통로 b/y/g/p/o/r 건너편)까지 탐색을 밀어붙임.

### 4.4 `return_to_start/` — 시작점 복귀

#### `return_to_start_subagent.py`
**목적**: 탐색 종료 후 `start_position`으로 A* 복귀.
**클래스**: `ReturnToStartAgent`. PositionFinder 없이 **PathFinder만** 사용 — 목표가 고정(`start_position`)이므로 탐색 불필요. `target_position_exists()`는 `start_position is not None`로 판정(line 32-34).

**라이브러리**: `numpy`, `PathFinder`.

**규정 연관성**: **탈출 보너스(EB +10%)** 의 "시작 타일 복귀" 조건 구현. `Agent.do_end()`의 4cm 판정과 짝을 이룬다.

---

## 5. 규정(2026) 연관성 요약

| 규정 항목 | 구현 위치 |
|---|---|
| 토큰 식별 TI / 종류 TT (반 타일 이내 접근) | `go_to_fixtures_*` (zone of influence + fixture_distance_margin) |
| 체크포인트 CN (+10) | `go_to_fixtures_position_finder` (`checkpoints` 레이어 후보 추가) |
| 벽 토큰(글자/인지표적) 탐지 기회 확보 | `follow_walls_*` (벽 마진 추종) |
| 늪(swamp) 회피(시간 5~10배) | `follow_walls_position_finder` (swamps 후보 제거), A* `navigation_preference` |
| 매핑 보너스 MB (탐색 완전성) | `go_to_non_discovered_*` (frontier 탐색) |
| 탈출 보너스 EB (시작 타일 복귀) | `return_to_start_*` + `Agent.do_end()` (4cm 임계) |
| 시간 제한 480초 | `Agent.max_time`, `PathTimeCalculator` |
| 자율 제어(사전매핑/데드레커닝 금지) | 전 모듈이 실시간 `pixel_grid` 레이어 기반 동적 결정 |
| 벽/구멍/장애물 회피 주행 | `pathfinder.py` A*(`traversable`) + 장애물 재계획 |

---

## 6. 다른 모듈과의 상호작용 (호출 관계)

- **호출하는 쪽(상위)**: `Executor`(메인 루프) → `Agent.update()` / `get_target_position()` / `do_end()`.
- **`Agent`가 호출**: `SubagentPriorityCombiner` → 3개 서브에이전트, `ReturnToStartAgent`, `StateMachine`, `StepCounter`, `PathTimeCalculator`.
- **각 서브에이전트가 호출**: 자신의 `PositionFinder` + `PathFinder`.
- **PathFinder/PathTimeCalculator가 호출**: `algorithms/np_bool_array`의 `aStarAlgorithm`, `NavigatingBFSAlgorithm`; `PathSmoother`.
- **읽는 데이터(전 모듈)**: `Mapper.pixel_grid`의 레이어들(`traversable`, `navigation_preference`, `discovered`, `traversed`, `robot_center_traversed`, `victims`, `checkpoints`, `fixture_distance_margin`, `swamps`), `Mapper.robot_position`/`robot_grid_index`/`start_position`, `Mapper.visualizer`.
- **출력 소비자**: `Agent.get_target_position()`의 좌표를 `Executor`가 받아 `drive_base`/`Robot`에 이동 명령으로 전달.

---

### 부록 — 핵심 설계 패턴 한눈에

- **2층 폴백 전략**: `Agent`(FSM) → `SubagentPriorityCombiner`(우선순위 체인) → 서브에이전트(전략) → PositionFinder/PathFinder(알고리즘).
- **"무엇을(목표)"과 "어떻게(경로)"의 분리**: PositionFinder = BFS로 *어디로*, PathFinder = A*로 *어떻게*.
- **레이어 기반 의사결정**: 모든 판단이 pixel_grid 불리언/값 레이어의 마스킹·컨볼루션·BFS/A*로 환원된다.
