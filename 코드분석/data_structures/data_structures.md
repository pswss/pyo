# `data_structures` 모듈 분석

RoboCupJunior Rescue Simulation 2026 — Webots/Erebus 자율 로봇 코드

---

## 1. 폴더/모듈 개요

`data_structures`는 로봇 코드 전체가 의존하는 **기반(foundation) 자료구조 계층**이다. 상위 로직(매핑, 경로탐색, 서브에이전트, 토큰 탐지, 디바이스 추상화)이 다루는 모든 수치 데이터는 결국 이 폴더의 네 가지 타입으로 표현된다.

- **`angle.py`** — 각도를 라디안/도(degree) 양쪽으로 다루며 0~2π 정규화·최단거리(래핑) 수학을 제공하는 불변 수치형 `Angle`. 로봇 방향, 라이다 빔 각도, 카메라 토큰 방위 등 "방향이 들어가는 모든 계산"의 단위.
- **`vectors.py`** — 직교 좌표 `Position2D`(x, y)와 극좌표 `Vector2D`(방향 Angle + 크기). 두 좌표계를 상호 변환(`to_vector`/`to_position`)하여 "라이다가 준 (각도, 거리)"를 "지도상의 (x, y)"로 바꾸는 다리 역할.
- **`compound_pixel_grid.py`** — 시스템의 **지도 핵심 저장소**. 벽·바닥·구멍·늪·체크포인트·조난자 등 24개 정보 레이어를 같은 좌표계 위에 쌓아 두고, **미터 ↔ 그리드 인덱스 ↔ 배열 인덱스** 3단계 좌표 변환과 배열 자동 확장을 담당한다.
- **`tile_color_grid.py`** — 타일 단위(저해상도) 색상 그리드. 픽셀 그리드와 같은 좌표 변환 API를 갖지만 단일 불리언 배열만 보관하며, 현재는 주로 참조/레거시 용도.

핵심 설계는 **단위 안전성**(각도는 항상 `Angle`, 좌표는 항상 `Position2D`)과 **단일 좌표계**(모든 레이어가 같은 픽셀 격자를 공유)다. 이 덕분에 매핑·경로탐색이 좌표계 불일치 없이 같은 배열을 인덱싱할 수 있다.

---

## 2. 파일별 상세 분석

### 2.1 `angle.py` — `Angle` 클래스

**목적**
방향 값을 라디안과 도 양쪽으로 다루고, 원형(circular) 각도 특유의 "래핑"을 안전하게 처리하는 수치형. 내부 저장은 항상 라디안(`__radians`), 도 단위는 프로퍼티 변환으로 노출한다.

**핵심 클래스/함수와 시그니처**
- `Angle(value, unit=RADIANS)` — `unit`은 `Angle.RADIANS(0)` 또는 `Angle.DEGREES(1)` (`angle.py:12-19`).
- `radians` / `degrees` 프로퍼티 (getter/setter) — 도 setter는 `value * π / 180`으로 라디안 변환 후 저장 (`angle.py:35-38`).
- `normalize()` — `__radians %= 2π` 후 음수면 `+2π`. 결과를 `[0, 2π)`로 강제 (`angle.py:40-44`).
- `get_absolute_distance_to(angle) -> Angle` — 두 각도 사이 **부호 없는 최단 거리**(최대 π). 시계/반시계 거리 중 작은 값 선택 (`angle.py:46-56`).
- `get_distance_to(angle) -> Angle` — **부호 있는 거리**(반시계=양수, 시계=음수). 회전 방향 결정에 사용 (`angle.py:58-65`).
- 산술/비교/형변환 매직 메서드 전부 오버로딩(`__add__`, `__sub__`, `__mul__`, `__lt__`, `__eq__`, `__round__` 등 `angle.py:74-189`) — 일반 숫자처럼 `+`, `<`, `abs()`를 쓸 수 있다.

**사용 알고리즘**
- **각도 정규화(modular wrapping)**: `mod 2π` + 음수 보정으로 모든 각도를 단일 표준 구간에 사상.
- **원형 최단거리**: 두 각을 정렬해 `clockwise = max - min`, `counterclockwise = (2π + min) - max`를 구하고 둘 중 최솟값 선택 (`angle.py:53-56`). 0°와 350°가 "10° 차이"임을 올바르게 계산하는 핵심.
- 부호 판정은 `self - angle`의 도 단위 부호 구간(`180 > diff > 0` 또는 `diff < -180`)으로 회전 방향을 결정 (`angle.py:62`).

**사용 기능/라이브러리**
- `math`(π, atan2 계열 사용처에 단위 제공), `copy.copy`(`get_absolute_distance_to`에서 인자를 복제해 원본 정규화 부작용 방지, `angle.py:48`). 외부 의존 없음.

**입력/출력 데이터 흐름**
입력: 라디안/도 스칼라 또는 다른 `Angle`. 출력: `Angle` 인스턴스(연산 결과도 항상 `Angle`로 래핑되어 단위가 보존됨). 자이로/IMU의 방향 추정, 라이다 빔 각도, `Position2D.get_angle_to`의 반환값이 모두 `Angle`.

---

### 2.2 `vectors.py` — `Position2D`, `Vector2D`

**목적**
2D 공간을 직교 좌표(`Position2D`)와 극좌표(`Vector2D`) 두 형태로 표현하고 상호 변환. 센서(극좌표: 방향+거리)와 지도(직교좌표: x, y) 사이의 변환 다리.

**핵심 클래스/함수와 시그니처**

`Position2D` (`vectors.py:6-171`)
- `Position2D()` / `Position2D(iterable)` / `Position2D(x, y)` — 가변 인자 생성 (`vectors.py:14-31`). numpy 배열·튜플·리스트·두 스칼라 모두 허용.
- `__iter__`, `__array__`(numpy 변환 시 `[x, y]`), `__getitem__/__setitem__`(인덱스 0=x, 1=y) — numpy/리스트처럼 동작 (`vectors.py:33-143`).
- `__abs__` → `sqrt(x² + y²)` (원점 거리, `vectors.py:125-127`).
- `apply_to_all(function)` / `astype(dtype)` — x, y 각각에 함수 적용한 새 `Position2D` (`vectors.py:145-151`). 좌표를 `int`로 캐스팅할 때 사용.
- `get_distance_to(other)` → `abs(self - other)` 유클리드 거리 (`vectors.py:153-155`).
- `get_angle_to(other) -> Angle` — `atan2(Δx, Δy) + 180°` 후 정규화 (`vectors.py:157-165`).
- `to_vector() -> Vector2D` — 원점 기준 (방향, 크기) 극좌표로 변환 (`vectors.py:167-171`).

`Vector2D` (`vectors.py:174-220`)
- `Vector2D(direction: Angle, magnitude)` — 극좌표 표현. 라이다 감지 결과(빔 방향 + 거리) 표현이 주 용도 (`vectors.py:179-181`).
- `__add__/__sub__/__neg__` — 방향·크기를 성분별로 연산 (`vectors.py:192-214`).
- `to_position() -> Position2D` — `x = magnitude·sin(dir)`, `y = magnitude·cos(dir)` (`vectors.py:216-220`).

**사용 알고리즘**
- **극좌표 ↔ 직교좌표 변환**: `to_position`에서 x에 `sin`, y에 `cos`를 사용(`vectors.py:218-219`). 일반 수학(x=cos, y=sin)과 축이 교환되어 있는데, 이는 Webots 좌표계 정렬과 `get_angle_to`의 `atan2(Δx, Δy)+180°` 규약(`vectors.py:163`)에 맞춘 것으로, 두 함수가 짝을 이뤄 왕복 변환의 일관성을 유지한다.
- **방위각 계산**: `atan2`로 두 점 사이 방향을 구한 뒤 180° 더하고 정규화해 `[0, 2π)`로 표준화.

**사용 기능/라이브러리**
- `math`(`sqrt`, `cos`, `sin`, `atan2`), `numpy`(`__array__` 변환), `data_structures.angle.Angle`(방향 단위).

**입력/출력 데이터 흐름**
입력: GPS 좌표, 라이다 (각도, 거리), 카메라 토큰 상대 위치. 출력: 지도 인덱싱용 (x, y) 또는 회전 제어용 (방향, 크기). `Position2D`는 `compound_pixel_grid`의 좌표 변환 메서드에 직접 들어가 그리드 인덱스로 바뀐다.

---

### 2.3 `compound_pixel_grid.py` — `CompoundExpandablePixelGrid`

**목적**
지도 정보를 **레이어(채널)별 numpy 배열**로 같은 좌표계 위에 중첩 저장하는, 동적 확장 가능한 픽셀 그리드. 시스템 전체의 단일 지도 저장소이며, 매핑이 쓰고 경로탐색/서브에이전트/최종행렬생성이 읽는다.

**초기화와 좌표계 파라미터** (`compound_pixel_grid.py:37-70`)
- `__init__(initial_shape, pixel_per_m, robot_radius_m)`:
  - `array_shape` — numpy 배열의 실제 (행, 열) 크기.
  - `offsets = array_shape // 2` — 배열 인덱스와 그리드 인덱스 간 평행이동량. **로봇 시작점이 그리드 원점(0,0)** 이 되도록 배열 중앙을 가리킨다.
  - `resolution = pixel_per_m` — 픽셀/미터 비율(해상도). `mapper.py:52-55`에서 `pixels_per_tile(10) / quarter_tile_size`로 주입된다(쿼터타일 6cm 기준 → 미터당 약 167픽셀).

**레이어 구조 (`arrays` 딕셔너리, `compound_pixel_grid.py:43-70`)**
모든 레이어는 같은 `array_shape`를 공유하므로 동일 인덱스 `[r, c]`가 모든 레이어에서 같은 물리적 위치를 가리킨다. 주요 레이어:

| 레이어 | dtype | 의미 / 규정 연관 |
|---|---|---|
| `detected_points` | uint16 | 라이다가 같은 셀을 감지한 누적 횟수(임계 초과 시 벽 확정) |
| `walls` | bool | 벽/장애물 셀 |
| `occupied` | bool | 벽 OR 구멍 = 실제 이동 불가 영역 |
| `traversable` | bool | 벽을 로봇 반경만큼 팽창한 통행 불가 영역(충돌 회피용) |
| `navigation_preference` | float32 | 경로 비용 가중치(벽 근처일수록 높음) — A*/BFS 비용맵 |
| `traversed` / `robot_center_traversed` | bool | 로봇 몸체/중심이 지나간 영역 |
| `seen_by_camera` / `seen_by_lidar` / `discovered` | bool | 센서 커버리지(미탐색 영역 탐지용) |
| `walls_seen_by_camera` / `walls_not_seen_by_camera` | bool | 벽의 카메라 확인 여부(가짜 3D victim·통로색 판정 보조) |
| `floor_color` / `average_floor_color` | uint8 ×3 | 바닥 BGR 색(체크포인트·늪·구멍·통로색 판정 원천) |
| `floor_color_detection_distance` | uint8 | 바닥 색 신뢰도(가까울수록 높음) |
| `holes` / `hole_detections` | bool | 구멍(빠지면 LoP) |
| `swamps` | bool | 늪(시간 소모 가속) |
| `checkpoints` | bool | 체크포인트(은색 타일, +10점) |
| `victims` / `victim_angles` | bool / float32 | 글자 조난자 위치와 접근 방향 |
| `hazmats` | bool | 인지 표적(해즈맷) 위치 |
| `fixture_detection` / `fixture_detection_zone` / `fixture_distance_margin` | bool | 토큰 탐지·접근 영역(벽 주변 마진) |
| `robot_detected_fixture_from` | bool | 이미 보고한 위치(중복 보고/오식별 TMI 방지) |

**좌표 변환 (3단 좌표계, `compound_pixel_grid.py:84-110`)**
세 좌표계가 있다:
1. **실제 좌표(coordinates, m)** — Webots 월드의 미터 단위. GPS·라이다가 주는 값.
2. **그리드 인덱스(grid_index)** — 로봇 시작점을 (0,0)으로 하는 부호 있는 상대 인덱스. `grid_index_min = -offsets`, `grid_index_max = array_shape - offsets`.
3. **배열 인덱스(array_index)** — numpy가 실제로 쓰는 0 이상 인덱스.

변환 체인:
- `coordinates_to_grid_index`: `(coords * resolution).astype(int)` 후 `np.flip` — **(x, y) → (row, col)** 축 교환 포함 (`compound_pixel_grid.py:84-87`). 미터에 해상도를 곱해 픽셀 단위 정수로 만들고, 배열은 (행=y, 열=x) 규약이므로 뒤집는다.
- `grid_index_to_coordinates`: 위의 역연산(`/ resolution` + flip) (`:89-92`).
- `array_index_to_grid_index` / `grid_index_to_array_index`: `±offsets` (`:94-100`).
- 합성 메서드 `coordinates_to_array_index`, `array_index_to_coordinates` (`:102-110`)가 미터 ↔ 배열 인덱스를 한 번에 변환. 매핑·경로탐색이 가장 많이 호출하는 진입점.

**동적 확장 알고리즘 (`compound_pixel_grid.py:114-181`)**
탐색 중 로봇이 초기 배열 범위를 벗어나는 좌표에 접근하면 배열을 키운다.
- `expand_to_grid_index(grid_index)`: 대상 인덱스를 배열 인덱스로 바꿔 네 방향(아래/오른쪽/위/왼쪽) 초과량을 계산하고 해당 `add_*` 호출 (`:114-127`).
- `add_end_row/column`은 끝에 0 배열을 붙이고, `add_begining_row/column`은 앞에 붙이면서 **`offsets`를 같이 증가**시킨다(시작점이 여전히 같은 그리드 좌표 (0,0)에 남도록, `:135-153`).
- 실제 결합은 `np.vstack`(행 방향)·`np.hstack`(열 방향)으로 모든 레이어에 동일 적용 (`__add_*_to_grid`, `:155-181`). 각 레이어의 `dtype`을 보존해 새 0 블록을 만든다.
- 주의: 확장 후 `offsets`/`array_shape`가 바뀌므로 **호출 직후 배열 인덱스는 재계산**해야 한다(독스트링 `:117` 명시).

**디버그 시각화 (`get_colored_grid`, `compound_pixel_grid.py:183-201`)**
여러 불리언 레이어를 하나의 float32 BGR 이미지로 합성: 마진=파랑, 점유=흰색(전체 ×0.3로 어둡게), victim=초록, hazmat=빨강, fixture_detection=노랑. 개발 중 지도 상태 확인용.

**사용 기능/라이브러리**
- `numpy`(전 레이어 배열, `vstack`/`hstack` 확장, dtype별 0 배열), `cv2`(import만 되어 있고 이 파일에서 직접 사용은 없음, `:2`), `copy`, `math`, `flow_control.step_counter.StepCounter`(import만, 현재 직접 사용 없음), `Position2D`/`Vector2D`/`Angle`.

**입력/출력 데이터 흐름**
입력: 매핑 계층이 `coordinates_to_array_index`로 얻은 인덱스에 각 레이어 값을 기록. 출력: 경로탐색이 `occupied`/`traversable`/`navigation_preference`를 비용맵으로, 서브에이전트가 `discovered`/`fixture_*`를, 최종행렬생성이 `walls`/`holes`/`swamps`/`checkpoints`/`victims`/`hazmats`를 읽어 규정 맵 행렬을 만든다.

---

### 2.4 `tile_color_grid.py` — `TileColorExpandableGrid`

**목적**
타일(격자칸) 단위의 저해상도 단일 불리언 그리드. 픽셀 그리드와 동일한 좌표 변환·동적 확장 API를 갖지만 한 칸이 `tile_size`(m)에 대응한다. 독스트링상 **현재는 주로 참조용**이며 실제 색 저장은 픽셀 그리드(`floor_color`)가 담당한다(`tile_color_grid.py:6-13`).

**핵심 클래스/함수와 시그니처**
- `__init__(initial_shape, tile_size)` — `resolution = 1 / tile_size`(타일/미터). `grid_index_max/min`을 속성으로 미리 계산해 둠(픽셀 그리드는 프로퍼티인 점과 차이, `:14-22`).
- 단일 `self.array`(bool) 하나만 보관 (`:21`).
- 좌표 변환 메서드(`coordinates_to_grid_index` 등)는 `CompoundExpandablePixelGrid`와 동일한 `*resolution` + `np.flip` + `±offsets` 로직 (`:26-48`).
- 동적 확장(`expand_to_grid_index`, `add_*`, `__add_*_to_array`)도 픽셀 그리드와 동일하나 레이어 딕셔너리 대신 단일 배열에만 적용 (`:52-92`).
- `get_colored_grid()` — `pass`, 미구현 (`:94-95`).

**사용 알고리즘 / 라이브러리**
동적 확장은 `np.vstack`/`np.hstack` 기반(픽셀 그리드와 동일). `numpy`, `cv2`(import만), `copy`. `mapper.py:57-59`에서 `tile_size`로 1×1 형태로 생성된다.

**입력/출력 데이터 흐름**
`Mapper`가 보유하지만 핵심 데이터 경로는 픽셀 그리드로 이전되어 있어 비중이 낮다. 픽셀 그리드보다 거친 타일 단위 마스크가 필요할 때 사용되는 보조/레거시 구조.

---

## 3. 규정 연관성 (2026)

- **맵 행렬 정확도 / 매핑 보너스(MB ×최대 2)**: `compound_pixel_grid`의 레이어들(`walls`, `holes`, `swamps`, `checkpoints`, `victims`, `hazmats`, `floor_color`)이 규정 맵 행렬의 셀 값(벽=1, 구멍=2, 늪=3, 체크포인트=4, 토큰 코드 등)으로 변환되는 원천 데이터. 정확한 좌표 변환과 시작점 원점 정렬이 행렬 비교 정확도에 직결.
- **시작 타일/탈출 보너스(EB)**: `offsets`로 시작점을 그리드 (0,0)에 고정하는 설계가 "시작타일로 정렬"·복귀 판정의 기준점.
- **체크포인트(CN +10)·늪·구멍(LoP)**: `checkpoints`/`swamps`/`holes`/`hole_detections` 레이어로 표현, 바닥색은 `floor_color`로 판정.
- **토큰 식별/오식별(TI/TT/TMI -5)**: `victims`/`hazmats`/`victim_angles`/`fixture_*` 레이어가 글자 조난자·인지 표적 위치와 접근 방향을 보관. `robot_detected_fixture_from`이 중복 보고로 인한 오식별(-5)을 방지. "토큰에서 반 타일 이내" 판정은 이 좌표계 위에서 거리 계산(`Position2D.get_distance_to`)으로 이뤄진다.
- **노이즈 강건성(v26 GPS 노이즈)**: `detected_points` 누적 임계(여러 번 본 셀만 벽 확정)와 `floor_color_detection_distance`(가까울수록 신뢰) 레이어가 노이즈 완화 장치. `Angle`의 정규화·최단거리 수학은 자이로/방향 추정의 래핑 오류를 막는다.
- **격자 기반 필드(Area 1~3 / 비격자 Area 4)**: `resolution`(픽셀/미터)과 `tile_size`/`quarter_tile_size` 파라미터로 12cm 타일·6cm 쿼터타일을 모두 같은 픽셀 격자에 표현.

---

## 4. 다른 모듈과의 상호작용

**`Angle`을 사용하는 곳**
- `vectors.Position2D.get_angle_to` / `Vector2D.direction`이 `Angle` 반환·보관.
- `robot/pose_manager.py`, `robot/devices/gyroscope.py`, `drive_base.py`가 로봇 방향·회전 제어에서 `Angle`의 정규화·`get_distance_to`(부호 있는 회전 방향)를 사용.
- 라이다·카메라 방위 계산.

**`Position2D` / `Vector2D`를 사용하는 곳**
- `robot/devices/gps.py`(위치), `lidar.py`(빔 → `Vector2D` → `to_position`), `camera.py`(토큰 상대 위치).
- 매핑 계층 전체와 경로탐색이 좌표를 `Position2D`로 다루다가 그리드 변환 메서드에 넘김.

**`CompoundExpandablePixelGrid`를 사용하는 곳** (단일 인스턴스, `mapping/mapper.py:52`에서 생성)
- **쓰기**: `wall_mapper`, `floor_mapper`, `robot_mapper`, `occupied_mapping`, `fixture_mapper`, `fixture_detection`이 각 레이어를 갱신.
- **읽기**: `agent/pathfinding/pathfinder.py`·`path_time_calculator.py`(비용맵), `agent/subagents/*`(follow_walls·go_to_fixtures·go_to_non_discovered·return_to_start의 position_finder들), `executor/stuck_detector.py`(LoP 감지), `final_matrix_creation/final_matrix_creator.py`(규정 맵 행렬 생성).
- `mapper`가 매 스텝 `expand_to_grid_index`로 배열을 키우고 `coordinates_to_array_index`로 로봇 위치를 인덱싱.

**`TileColorExpandableGrid`를 사용하는 곳**
- `mapping/mapper.py:57`에서 생성·보유. 핵심 데이터 경로는 픽셀 그리드로 이관되어 보조 역할.

**호출 방향 요약**
`robot/devices/*`(센서) → `Angle`/`Vector2D`/`Position2D` 생성 → `mapping/*`가 `CompoundExpandablePixelGrid`에 기록 → `agent/*`·`executor/*`·`final_matrix_creation/*`가 그리드를 읽어 탐색·경로·맵 전송 수행. `data_structures`는 이 사슬의 최하층이며 어떤 상위 모듈도 호출하지 않는다(순수 피호출 계층).
