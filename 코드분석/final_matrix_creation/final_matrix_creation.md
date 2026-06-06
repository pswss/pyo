# `final_matrix_creation` 모듈 상세 분석

> RoboCupJunior Rescue Simulation 2026 / Webots·Erebus 자율 로봇 코드
> 분석 대상: `src/final_matrix_creation/final_matrix_creator.py`

---

## 1. 폴더/모듈 개요

`final_matrix_creation`은 로봇이 탐색 중 내부적으로 누적해 온 **픽셀 단위 지도(픽셀 그리드)** 를, 대회 게임 매니저(Erebus 서버)가 채점에 사용하는 **규정 맵 행렬(텍스트 매트릭스)** 형식으로 변환하는 단 하나의 출력 단계 모듈이다. 즉 시스템 전체에서 "센서로 만든 내부 표현 → 제출용 공식 표현"으로 번역하는 마지막 어댑터 역할을 한다.

이 변환의 결과는 **매핑 보너스(MB)** 점수에 직결된다. 규정상 MB는 `정확도 × 1.2 + 1`(최대 2배)로 전체 점수에 곱해지므로, 이 모듈이 만드는 행렬이 실제 필드와 얼마나 일치하느냐가 최종 점수를 좌우한다. 모듈은 `src/mapper`가 채운 픽셀 그리드(`CompoundExpandablePixelGrid`)를 입력받아, 벽·바닥색·로봇 시작점을 합성한 2D 문자 배열을 만들고, 이를 `robot/devices/comunicator.send_map()`이 직렬화하여 서버로 보낸다.

폴더 구성은 단일 파일이다(`__pycache__` 제외):
- `final_matrix_creator.py` — 변환에 필요한 3개 클래스 전부 포함.

---

## 2. 파일별 상세 분석 — `final_matrix_creator.py`

이 파일은 3개의 클래스로 구성된다. 처리 순서대로 (1) 벽, (2) 바닥색, (3) 합성·진입점이다.

```
픽셀 그리드(walls / floor_color)
      │
      ├── WallMatrixCreator   : walls 배열   → 벽 노드 이진 배열(0/1)
      ├── FloorMatrixCreator  : floor_color  → 타일 코드 문자열 그리드
      │
      └── FinalMatrixCreator  : 위 둘 + 시작위치(5) 합성 → 최종 텍스트 행렬
```

### 의존 라이브러리/기능
- **numpy** (`import numpy as np`): 전 과정의 배열 연산, 슬라이싱, `np.rot90`(템플릿 회전), `np.where`, `np.sum`, 마스크 인덱싱.
- **OpenCV (`cv2`)**: `cv.cvtColor`(BGR→HSV), `cv.inRange`(색 범위 마스킹), `cv.rectangle`/`cv.imshow`/`cv.resize`(디버그 시각화), `cv.imwrite`(디버그 이미지 저장).
- **skimage**: import만 되어 있고 실제 사용처는 없음(잔재 import, `final_matrix_creator.py:5`).
- **copy, math, time**: `copy.deepcopy`(디버그용 배열 복제), `time.time()`(디버그 파일명), `math`는 import만 되고 미사용.
- **데이터 구조**: `data_structures.compound_pixel_grid.CompoundExpandablePixelGrid`, `data_structures.vectors.Position2D`.
- **flags**: `SHOW_MAP_AT_END`, `DO_SAVE_FINAL_MAP`, `SAVE_FINAL_MAP_DIR`, `DO_SAVE_DEBUG_GRID`, `SAVE_DEBUG_GRID_DIR` — 모두 디버그 토글. `src/flags.py`에서 전부 0(비활성)이 기본값.

---

### 2.1 `WallMatrixCreator` — 벽 픽셀 → 벽 노드 이진 배열

**목적**: 픽셀 단위 벽 배열(`walls`, bool)을 "타일 경계 노드" 단위의 0/1 배열로 변환한다. 규정 맵 행렬에서 **벽 = 1**에 해당한다.

**핵심 알고리즘: 템플릿 매칭(가중 합산) 기반 벽 방향 판정**

생성자(`final_matrix_creator.py:25-70`)에서 두 종류의 10×10 가중치 템플릿을 만든다. 여기서 10은 `square_size_px`(= 쿼터타일 한 변의 픽셀 수)에 맞춘 크기다.

- **직선 벽 템플릿** `straight_template` (`:30-43`): 상단 두 행에 가중치 `1,2,2,2,2,1`을 배치. 타일 한 변(에지)에 벽이 붙어 있는지를 검출한다.
- **코너 벽 템플릿** `vortex_template` (`:46-59`): 좌상단 3×3 구역에 가중치 3을 배치. 타일 모서리(버텍스)에 벽이 모이는지를 검출한다.

이 두 템플릿을 `np.rot90`으로 90°씩 회전시켜 **8방향 템플릿 딕셔너리**를 만든다(`:62-70`). 키는 (dy, dx) 오프셋이다:
- 직선 4방향: `(-1,0)`=위, `(0,-1)`=좌, `(1,0)`=아래, `(0,1)`=우.
- 코너 4방향: `(-1,-1)`=좌상, `(1,-1)`=좌하, `(1,1)`=우하, `(-1,1)`=우상.

**`__get_tile_status(min_x, min_y, max_x, max_y, wall_array)`** (`:73-94`)
- 타일 영역 `square = wall_array[min_x:max_x, min_y:max_y]`를 잘라낸다.
- 크기가 `(square_size_px, square_size_px)`가 아니면 빈 목록 반환(경계 타일 방어).
- `non_zero_indices = np.where(square != 0)`로 벽 픽셀 위치를 얻고, 각 방향 템플릿에 대해 `np.sum(template[non_zero_indices])`로 **벽 픽셀이 떨어진 위치의 가중치 합**을 계산한다. 즉 일반적 이미지 컨볼루션이 아니라 "마스크된 위치의 템플릿 가중치 합산"이다.
- 합이 `threshold=10`(`:26`) 이상인 방향만 결과 목록에 담는다.

**`transform_wall_array_to_bool_node_array(wall_array, offsets)`** (`:96-126`)
- `offsets[0]/offsets[1]`에서 시작해 `square_size_px` 보폭으로 벽 배열을 **타일 단위 순회**(`:105-118`)하며 각 타일의 방향 목록(`__get_tile_status`)을 2D `grid`에 쌓는다.
- 이후 `__orientation_grid_to_final_wall_grid`로 노드 배열로 변환.
- `SHOW_MAP_AT_END`가 켜져 있으면 `cv.rectangle`로 타일 경계를 그려 시각화.

**`__orientation_grid_to_final_wall_grid(orientation_grid)`** (`:128-152`)
- 출력 노드 배열은 타일 수의 **2배** 크기(`shape *= 2`): 즉 타일 하나가 **2×2 노드**로 펼쳐진다.
- 각 타일 `(y, x)`의 기준 노드는 `(y*2, x*2)`. 방향 오프셋 `(dy, dx)`를 더한 위치를 `True`로 설정한다. 예: 위쪽 벽 `(-1,0)` → 노드 `(y*2-1, x*2)`가 벽. 코너 `(-1,-1)` → `(y*2-1, x*2-1)`가 벽.
- 결과: 벽이 있는 곳은 `True`, 나머지는 `False`인 노드 이진 배열.

> **규정 대응**: 규정 맵 형식의 "쿼터타일 + 주변 엣지/버텍스를 셀로 표현, 벽=1"을 정확히 구현. 직선 템플릿 = 엣지 벽, 코너 템플릿 = 버텍스 벽.

---

### 2.2 `FloorMatrixCreator` — 바닥 색 픽셀 → 타일 코드 문자열

**목적**: 카메라가 기록한 바닥 색(BGR) 배열(`floor_color`)을 HSV로 변환해, 각 타일이 일반/구멍/늪/체크포인트/연결통로 중 무엇인지 **코드 문자열**로 판정한다.

**중요 차이점**: 생성자에서 `self.__square_size_px = square_size_px * 2`(`:165`). 벽은 쿼터타일(절반 타일) 크기로 보지만, **바닥은 풀 타일(=절반의 2배) 크기 단위**로 본다. 즉 색 판정은 타일 1개 단위로 한다.

**색 범위 테이블** `__floor_color_ranges` (`:167-206`): 각 코드별로 `(HSV 하한, HSV 상한)`과 `threshold`(타일 내 해당 색 픽셀 비율 하한)를 둔다.
- `"0"` 일반 바닥 / `"0"` 빈 공간(void) — **둘 다 키가 `"0"`이라 파이썬 딕셔너리에서 뒤의 void 정의가 앞의 일반 바닥 정의를 덮어쓴다(버그성 동작).** 결과적으로 일반 바닥 색 범위 `((0,0,37),(0,0,192))`는 사용되지 않고, void 범위 `((100,0,0),(101,1,1))` threshold 0.9만 `"0"`으로 등록된다.
- `"4"` 체크포인트(은색): `((95,0,65),(128,122,198))`.
- `"2"` 구멍(검정): `((0,0,10),(0,0,30))`.
- `"3"` 늪(갈색): `((19,112,32),(19,141,166))`.
- `"6"` 레벨 1-2 연결(파랑), `"7"` 레벨 2-3 연결, `"8"` 레벨 3-4 연결.

**`__get_square_color(...)`** (`:210-231`) — 알고리즘: **HSV 색공간 임계 + 다수결**
1. 타일 영역을 잘라 `cv.cvtColor(square, COLOR_BGR2HSV)`로 HSV 변환.
2. 전부 0이면(미관측) `"0"` 반환.
3. 각 코드 범위에 대해 `cv.inRange`로 마스크를 만들고 `np.count_nonzero`로 매칭 픽셀 수 계산.
4. 매칭 픽셀 수가 `threshold × 타일넓이`를 초과하는 코드만 후보로 모으고, 그중 **가장 많이 매칭된 코드**(`max(color_counts, key=...)`)를 선택. 후보가 없으면 `"0"`.

**`get_floor_colors(floor_array, offsets)`** (`:234-263`): `__square_size_px`(풀 타일) 보폭으로 순회하며 타일별 코드 문자열을 2D 그리드로 만든다.

> **규정 대응**: 규정의 바닥 종류 코드 — 구멍=2, 늪=3, 체크포인트=4 — 를 색으로 판정해 그대로 매핑. 통로 색(b/y/g/p/o/r)은 여기서 숫자 코드 6/7/8 등으로 대체 표현(아래 "규정 연관성"의 불일치 항목 참조).

---

### 2.3 `FinalMatrixCreator` — 진입점 및 합성

**목적**: 위 두 변환 결과와 로봇 시작 위치를 하나의 텍스트 행렬로 합성. 이 모듈의 공개 API.

**생성자** (`:285-290`): `self.__square_size_px = round(tile_size / 2 * resolution)`. 타일의 **절반(쿼터타일 변)** 을 픽셀 수로 환산. 이를 그대로 `WallMatrixCreator`에, 내부에서 ×2 한 값을 `FloorMatrixCreator`에 전달.

**`pixel_grid_to_final_grid(pixel_grid, robot_start_position)`** (`:293-341`) — 메인 데이터 흐름:
1. `wall_array = pixel_grid.arrays["walls"]`, `color_array = pixel_grid.arrays["floor_color"]` 추출(`:306-307`).
2. (디버그 플래그시) 벽/컬러 그리드 PNG 저장(`:310-314`).
3. 벽 오프셋 계산: `__get_offsets(square_size_px, pixel_grid.offsets)`(`:317`).
4. 벽 노드 배열 생성(`:320`).
5. 바닥 오프셋은 **풀 타일 기준 + 절반 타일만큼 이동**: `__get_offsets(square_size_px*2, pixel_grid.offsets + square_size_px)`(`:323`). 벽 노드 격자와 바닥 타일 중심을 정렬하기 위한 보정.
6. 바닥 코드 그리드 생성(`:326`).
7. `robot_start_position`이 `None`이면 빈 배열 반환(`:329-330`).
8. **시작 위치를 노드 인덱스로 변환**(`:333-336`):
   - `coordinates_to_array_index(robot_start_position)`로 픽셀 인덱스 변환 후 `-= offsets`로 타일 격자 원점 맞춤.
   - `np.round((start_array_index / square_size_px) * 2).astype(int) - 1`: 쿼터타일 단위로 나눈 뒤 ×2(타일당 2노드) 후 -1 보정 → 노드 좌표.
9. `__get_final_text_grid(...)`로 합성 후 `np.array`로 반환(`:339-341`).

**`__get_final_text_grid(wall_node_array, floor_type_array, robot_node)`** (`:343-379`):
- 벽 노드 배열을 순회해 `True→"1"`, `False→"0"` 문자 그리드로 변환(`:360-367`). 여기서 **노드 좌표계가 4× 해상도로 확장됨에 유의** — 아래 4.노드 설명 참조.
- 바닥 코드를 각 타일 중심에 기록(`:370-374`): 타일 `(y,x)` 중심 노드는 `(y*4+3, x*4+3)`. 여기서 보폭이 **4**라는 점이 핵심 — 한 타일이 최종 행렬에서 **4×4 노드**를 차지한다(엣지/버텍스 셀까지 포함한 규정 표현). `__set_node_as_character`로 중심 주변 2×2에 코드 기록.
- 로봇 시작 위치를 `"5"`로 기록(`:377`).

**`__get_offsets(square_size, raw_offsets)`** (`:382-384`): `np.round(raw_offsets % square_size)`. 픽셀 그리드 원점 오프셋을 타일 크기로 모듈러 → 격자 정렬용 잔여 오프셋.

**`__set_node_as_character(final_text_grid, node, character)`** (`:387-400`): 중심 노드의 **대각선 4이웃** `(±1,±1)`에 같은 문자를 기록(중심 자체는 안 건드림). 타일 중심부 2×2 셀에 코드를 채우는 효과. 경계를 벗어나면 `IndexError`를 무시.

> **규정 대응**: 최종 행렬에서 한 타일이 4×4 노드(쿼터타일 4개 + 그 엣지/버텍스)를 차지하도록 펼치고, 벽=1, 통로=0, 시작=5, 바닥특수=2/3/4를 타일 중심에 배치. 규정의 "쿼터타일+엣지/버텍스 셀" 표현과 정렬·90° 회전 비교(채점은 서버가 수행)를 전제로 한 출력이다.

---

## 3. 규정 연관성

| 규정 항목 | 코드 구현 | 비고 |
|---|---|---|
| 맵 행렬: 벽=1 | `WallMatrixCreator` 템플릿 매칭 → `"1"` | 직선=엣지, 코너=버텍스 |
| 통로=0 | 기본값 `"0"` | |
| 구멍=2 | `FloorMatrixCreator` `"2"` (검정 HSV) | |
| 늪=3 | `"3"` (갈색 HSV) | |
| 체크포인트=4 | `"4"` (은색 HSV) | 규정상 +10·AM 점수 대상 |
| 시작 타일=5 | `__get_final_text_grid` `"5"` | 정렬 기준점, EB(탈출 보너스) 관련 |
| 매핑 보너스(MB) | 모듈 출력 전체가 MB 입력 | 정확도×1.2+1, 최대 2배 |

**규정과의 불일치/누락(중요 발견)**:
- **토큰(글자/해즈맷) 미인코딩**: 규정은 토큰을 글자코드(H,S,U,F,P,C,O)로 행렬에 넣어야 하나, 이 모듈은 `walls`/`floor_color`만 읽고 `victims`/`hazmats`/`fixture_detection` 배열을 전혀 참조하지 않는다. 즉 **맵 행렬에 토큰 글자가 들어가지 않는다.** (토큰 점수 자체는 `send_victim`으로 별도 획득하므로 점수 손실은 아니지만, MB 정확도에 토큰 셀은 기여하지 못함.)
- **장애물 `x` 미구현**: 타일 중앙 장애물 코드 없음.
- **통로 색 코드 불일치**: 규정은 b/y/g/p/o/r 문자를 요구하지만 코드는 `6`/`7`/`8` 숫자만 처리(`g`,`o`,`r` 등 일부 색은 누락). 서버 평가 형식과 어긋날 수 있다.
- **Area 4 전체 `*` 미처리**: 비격자 구역 표시 없음.
- **딕셔너리 키 중복 버그**: `__floor_color_ranges`에 `"0"` 키가 두 번(`:168`, `:174`) — 일반 바닥용 범위가 void 정의에 덮여 사라진다. 일반 바닥은 결국 "후보 없음 → `"0"`" 경로로만 0이 된다.

---

## 4. 다른 모듈과의 상호작용

**누가 이 모듈을 호출하는가 (호출됨)**:
- `executor/executor.py:62` — `FinalMatrixCreator(mapper.tile_size, mapper.pixel_grid.resolution)` 생성.
- `executor/executor.py:304` (`state_end`) — 미션 종료 시 행렬 생성 → `comunicator.send_map` → `send_end_of_play`.
- `executor/executor.py:310` (`state_send_map`) — 시간 임박 시 강제 전송 후 다시 explore로 복귀.
- `rescue_robot.py:51, 295` — 초보자용 facade도 동일 API 사용.

**이 모듈이 호출하는 것 (호출함)**:
- `data_structures.compound_pixel_grid.CompoundExpandablePixelGrid`:
  - `.arrays["walls"]`, `.arrays["floor_color"]` 읽기.
  - `.offsets` 읽기(격자 정렬), `.coordinates_to_array_index()`(시작 위치 변환), `.get_colored_grid()`(디버그 저장).
- `data_structures.vectors.Position2D`: import만(타입 참고).
- OpenCV/numpy: 내부 계산.

**출력 소비처**: 반환된 `np.array`(문자 2D 행렬)는 `robot/devices/comunicator.py:55 send_map`이 받아서
`struct.pack('2i', *shape)` + `','.join(np_array.flatten())`(UTF-8)로 직렬화 → emitter로 Erebus 서버에 전송, 직후 `'M'`(맵 평가 요청) 메시지 전송(`comunicator.py:64-71`).

**데이터 흐름 요약**:
```
mapper(센서→픽셀그리드)
   → FinalMatrixCreator.pixel_grid_to_final_grid(grid, start_pos)
        ├ WallMatrixCreator  : walls → 벽 노드(0/1)
        ├ FloorMatrixCreator : floor_color(HSV) → 타일 코드(0/2/3/4/6/7/8)
        └ 합성: 4×4 노드/타일, 중심에 바닥코드, 시작=5
   → comunicator.send_map(matrix)  → Erebus 서버(MB 채점)
```

---

## 5. 노드 좌표계 정리(혼동 방지)

세 단계에서 "타일당 노드 수"가 달라 보이지만 일관된다:
1. `WallMatrixCreator`는 **쿼터타일** 단위로 보고, 타일당 **2×2 노드** 배열을 만든다(`shape *= 2`).
2. 단, 입력 `square_size_px`는 "절반 타일"이므로, 풀 타일 기준으로 보면 노드 배열 해상도는 **풀 타일당 4×4 노드**가 된다.
3. 따라서 `__get_final_text_grid`에서 바닥 코드 보폭이 **4**, 중심 오프셋 `+3`이 맞아떨어진다(`y*4+3`).

이 4×4 노드 표현이 규정의 "쿼터타일 4개 + 사이 엣지/버텍스 셀" 표현과 1:1 대응한다.
