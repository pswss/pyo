# fixture_detection 모듈 분석 (RoboCup Rescue Sim 2026)

## 1. 폴더/모듈 개요

`fixture_detection`은 RGB 카메라 이미지에서 **벽 토큰(fixture)** 을 찾아내고(detection) 그 종류를 분류(classification)하는 모듈이다. 규정상 벽에는 두 종류의 토큰이 붙는다: **Letter victim**(검은 산세리프 글자 Φ/Ψ/Ω → H/S/U)과 **Cognitive target/hazmat**(동심원 색 합산 또는 UN 플래카드 → F/P/C/O), 그리고 함정인 **가짜 3D victim**. 이 모듈은 카메라 프레임을 HSV로 필터링하고, 벽 영역에 한정해 컨투어를 잡아 후보를 추출한 뒤, 색 픽셀 수·컨투어 계층·동심원 색 합산 같은 규칙 기반 알고리즘으로 토큰 종류를 판정한다.

역할은 두 갈래로 나뉜다. (a) **FixtureDetector**(`fixture_detection.py`)는 "토큰이 공간 어디에 있는가"를 담당한다 — 카메라 픽셀 위치를 로봇 위치/카메라 FOV/거리와 결합해 실제 좌표로 변환하고, LiDAR로 만든 벽 픽셀 그리드(`walls` 레이어)에 레이캐스팅하여 토큰을 벽면에 고정시킨다. 동시에 벽에서 과도하게 돌출된 감지를 **가짜 3D victim**으로 걸러낸다. (b) **FixtureClasiffier**(`fixture_clasification.py`)는 "이 토큰이 무엇인가"를 담당한다 — 잘라낸 토큰 이미지를 받아 H/S/U/F/P/C/O 중 하나의 보고 글자 또는 `None`(보고 안 함)을 반환한다. `victim_clasification.py`, `color_filter.py`, `non_fixture_filterer.py`는 분류기를 떠받치는 보조 유틸이다.

---

## 2. 파일별 상세 분석

### 2.1 `color_filter.py` — HSV 색 필터 기반 빌딩블록

**목적**: 모든 검출/분류의 최하단 도구. HSV 색공간에서 특정 색 범위를 이진 마스크로 뽑아낸다. 벽 영역을 채워진 마스크로 만드는 함수도 제공한다.

**핵심 클래스/함수**
- `ColorFilter(lower_hsv, upper_hsv)` — 생성 시 범위를 numpy 배열로 저장(`color_filter.py:18-19`).
- `ColorFilter.filter(img) -> uint8 mask` — `cv.cvtColor(img, cv.COLOR_BGR2HSV)`로 BGR→HSV 변환 후 `cv.inRange()`로 범위 내 픽셀을 255, 나머지를 0으로 만든다(`color_filter.py:27-29`).
- `get_wall_mask(image) -> bool 배열` — 모듈 전역 `WALL_COLOR_FILTER = ColorFilter((90,44,0),(95,213,158))`(청록 계열 벽색)으로 원시 벽 마스크를 만든 뒤, 좌우 1px margin을 255로 패딩하고 `cv.findContours(RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)` + `cv.fillPoly()`로 **윤곽 내부를 채운** 벽 마스크를 반환한다(`color_filter.py:36-49`). margin 패딩은 화면 가장자리에 걸친 벽 윤곽이 닫히도록 보조하는 트릭이다.

**사용 알고리즘**: HSV 색공간 임계값 필터링(inRange), 컨투어 검출 + 폴리곤 채우기.
**라이브러리**: OpenCV(`cv.cvtColor`, `cv.inRange`, `cv.findContours`, `cv.fillPoly`), numpy.
**입출력**: 입력 = BGR 이미지(Webots 카메라). 출력 = uint8(0/255) 또는 bool 마스크.

> 주의(코드 관찰): Webots 카메라는 통상 BGRA이며, 코드 주석은 "BGR 형식"으로 가정한다(`color_filter.py:26`). HSV 임계값은 OpenCV 관례(H 0–179)를 따른다.

---

### 2.2 `non_fixture_filterer.py` — 배경(비-토큰) 색 마스킹

**목적**: 벽·바닥·장애물·구멍·빨간타일·체크포인트 같은 **알려진 배경 색**을 True로 표시한다. 분류 단계에서 배경 픽셀이 토큰 색으로 오인식되는 것을 막는다.

**핵심 클래스/함수**
- `NonFixtureFilter()` — 7개의 `ColorFilter`를 튜플로 보유(`non_fixture_filterer.py:17-25`): 벽(청록), 일반 바닥(흰), 장애물(회색), 외부 구멍(매우 어두운 검정), 내부 구멍(중간 회색), 빨간 타일, 체크포인트(보라/파랑).
- `filter(image) -> bool 마스크` — 모든 배경 필터를 OR 누적(`base += filtered`)하여 합친다(`non_fixture_filterer.py:27-38`).

**사용 알고리즘**: 다중 HSV 임계값 마스크의 논리합(OR).
**라이브러리**: numpy, `ColorFilter`(내부적으로 cv2).
**입출력**: 입력 = BGR 이미지. 출력 = 배경=True 인 bool 마스크. 이 마스크는 `fixture_clasification.find_fixtures`에서 `(non_fixture_by_color == 0)` 형태로 **반전**되어 배경 영역을 제거하는 데 쓰인다(`fixture_clasification.py:109-110`).

> 일부 필터는 lower==upper로 단일 색만 통과시킨다(예: 일반 바닥 `(0,0,192)~(0,0,192)`). 시뮬레이터의 단색 배경에 정확히 맞춘 하드코딩이다.

---

### 2.3 `fixture_detection.py` — FixtureDetector (공간 위치 매핑 + 가짜 필터)

**목적**: 카메라에서 토큰 후보를 찾아 이미지 내 위치를 잡고, 이를 로봇 좌표계 → 픽셀 그리드 좌표로 변환해 `victims` 레이어에 기록한다. v26에서 **가짜 3D victim 필터**, **최소 컨투어 면적 필터**, **벽 근접성 검증**이 추가됐다.

**초기화**(`fixture_detection.py:27-41`)
- `color_filters`: black/white/yellow/red_low/red_high — 토큰 후보 픽셀(검정 글자, 흰 배경, 노랑/빨강 hazmat)을 잡는 5개 HSV 필터.
- `max_detection_distance = 0.12 * 5` = 0.6 m (12cm 타일 5칸). 카메라 정면 레이캐스팅 최대 사거리.
- `min_contour_area = 30`(노이즈 컨투어 제거), `fake_protrusion_threshold = 3`(벽 근접 판정 px 반경).

**핵심 메서드**
- `get_fixture_positions_in_image(image) -> List[Position2D]`(`:104-144`)
  - 5개 color_filter 출력을 bool로 OR 누적 → `image_sum`.
  - `get_wall_mask(image)`로 벽 마스크를 구해 `image_sum *= wall_mask` — **벽 위에 있는 토큰만** 남긴다.
  - `cv.findContours(image_sum, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)`로 후보 컨투어 추출.
  - 필터 2종: `cv.contourArea(c) < 30`이면 버림(노이즈); `cv.boundingRect`로 종횡비 계산해 `aspect > 4.0`(너무 길쭉)이면 버림(`:126-134`).
  - 남은 후보의 bounding box 중심을 `Position2D`로 반환.
- `get_fixture_positions_and_angles(robot_position, camera_image)`(`:43-91`)
  - 입력 이미지를 `np.flip(..., axis=1)`로 좌우 반전(카메라 좌표↔공간 좌표 정합).
  - 각 후보 픽셀 x좌표 → 수평 각도로 환산: `relative_horizontal_angle = position[1] * (horizontal_fov/width)`, FOV 절반을 빼고 카메라 수평 방향을 더해 절대 각도 산출 후 `normalize()`.
  - 카메라 위치(`distance_from_center` 벡터 + 로봇 위치)에서 토큰 각도로 `max_detection_distance`만큼 뻗는 **검출 벡터**를 만든다.
  - `skimage.draw.line()`으로 카메라→검출점 직선의 픽셀들을 따라가며(레이캐스팅), `pixel_grid.arrays["walls"]`가 True인 첫 지점을 벽 충돌점으로 본다. 충돌 2px 전(`back_index = index-2`)을 토큰 위치로 채택(`:71-88`).
  - **가짜 3D victim 필터**: 충돌점에서 `_is_near_wall()`이 False면(벽에서 멀면) break하여 버린다 — 실제 토큰은 벽면에 붙어있고, 가짜 입체 토큰은 벽 밖으로 돌출되므로 레이가 진짜 벽을 만나기 전 허공에 잡히는 것을 제거한다.
- `_is_near_wall(x, y)`(`:93-102`) — `fake_protrusion_threshold`(3px) 반경 정사각 영역에서 `walls` 레이어에 True가 하나라도 있으면 True. `np.any(region)`.
- `map_fixtures(camera_images, robot_position)`(`:146-158`) — 여러 카메라 이미지를 돌며 위치/각도를 구해 그리드 범위 내면 `arrays["victims"]=True`, `arrays["victim_angles"]=angle.radians` 기록.
- `mark_reported_fixture(robot_position, fixture_position)`(`:160-163`) — `skimage.draw.disk(idx, 4)`로 반경 4px 원을 `arrays["fixture_detection"]`에 True 표시(중복 보고 방지용).

**사용 알고리즘**: HSV 다중 필터 OR, 벽마스크 AND, 컨투어 검출 + 면적/종횡비 필터, 카메라 FOV→각도 투영, **직선 레이캐스팅(Bresenham, skimage.draw.line)**, 벽 충돌 검출, 근접성 기반 가짜 판별.
**라이브러리**: OpenCV(`findContours`, `contourArea`, `boundingRect`, `circle`, `imshow`), numpy, skimage.draw(`line`, `disk`), 자체 `Angle`/`Vector2D`/`Position2D`, `CompoundExpandablePixelGrid`.
**입출력**: 입력 = `CameraImage`(이미지 + FOV/방향/너비/중심거리 메타) + 로봇 `Position2D`. 출력 = 토큰 좌표/각도 리스트, 그리고 픽셀 그리드 `victims`/`victim_angles`/`fixture_detection` 레이어 갱신.

---

### 2.4 `fixture_clasification.py` — FixtureClasiffier (종류 판정의 핵심)

**목적**: 잘라낸 토큰 이미지를 받아 보고 글자(H/S/U/F/P/C/O) 또는 `None`을 반환한다. 규정의 토큰 2종 + 가짜 판별을 모두 여기서 처리한다.

**보조 클래스 `FixtureType`**(`:12-23`) — `ranges`(색이름→(min,max) 픽셀 수 dict)를 들고, `is_fixture(colour_counts)`는 모든 색이 범위 안일 때만 True. hazmat/already_detected 규칙 표현에 사용.

**초기화**(`:41-82`)
- `color_filters`: black/white/yellow (기본 색 카운트용). 단, 여기 black 범위는 `(0,0,0)~(0,0,160)`로 `fixture_detection.py`의 black(`~(0,0,9)`)보다 훨씬 넓다(분류용은 회색까지 포함).
- `extra_color_filters`: red_low/red_high/green/blue — Cognitive target 색 판별 및 hazmat 빨강 카운트용.
- `min_fixture_height = 16`, `min_fixture_width_factor = 0.8` — 너무 작은 후보 컷.
- `hazmat_types`(`:59-68`): UN 플래카드 색 픽셀 수 규칙 (우선순위 순)
  - O(organic_peroxide): red≥500 & yellow≥500
  - F(flammable): white≥500 & red≥500
  - C(corrosive): white 700–4500 & black 900–3000
  - P(poison): white 2000–5000 & black 100–1000
- `already_detected_types`(`:71-80`): 이미 표시된(흰 박스로 덮인) 토큰 — white가 매우 많거나 매우 적고 black/red/yellow=0 → None.

**핵심 메서드**
- `find_fixtures(image)`(`:98-122`) — 이미지를 `np.rot90(k=3)` 회전, black/white/yellow 필터를 `sum_images`로 합산(`:84-89`, 255 클램프). `get_wall_mask` + `non_fixture_filter`를 결합해 `binary_image *= (walls_mask + (non_fixture==0))`로 벽 위·배경 아닌 영역만 남김. `cv.findContours(RETR_TREE, CHAIN_APPROX_SIMPLE)`로 후보 잘라내고 `filter_fixtures`(`:91-96`)로 최소 크기 컷.
- `count_colors(image) -> dict`(`:124-131`) — 기본+추가 필터별 `np.count_nonzero`. red_low+red_high를 합쳐 `"red"` 키로 통합.
- **`classify_fixture(fixture) -> str|None`**(`:133-177`) — 핵심 분기. 우선 `cv.resize(image,(100,100), INTER_AREA)`로 정규화 후 `count_colors`:
  1. `already_detected` 매칭 → `None`.
  2. **green>50 또는 blue>50 → `_classify_cognitive_target`**(동심원). (규정상 동심원은 검/빨/노/초/파 5색을 쓰므로 초/파 존재가 동심원 신호.)
  3. red>300 또는 yellow>300 → `_classify_hazmat`(UN 플래카드).
  4. black>300 & white>300 → `victim_classifier.classify_victim`(글자 H/S/U).
  5. 그 외 → `None`(랜덤 추측 제거, 오식별 TMI -5점 회피).
- `_classify_hazmat(color_counts)`(`:179-188`) — `hazmat_types`를 순회해 첫 매칭 글자 반환, 없으면 None.
- **`_classify_cognitive_target(fixture)`**(`:190-229`) — Cognitive target 동심원 색 합산:
  - 100x100 리사이즈 후 `cv.cvtColor(BGR2HSV)`.
  - 중심(50,50), `max_r=45`, 링 반경 비율 `[0.92,0.72,0.52,0.32,0.12]`(바깥→안쪽 5개 링).
  - 각 링에서 45° 간격 8방향 샘플의 HSV `np.median`을 취하고 `_hsv_to_color_code`로 K/R/Y/G/B 판정.
  - 색 값 `{K:-2, R:-1, Y:0, G:1, B:2}` 누적 → `score_sum`.
  - **합계 0~3 → `['F','P','C','O'][score_sum]`. 음수 또는 >3 → `None`(가짜)**. 규정의 "색값 합 0=F,1=P,2=C,3=O, 미일치=가짜"를 그대로 구현.
- `_hsv_to_color_code(hsv_val)`(`:231-246`) — v<60 또는 s<50이면 K(검정/무채색), h≤10 또는 ≥170 → R, 20–35 → Y, 40–85 → G, 100–130 → B, 그 외 K.

**사용 알고리즘**: HSV 다중 필터 합산, 벽마스크/배경마스크 결합, 컨투어 검출 + 크기 필터, **색 픽셀 수(histogram-like count) 규칙 기반 분류**, **동심원 방사형 샘플링 + 색값 합산(concentric ring color-sum)**, 중앙값(median) 노이즈 강건화.
**라이브러리**: OpenCV(`resize`, `findContours`, `boundingRect`, `cvtColor`, `imshow`), numpy(`rot90`, `count_nonzero`, `median`, `clip`, `radians`, `cos/sin`), math(`inf`).
**입출력**: 입력 = 토큰 dict `{"image":..., "position":...}`. 출력 = 보고 글자 문자열 또는 None.

---

### 2.5 `victim_clasification.py` — VictimClassifier (Φ/Ψ/Ω → H/S/U)

**목적**: 흑백 글자 victim 심볼을 형태로 구분한다. **템플릿매칭/ML이 아니라 컨투어 계층 + 픽셀 밀도 휴리스틱**을 쓴다.

**초기화**(`:25-27`) — `victim_letter_filter = ColorFilter((0,0,0),(0,0,130))`(검정 글자 추출), `min_hole_area = 80`.

**핵심 메서드**
- `isolate_victim(image)`(`:29-34`) — 검정 필터로 이진화 후 `get_biggest_blob`로 최대 글자 영역 추출.
- `get_biggest_blob(binary)`(`:36-45`) — `cv.findContours(RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)`로 최대 boundingRect 영역을 잘라 반환.
- **`classify_victim(victim)`**(`:47-82`):
  1. 심볼 분리 후 `cv.resize((100,100), INTER_AREA)`.
  2. `_count_holes`로 닫힌 내부 공간 수 계산. **≥1이면 Φ → 'H'**(Φ는 타원 내부에 닫힌 구멍이 있음).
  3. 구멍이 없으면 하단 중앙 영역 `letter[70:95, 35:65]`의 픽셀 밀도(`count_nonzero/size`)로 Ψ vs Ω 구분: **밀도>0.25 → Ψ → 'S'**(하단 세로바 존재), 아니면 **Ω → 'U'**(두 발 사이 간격).
- `_count_holes(binary_100x100)`(`:84-98`) — `cv.findContours(cv.RETR_CCOMP, CHAIN_APPROX_SIMPLE)`로 2단 계층을 얻어, `hierarchy[i][3] != -1`(부모 있음=내부 컨투어=구멍)이고 `cv.contourArea ≥ 80`인 것만 카운트.

**사용 알고리즘**: 이진화, 최대 blob 추출, **컨투어 계층 분석(RETR_CCOMP로 구멍 검출)**, **영역 픽셀 밀도 휴리스틱**. (외부 학습 모델/템플릿 없음 — 순수 형태 규칙.)
**라이브러리**: OpenCV(`findContours`, `boundingRect`, `resize`, `contourArea`, `imshow`), numpy(`count_nonzero`, `size`), `ColorFilter`.
**입출력**: 입력 = 토큰 dict. 출력 = 'H'/'S'/'U' 또는 None(blob 없을 때).

---

## 3. 규정 연관성 (2026)

- **벽 토큰 2종 구현**:
  - Letter victim Φ/Ψ/Ω → H/S/U: `VictimClassifier.classify_victim`(구멍 유무 + 하단 밀도)로 종류(TT) 판정.
  - Cognitive target 동심원: `_classify_cognitive_target`이 규정의 색값(검-2/빨-1/노0/초1/파2) 5링 합산 → 합 0~3=F/P/C/O를 정확히 구현. UN 플래카드형 hazmat는 `_classify_hazmat`이 색 픽셀 수로 F/P/C/O 판정.
- **가짜 토큰 처리(TMI -5점 회피)**:
  - 가짜 3D victim: `FixtureDetector._is_near_wall`이 벽에서 돌출(>3px)된 감지를 제거(`fixture_detection.py:80-84, 93-102`).
  - 가짜 동심원: 색합산이 범위(0~3) 밖이면 None 반환(`fixture_clasification.py:221-224`).
  - 불확실 시 모두 `None` 반환(`classify_fixture` 5단계, `_classify_hazmat` 매칭 실패) → 랜덤 추측 금지로 오식별 점수 손실 방지.
- **토큰 식별 위치 정확도(반 타일 이내)**: `get_fixture_positions_and_angles`의 카메라 FOV 투영 + 레이캐스팅으로 토큰을 벽면 픽셀에 정확히 고정(`max_detection_distance=0.6m`).
- **맵 행렬 토큰 코드**: `map_fixtures`가 `victims`/`victim_angles` 레이어에 위치/방향을 기록 → 후단(fixture_mapper/FinalMatrixCreator)에서 맵 행렬의 토큰 코드(H,S,U,F,P,C,O)로 변환되는 입력.
- **중복 보고 방지**: `mark_reported_fixture`가 보고 완료 토큰을 `fixture_detection` 레이어에 마킹.
- **센서 노이즈 강건성(v26 GPS 노이즈 등)**: 동심원 8방향 `np.median` 샘플링, 최소 면적/종횡비/벽근접 필터가 노이즈·오탐을 억제.

---

## 4. 다른 모듈과의 상호작용

**이 모듈이 호출하는 것**
- `data_structures.vectors`(Position2D, Vector2D), `data_structures.angle`(Angle): 좌표/각도 계산.
- `data_structures.compound_pixel_grid.CompoundExpandablePixelGrid`: `walls`/`victims`/`victim_angles`/`fixture_detection` 레이어 읽기·쓰기 및 좌표↔배열 인덱스 변환.
- `robot.devices.camera.CameraImage`: 이미지 + FOV/방향/너비/중심거리 메타데이터.
- OpenCV(cv2), numpy, skimage.draw, math.
- `flags.SHOW_FIXTURE_DEBUG`: 디버그 시각화/로그 토글.
- 내부 의존: `FixtureClasiffier` → `VictimClassifier`, `ColorFilter`/`get_wall_mask`, `NonFixtureFilter`. `FixtureDetector` → `ColorFilter`/`get_wall_mask`.

**이 모듈을 호출하는 것(아키텍처상)**
- **Mapper/fixture_mapper**: 매핑 루프에서 `FixtureDetector.map_fixtures(camera_images, robot_position)`를 호출해 토큰 위치를 픽셀 그리드에 기록(아키텍처 문서의 "fixture_mapper(조난자 위치 기록)"와 직결).
- **Executor/Agent(GoToFixtures 서브에이전트)**: 토큰 앞에서 1초 정지 후 `FixtureClasiffier.find_fixtures` + `classify_fixture`로 보고 글자를 얻고, `comunicator.send_victim`으로 Erebus 게임매니저에 전송. 보고 후 `mark_reported_fixture`로 중복 방지.
- **FinalMatrixCreator**: `victims` 레이어를 맵 행렬의 토큰 코드로 변환(매핑 보너스 MB 입력).

---

## 5. 코드 관찰 / 잠재적 이슈

- `get_wall_mask` 반환은 bool인데 `fixture_detection.py:116`에서 `image_sum *= wall_mask`로 곱한다(uint8 255 * bool → 정상 동작하나 dtype 의존적). `fixture_clasification.py:110`은 `(walls_mask + (non_fixture==0))`로 bool 합산 후 uint8 binary와 곱한다 — 1을 넘을 수 있어 마스킹이라기보다 "둘 중 하나라도 통과"에 가깝다.
- `count_colors`에서 `pop("red_low",0)`는 dict를 변형하므로 호출마다 키가 사라진다(매 호출 새 dict라 문제 없으나 의도 확인 권장).
- 동심원 샘플 좌표는 `x=cx+r*cos`, `y=cy+r*sin`인데 인덱싱은 `hsv[y, x]`(행=y) — 좌표계 일관성 OK.
- victim 분류는 ML/템플릿이 아닌 순수 형태 휴리스틱(구멍 수 + 하단 밀도)으로, 노이즈/회전에 다소 취약할 수 있다(밀도 임계 0.25 하드코딩).
