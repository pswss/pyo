# `mapping` 모듈 상세 분석 — RoboCup Rescue Simulation 2026

## 1. 폴더/모듈 개요

`mapping` 모듈은 이 로봇의 **SLAM(동시적 위치추정 및 지도작성) 중 "매핑(Mapping)" 부분**을 전담한다. 위치추정(localization)은 `robot/pose_manager`가 GPS+자이로로 처리해 주고, `mapping`은 그 위치를 신뢰한다는 전제 아래 **LiDAR / RGB 카메라 / 컬러센서 / GPS 데이터를 하나의 다층(多層) 픽셀 그리드 지도에 누적**하는 일을 한다. 결과물인 픽셀 그리드는 두 가지 용도로 흘러간다. (1) `pathfinding`(A*/BFS)이 이 그리드의 `traversable` / `navigation_preference` 레이어를 읽어 경로를 계산하고, (2) `FinalMatrixCreator`가 이 그리드를 규정의 맵 행렬(벽=1, 구멍=2, 늪=3, 체크포인트=4 …)로 변환해 매핑 보너스(MB) 점수를 받는다.

핵심 설계 사상은 **"단일 진실 원천(single source of truth)인 `CompoundExpandablePixelGrid`에 모든 레이어를 모아두고, 각 `*_mapper`가 자기 담당 레이어만 갱신한다"**는 것이다. `Mapper`(mapper.py)가 오케스트레이터로서 매 타임스텝 각 하위 맵퍼를 정해진 순서로 호출한다. 좌표계는 3단계 — **실제 좌표(m) → 그리드 인덱스(시작점 상대) → 배열 인덱스(numpy 0-기반)** — 로 나뉘며, 모든 변환은 `compound_pixel_grid.py`가 제공한다. 지도는 고정 크기가 아니라 로봇이 새 영역을 보면 numpy `vstack`/`hstack`으로 **동적으로 확장**된다. 거의 모든 연산이 **numpy 벡터화 + OpenCV(cv2) 컨볼루션/모폴로지/원근변환**으로 구현돼 있어, 매 스텝 픽셀 단위 루프를 도는 대신 배열 마스크 연산으로 처리한다 — 이것이 노이즈 강건성과 실시간성을 동시에 잡는 핵심이다.

> 픽셀 해상도: `pixels_per_tile = 10`, `quarter_tile_size = tile_size / 2 = 0.06m`. 따라서 `resolution = 10 / 0.06 ≈ 166.7 px/m`. 즉 1cm ≈ 1.67px, 12cm 타일 ≈ 20px, 1cm 두께 벽 ≈ 1.67px 폭으로 표현된다.

---

## 2. 파일별 상세 분석

### 2.1 `mapper.py` — 오케스트레이터 (`Mapper`)

**목적.** 모든 하위 맵퍼·추출기·필터를 조립하고, 매 타임스텝 `update()`에서 정해진 순서로 호출해 픽셀 그리드를 갱신하는 최상위 클래스.

**핵심 시그니처.**
- `__init__(self, tile_size, robot_diameter, camera_distance_from_center)` — `CompoundExpandablePixelGrid`(공유 그리드)와 `TileColorExpandableGrid`를 만들고, `WallMapper`/`FloorMapper`/`OccupiedMapper`/`RobotMapper`/`FixtureMapper`/`ArrayFilterer`/`PointCloudExtarctor`/`FloorColorExtractor`/`FixtureDetector`에 **동일한 `self.pixel_grid` 인스턴스를 주입**한다(mapper.py:52-85). 모든 맵퍼가 같은 그리드 객체를 공유하는 것이 이 구조의 핵심.
- `update(in_bounds_point_cloud, out_of_bounds_point_cloud, lidar_detections, camera_images, robot_position, robot_orientation, time)` — 메인 갱신 루프(mapper.py:95-147).
- `register_start(robot_position)` — 시작 위치 `deepcopy` 기록(복귀 경로용, mapper.py:149-152).
- `has_detected_victim_from_position()` — 현재 위치에서 이미 조난자를 보고했는지 `robot_detected_fixture_from` 레이어로 확인(중복 보고 방지, mapper.py:160-163).
- `is_close_to_swamp()` — 로봇 주변 약 2cm(`swamp_check_area = 0.02`) 박스 영역에 `swamps`가 있는지 `np.any`로 검사(mapper.py:165-183).

**사용 알고리즘.**
- **고정 순서 파이프라인 (상태 없는 오케스트레이션).** `update()`는 매 스텝 (1) LiDAR→벽, (2) 로봇 궤적/시야, (3) 조난자 마진 영역, (4) 바닥색, (5) 조난자 위치, (6) 점유 영역, (7) 노이즈 제거 순으로 실행한다(mapper.py:116-141). 순서가 중요한데, 예컨대 `occupied`는 (1)에서 만든 `walls`와 (4)에서 만든 `holes`에 의존하므로 둘 뒤에 와야 하고, 노이즈 제거(7)는 가장 마지막이다.
- **방어적 입력 처리.** `robot_position`/`robot_orientation`이 없으면 즉시 반환(mapper.py:109-110). 흔들릴 때(자이로 신뢰도 저하 등)는 LiDAR 없이 카메라만으로 호출되기도 하므로, point_cloud / camera_images 각각을 `if not None`으로 가드한다(mapper.py:117, 130, 134).

**사용 기능/라이브러리.** `numpy`(마스크/슬라이싱), `cv2`(`waitKey`), `copy.deepcopy`.

**입출력 데이터 흐름.** 입력 = Executor가 모은 센서 데이터(point cloud, camera images, pose). 출력 = 갱신된 `self.pixel_grid`(부수효과). `is_close_to_swamp()`는 `pose_manager`에 GPS 대신 자이로만 쓰도록 신호를 주는 데 사용된다(규정의 v26 GPS 노이즈/늪 대응).

---

### 2.2 `data_extractor.py` — LiDAR/바닥 원시 추출 (`PointCloudExtarctor`, `FloorColorExtractor`)

**목적.** 포인트 클라우드와 바닥 이미지에서 **타일 단위의 의미(벽 방향, 타일 색종류)를 추출**하는 보조 클래스. 코드 주석상 두 클래스 모두 현재는 `WallMapper`/`FloorMapper`로 대체되어 **직접 사용되지 않을 가능성**이 명시돼 있다(data_extractor.py:13, 152-153). 다만 `Mapper.__init__`이 여전히 인스턴스를 생성하므로(mapper.py:82-83) 분석 대상에 포함한다.

**`PointCloudExtarctor` 핵심.**
- `get_tile_status(min_x, min_y, max_x, max_y, point_cloud)` / `transform_to_grid(point_cloud)`.
- **사용 알고리즘 — 템플릿 매칭(template matching).** 7×7 정수 가중치 템플릿 두 종류를 만든다: 직선 벽(`straight_template`, 위쪽 2행에 1·2 가중치)와 코너 벽(`curved_template`). 이를 `np.rot90`으로 90°씩 돌려 직선 4방향(u/l/d/r)과 코너 4방향(ul/dl/dr/ur), 총 8개 템플릿 딕셔너리를 만든다(data_extractor.py:191-196). 한 타일에서 포인트 클라우드의 비영(非零) 위치(`np.where(square != 0)`)에 대해 각 템플릿 값을 합산하고(`np.sum(template[non_zero_indices])`), 합이 `threshold=8` 이상인 방향만 그 타일의 벽 방향으로 채택한다(data_extractor.py:209-217). 즉 **회전 불변 패턴 매칭으로 벽의 위상(상하좌우/코너)을 분류**한다.

**`FloorColorExtractor` 핵심.**
- `get_square_color`, `get_sq_color`, `get_floor_colors(floor_image, robot_position)`.
- **사용 알고리즘 — HSV 색공간 범위 필터링 + 임계 비율 투표.** 타일 영역을 `cv.cvtColor(..., COLOR_BGR2HSV)`로 바꾸고, `floor_color_ranges` 딕셔너리(normal/hole/swamp/checkpoint/connection1-2/1-3/2-3)의 각 HSV 범위에 대해 `cv.inRange` 마스크의 `count_nonzero`를 센다. 영역 면적의 `threshold`(대개 0.2)를 넘는 색만 후보로 남기고 그중 최다 색을 그 타일 종류로 선택한다(data_extractor.py:74-83). connection1-2(파랑)/1-3(노랑)/2-3(보라) 범위는 규정의 **구역 간 색상 통로**를 직접 인코딩한다.
- **그리드 정렬 오프셋.** `robot_position`을 0.06(쿼터타일)로 모듈러 연산해 타일 격자가 로봇 위치에 정렬되도록 오프셋을 계산한다(data_extractor.py:105-111).

**사용 기능/라이브러리.** `numpy`, `cv2`(cvtColor/inRange/rectangle/imshow), 프로젝트 `utilities`(get_squares/draw_grid 등).

**입출력 흐름.** 입력 = bool 포인트클라우드 배열 / BGR 바닥 이미지. 출력 = `(타일인덱스, 색키)` 리스트 또는 벽 방향 그리드. (현 파이프라인에서는 산출물이 그리드에 직접 반영되지 않는, 레거시/보조 경로로 보임.)

---

### 2.3 `wall_mapper.py` — LiDAR→벽 레이어 (`WallMapper`)

**목적.** LiDAR 포인트 클라우드를 받아 `walls`, `detected_points`, `seen_by_lidar`, `traversable`, `navigation_preference`, `walls_seen_by_camera` 레이어를 갱신. **이 모듈에서 노이즈 강건성이 가장 핵심적으로 구현된 곳.**

**핵심 시그니처.**
- `load_point_cloud(in_bounds_pc, out_of_bounds_pc, robot_position)` — 진입점, 매 스텝 호출(wall_mapper.py:52).
- `load_in_bounds_point_cloud(...)` / `load_out_of_bounds_point_cloud(...)`.
- `occupy_point(point_array_index)` / `mark_point_as_seen_by_lidar(...)` / `filter_out_noise()` / `generate_navigation_margins()` / `calculate_seen_walls()`.

**사용 알고리즘.**
- **누적 카운팅 + 임계화(thresholding)로 벽 확정.** 각 LiDAR 포인트는 `detected_points` 카운터를 +1 하고, **3회(`to_boolean_threshold = 3`)를 넘겨야** 비로소 `walls = True`로 확정된다(wall_mapper.py:148-158). 단발성 노이즈 포인트는 벽이 되지 못한다. 반대로 카운트가 1(`delete_threshold = 1`) 이하인 포인트는 `filter_out_noise()`에서 0으로 지워진다(wall_mapper.py:132-136). 또한 이미 로봇이 지나간(`traversed`) 곳에는 벽을 찍지 않는다(wall_mapper.py:157) — 로봇이 통과한 자리에는 벽이 있을 수 없으므로.
- **Bresenham 직선 (빔 궤적).** `skimage.draw.line`으로 로봇 위치→포인트 사이 직선을 `seen_by_lidar`에 True로 긋는다. **끝 2픽셀은 제외**(`indexes[..][:-2]`)해 포인트(벽) 자체는 "관통한 자유공간"으로 오기록하지 않는다(wall_mapper.py:165-169). `seen_by_lidar`는 매 프레임 시작 시 0으로 초기화(wall_mapper.py:171-173)되어 "이번 스텝에 빔이 닿은 곳"만 표현한다.
- **모폴로지 팽창 (cv2 filter2D + 원형 커널).** `generate_navigation_margins()`는 `occupied`를 로봇 반경 원형 커널(`traversable_template`, 통로 통과를 위해 반경 1px 축소)로 `cv.filter2D` 컨볼루션해 **벽을 로봇 크기만큼 팽창시킨 `traversable`(통과 불가)** 레이어를 만든다(wall_mapper.py:115-126). 이는 경로탐색이 로봇을 점으로 다루도록 하는 표준 기법(C-space 팽창).
- **2차 원형 그라디언트 (벽 회피 선호도).** `__generate_quadratic_circle_gradient(min_r, max_r)`는 동심원을 안쪽일수록 큰 값(`max_radius² − i²`)으로 채운 그라디언트 커널을 만들고(wall_mapper.py:138-146), 이를 `occupied`에 컨볼루션해 **벽 근처일수록 높은 `navigation_preference`** 를 만든다. 늪지대(`swamps`)에도 150을 부여(wall_mapper.py:130) — pathfinding이 비용 함수로 사용해 벽/늪을 부드럽게 회피한다.
- **카메라-LiDAR 교차로 "본 벽" 분류.** `calculate_seen_walls()`는 `seen_by_camera * walls`로 카메라가 실제로 본 벽을, `logical_xor`로 못 본 벽을 분리한다(wall_mapper.py:107-113). 조난자가 벽에 붙어 있으므로 "카메라로 확인된 벽"이 조난자 탐색의 단서가 된다.

**사용 기능/라이브러리.** `numpy`, `cv2`(circle/filter2D), `skimage.draw.line`(Bresenham).

**입출력 흐름.** 입력 = (로봇 상대 좌표의) in/out-of-bounds 포인트 리스트 + 로봇 절대 위치. 포인트는 `p + robot_position`으로 절대 좌표화 후 그리드 인덱스로 변환, 필요 시 `expand_to_grid_index`로 그리드 확장. 출력 = 위 6개 레이어(부수효과).

---

### 2.4 `floor_mapper.py` — 카메라→바닥색 레이어 (`FloorMapper`, `ColorFilter`)

**목적.** 3개 RGB 카메라 영상을 **역투시변환(IPM, Inverse Perspective Mapping)으로 탑뷰(조감도)로 펴서** 바닥 색상을 픽셀 그리드에 누적하고, 색상에서 구멍/늪/체크포인트 타일을 판별.

**핵심 시그니처.**
- `ColorFilter(lower_hsv, upper_hsv).filter(img)` — HSV `inRange` 마스크(floor_mapper.py:10-20).
- `flatten_camera_pov(camera_pov)` / `set_in_background(pov)` / `rotate_image_to_angle(image, angle)` / `get_unified_povs(camera_images)` / `map_floor(camera_images, robot_grid_index)` / `load_povs_to_grid(...)`.
- `detect_holes()` / `detect_swamps()` / `detect_checkpoints()` / `get_squares_from_raw_array(...)` / `get_tile_centers_from_raw_array(...)` / `load_average_tile_color()`.

**사용 알고리즘.**
- **역투시변환(IPM).** `cv.getPerspectiveTransform`으로 카메라 원본 사다리꼴 4점(`center_tile_points_in_input_image`)을 탑뷰 직사각형 4점(`center_tile_points_in_final_image`)에 매핑하는 3×3 호모그래피 행렬(`DECOMP_SVD`)을 구하고, `cv.warpPerspective`로 영상을 평면화한다(floor_mapper.py:81-87). 카메라가 로봇 중심에서 떨어진 만큼(`pov_distance_from_center = 0.079m`) 위에 빈 공간을 `np.vstack`으로 덧대 로봇 중심 기준으로 정렬한다(floor_mapper.py:89-91).
- **다중 카메라 합성 + 방향 회전.** `get_unified_povs()`는 카메라별로 IPM→배경배치(`set_in_background`)→`imutils.rotate`로 전역 방향 회전 후, 3개 뷰를 단순 `sum`으로 합친다(floor_mapper.py:126-142). 로봇 방향에 따라 회전하므로 전역 좌표계에 정렬된 바닥 모자이크가 만들어진다.
- **거리 가중 갱신 (가까운 관측 우선).** `__get_distance_to_center_gradient`로 중심에 가까울수록 큰 값(거리²의 역수 정규화)인 그라디언트를 만들고(floor_mapper.py:197-210), 새 관측의 그라디언트가 기존 `floor_color_detection_distance`보다 클 때만 색을 덮어쓴다(floor_mapper.py:176, 185-189). 즉 **로봇에 가까워 더 또렷한 관측이 멀리서 본 흐린 관측을 이긴다.** 추가 마스크: `seen_by_camera`(카메라가 본 곳만) AND 알파>254(유효 픽셀) AND `wall_color_filter` 아님(벽색을 바닥색으로 오기록 방지). 세 조건의 곱(`final_mask`)으로 갱신(floor_mapper.py:169-189).
- **타일 단위 비율 투표로 종류 판정.** `detect_holes/swamps`는 HSV 필터 결과를 `get_squares_from_raw_array`로 타일 단위로 나눠, True 비율이 임계(구멍 0.2, 늪 0.3)를 넘으면 타일 전체(+margin)를 True로 칠한다(floor_mapper.py:263-291). `detect_checkpoints`는 `get_tile_centers_from_raw_array`로 비율 0.3 초과 시 **타일 중심점 하나만** True로 찍어 위치를 점으로 표현한다(floor_mapper.py:293-320). 구멍은 `hole_detections`에 매 스텝 누적(`+=`)해 일시적 오탐에 강하게 만든다(floor_mapper.py:250).
- **타일 평균색 (제출용).** `load_average_tile_color()`는 타일 80% 크기 평균 커널로 `cv.filter2D` 평활화 후 타일 중심색을 추출, `INTER_NEAREST`로 재확대해 `average_floor_color`에 저장(floor_mapper.py:323-355) — 최종 맵 행렬의 타일색 결정에 쓰인다.

**사용 기능/라이브러리.** `cv2`(getPerspectiveTransform/warpPerspective/resize/filter2D/inRange/cvtColor), `numpy`(vstack/flip/마스크), `imutils.rotate`.

**입출력 흐름.** 입력 = `CameraImage` 리스트(`.image`, `.data.horizontal_orientation`) + 로봇 그리드 인덱스. 출력 = `floor_color`, `floor_color_detection_distance`, `holes`/`hole_detections`, `swamps`, `checkpoints`, `average_floor_color` 레이어.

---

### 2.5 `robot_mapper.py` — 로봇 궤적/시야 레이어 (`RobotMapper`)

**목적.** 로봇이 지나간 영역(`traversed`), 카메라가 본 영역(`seen_by_camera`), 탐색된 영역(`discovered`)을 기록. **자유공간(free space)과 탐색 진척도를 표현하는 레이어 담당.**

**핵심 시그니처.**
- `map_traversed_by_robot(robot_grid_index)` / `map_traversed_by_center_of_robot(...)`.
- `map_seen_by_camera(robot_grid_index, robot_rotation)` / `map_discovered_by_robot(robot_grid_index, robot_rotation)`.
- `__get_cone_template(lenght, orientation, amplitude)`(내부) / `__get_circle_template_indexes(radius)`(내부).

**사용 알고리즘.**
- **원형 스탬프(궤적).** `cv.circle`로 만든 로봇 반경 원의 내부 인덱스를 미리 캐시(`__robot_diameter_indexes`)해 두고, 매 스텝 로봇 위치 오프셋을 더해 `traversed`에 True를 찍는다(robot_mapper.py:42-57). 별도로 약 2cm 중심 반경 원을 `robot_center_traversed`에 찍는다 — 좁은 영역 판단용.
- **부채꼴(원뿔) 시야 마스크 = 원 AND 삼각형.** `__get_cone_template`은 (1) `cv.circle`로 반경=시야거리인 원 마스크와 (2) 시야각(`amplitude`) 양 끝/중앙 방향 벡터(`Vector2D(angle, len)`)로 `cv.fillPoly`한 삼각형 마스크를 만들어, **둘을 곱해(`triangle * circle`) 부채꼴**을 얻는다(robot_mapper.py:124-168). 카메라 시야(`amplitude=25°`, 3개 방향 0°/270°/90°)와 넓은 탐색 시야(`discovery amplitude=170°`)에 각각 사용.
- **LiDAR 교차로 실제 가시영역 한정.** `seen_by_camera`와 `discovered`는 부채꼴 인덱스 위치에 **`seen_by_lidar`를 더한다**(robot_mapper.py:95-96, 121-122). 즉 "시야각 안" AND "이번 스텝 LiDAR 빔이 실제 도달한 곳"만 본 것으로 인정 — 벽 뒤를 본 것으로 착각하지 않게 하는 가시성(occlusion) 처리.

**사용 기능/라이브러리.** `numpy`, `cv2`(circle/fillPoly), `math`, 프로젝트 `data_structures`(Angle/Position2D/Vector2D).

**입출력 흐름.** 입력 = 로봇 그리드 인덱스 + 방향(Angle). 출력 = `traversed`, `robot_center_traversed`, `seen_by_camera`, `discovered`. `discovered`는 `agent`의 GoToNonDiscovered 서브에이전트가 미탐색 영역을 찾는 데 직접 쓰인다.

---

### 2.6 `fixture_mapper.py` — 조난자 위치 보조 레이어 (`FixtureMapper`)

**목적.** 조난자(victim/hazmat)가 **벽에 붙어 있다**는 규정 사실을 이용해, 벽 바로 바깥의 "조난자가 있을 수 있는 마진 영역"을 만들고, 오탐 마커를 청소하고, 보고 위치를 기록해 중복 보고를 막는다.

**핵심 시그니처.**
- `generate_detection_zone()` / `clean_up_fixtures()` / `map_detected_fixture(robot_position)`.

**사용 알고리즘.**
- **링(고리) 커널 컨볼루션으로 벽 바깥 마진 추출.** `fixture_distance_margin_template`은 반경 5cm 원 내부를 `-50`으로 채우고 **테두리만 `+1`** 로 둔 int8 커널이다(fixture_mapper.py:28-34). 이를 `walls`에 `cv.filter2D`하면, 벽에서 정확히 5cm 떨어진 고리 위치만 양수가 되어(`> 0`) `fixture_distance_margin`이 된다(fixture_mapper.py:38-42). 벽 자신이나 벽에서 너무 가까운/먼 곳은 음수가 되어 배제된다 — **벽과 일정 거리 떨어진 도달 가능 위치**를 골라내는 영리한 모폴로지 트릭. `agent`의 GoToFixtures가 이 영역으로 접근한다.
- **점유 마스크로 오탐 청소.** `clean_up_fixtures()`는 `occupied`(벽/구멍) 위에 잘못 찍힌 `victims`/`hazmats`를 False로 지운다(fixture_mapper.py:44-47).
- **원형 스탬프로 보고 위치 기록.** `map_detected_fixture`는 보고 시 로봇 주변 반경(`detected_from_radius`, 10cm) 원을 `robot_detected_fixture_from`에 찍어, 같은 자리 재보고를 막는다(fixture_mapper.py:49-56). 규정의 **반 타일 이내 인정 / 중복 보고 무효** 조건과 직접 연결.

**사용 기능/라이브러리.** `numpy`, `cv2`(circle/filter2D), `math`.

**입출력 흐름.** 입력 = `walls`/`occupied` 레이어 + 로봇 위치. 출력 = `fixture_distance_margin`, 청소된 `victims`/`hazmats`, `robot_detected_fixture_from`.

---

### 2.7 `array_filtering.py` — 노이즈 제거 필터 (`ArrayFilterer`)

**목적.** 픽셀 그리드의 **고립 노이즈 점을 주기적으로 제거.** LiDAR 노이즈 강건성의 2차 방어선(1차는 WallMapper의 카운팅 임계화).

**핵심 시그니처.**
- `remove_isolated_points(pixel_grid)` / `smooth_edges(array)`(미사용).

**사용 알고리즘.**
- **라플라시안형 고립점 검출 커널.** `[[-2,-2,-2],[-2,1,-2],[-2,-2,-2]]` 커널을 `occupied`에 `cv.filter2D` 한다(array_filtering.py:20-22, 41-43). 중앙만 켜져 있고 주변 8픽셀이 모두 비면 결과가 양수(`> 0`)가 되어 **이웃 없는 단독 점**으로 판정, 해당 위치의 `occupied`/`walls`/`holes`/`detected_points`를 모두 0으로 지운다(array_filtering.py:44-47). 주변에 이웃이 하나라도 있으면 음수가 되어 살아남는다 — 진짜 벽(연결된 픽셀 덩어리)은 보존하고 떠다니는 노이즈만 제거.
- **주기 실행 (StepCounter).** `StepCounter(20)`으로 20스텝마다 한 번만 실행(array_filtering.py:35, 40) — 매 스텝 돌리는 비용을 줄이면서 통로 근처 오탐 벽을 빠르게 청소한다.
- `smooth_edges`(엣지 스무딩, `missing_point_filler_kernel`)는 정의돼 있으나 현재 호출되지 않는다(array_filtering.py:50-56).

**사용 기능/라이브러리.** `numpy`, `cv2.filter2D`, 프로젝트 `flow_control.StepCounter`.

**입출력 흐름.** 입력/출력 모두 공유 `pixel_grid`(부수효과). `Mapper.update` 마지막 단계에서 호출.

---

### 2.8 `occupied_mapping.py` — 통합 점유 레이어 (`OccupiedMapper`)

**목적.** 경로탐색이 쓸 최종 "이동 불가" 레이어(`occupied`)를 합성.

**핵심 시그니처.** `map_occupied()`.

**사용 알고리즘.**
- **불리언 OR + 마스크 차감.** `occupied = walls OR holes`(`np.bitwise_or`)로 벽과 구멍을 합치고(occupied_mapping.py:18-20), 그다음 **로봇이 실제 지나간 `traversed` 위치는 다시 False로** 만든다(occupied_mapping.py:23). 로봇이 통과한 곳은 정의상 통행 가능하므로, 센서 오탐으로 잘못 찍힌 벽/구멍을 사후 정정하는 효과 — 노이즈 강건성의 3차 방어선.

**사용 기능/라이브러리.** `numpy`(bitwise_or, 불리언 인덱싱).

**입출력 흐름.** 입력 = `walls`/`holes`/`traversed`. 출력 = `occupied`(WallMapper의 `traversable`/`navigation_preference`, ArrayFilterer, pathfinding이 모두 이걸 읽음).

---

## 3. 규정(2026) 연관성

| 규정 항목 | 구현 위치 |
|---|---|
| **매핑 보너스 MB** (픽셀그리드→맵행렬 정확도) | 전체 `mapping` 모듈이 픽셀 그리드를 누적, `FinalMatrixCreator`가 변환. `floor_mapper.load_average_tile_color`가 타일색 행렬 근거 제공 |
| **벽=1** (두께 1cm, 높이 6cm) | `wall_mapper`의 `walls`/`occupied`. resolution≈166.7px/m라 1cm 벽≈1.67px |
| **구멍=2** (검은 테두리, LoP 위험) | `floor_mapper.detect_holes`(HSV 검정 0.2 비율), `hole_detections` 누적 |
| **늪=3** (갈색, 시간 5~10배) | `floor_mapper.detect_swamps`(HSV 갈색 0.3 비율) + `navigation_preference=150`으로 회피, `mapper.is_close_to_swamp` |
| **체크포인트=4** (은색, +10×AM) | `floor_mapper.detect_checkpoints`(타일 중심점 마킹) |
| **구역 간 색상 통로** (b/y/g/p/o/r) | `data_extractor.FloorColorExtractor`의 connection1-2/1-3/2-3 HSV 범위 |
| **토큰 식별 — 반 타일 이내 / 중복 무효 / TMI(-5)** | `fixture_mapper.map_detected_fixture`(10cm 보고 반경 기록), `mapper.has_detected_victim_from_position`, `fixture_distance_margin`(벽 5cm 마진으로 접근 위치 산출) |
| **센서 노이즈 강건성** (v26 GPS 노이즈 추가) | 다층 방어: ① `wall_mapper` 카운팅 임계화(3회) ② `array_filtering` 고립점 제거 ③ `occupied_mapping` traversed 차감 ④ `floor_mapper` 거리가중 갱신·구멍 누적 ⑤ `is_close_to_swamp`로 늪 근처 GPS→자이로 전환 |
| **데드레커닝/사전매핑 금지** | 모든 레이어가 매 스텝 실시간 센서 입력으로만 누적, 사전 정보 없음 |

> **v26 GPS 노이즈와의 직접 연관:** GPS가 흔들리면 같은 벽이 약간씩 다른 픽셀에 찍힌다. 이를 `to_boolean_threshold=3`(3회 이상 같은 픽셀 누적 필요)과 `delete_threshold=1`(1회짜리 제거), 그리고 `array_filtering`의 고립점 제거가 함께 흡수한다. 늪 근처에서는 GPS 오차가 특히 커지므로 `is_close_to_swamp`가 자이로 전용 모드로 전환을 유도한다.

---

## 4. 다른 모듈과의 상호작용

**호출되는 쪽 (`mapping`을 누가 쓰나):**
- `executor`(메인 루프)가 센서 데이터를 모아 `Mapper.update(...)`를 매 타임스텝 호출하고, `register_start`/`is_close_to_swamp`/`has_detected_victim_from_position`을 사용.
- `pose_manager`/Robot 추상화가 `is_close_to_swamp` 결과로 GPS↔자이로 전환 결정에 활용.
- `agent`의 서브에이전트들이 픽셀 그리드 레이어를 직접 소비: GoToFixtures←`fixture_distance_margin`, FollowWalls←`walls_seen_by_camera`/`walls`, GoToNonDiscovered←`discovered`.
- `pathfinding`(A*/BFS, np_bool_array)이 `traversable`(통과 불가)와 `navigation_preference`(비용 가중)를 읽어 경로 계산.
- `FinalMatrixCreator`가 `occupied`/`holes`/`swamps`/`checkpoints`/`victims`/`hazmats`/`average_floor_color`를 규정 맵 행렬로 변환(MB 점수).

**호출하는 쪽 (`mapping`이 무엇을 쓰나):**
- `data_structures`: `CompoundExpandablePixelGrid`(공유 다층 그리드, 좌표변환·동적확장), `TileColorExpandableGrid`, `Angle`/`Position2D`/`Vector2D`.
- `fixture_detection.FixtureDetector`: 카메라에서 조난자/해즈맷을 찾아 `victims`/`hazmats`/`fixture_detection` 레이어에 등록(`Mapper.update` 5단계).
- `flow_control.StepCounter`: ArrayFilterer의 주기 실행.
- 외부 라이브러리: `numpy`, `cv2`(원근변환/모폴로지/컨볼루션), `skimage.draw.line`(Bresenham), `imutils.rotate`.

**모듈 내부 데이터 흐름 요약 (한 스텝):**
```
센서 → Mapper.update
  WallMapper      : LiDAR → walls/detected_points/seen_by_lidar/traversable/navigation_preference
  RobotMapper     : pose  → traversed/seen_by_camera/discovered (∩ seen_by_lidar)
  FixtureMapper   : walls → fixture_distance_margin, victims/hazmats 청소
  FloorMapper     : camera(IPM) → floor_color/holes/swamps/checkpoints
  FixtureDetector : camera → victims/hazmats
  OccupiedMapper  : walls OR holes − traversed → occupied
  ArrayFilterer   : occupied 고립점 제거(20스텝 주기)
→ 갱신된 CompoundExpandablePixelGrid (pathfinding / FinalMatrixCreator가 소비)
```
