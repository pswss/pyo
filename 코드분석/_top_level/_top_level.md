# `src/` 최상위 파일 분석 — 진입점 지도

> RoboCupJunior Rescue Simulation 2026 / Webots·Erebus 자율 구조 로봇 코드
> 분석 대상: `run.py`, `main.py`, `rescue_robot.py`, `flags.py`, `utilities.py`, `map_visualizer.py`, `student_example.py`

---

## 1. 폴더/모듈 개요

`src/` 최상위 파일들은 **시스템의 진입점(entry point)과 공용 인프라**를 담당한다. 실제 자율 주행 두뇌(상태머신, 매핑, 경로탐색, 조난자 인식)는 `executor/`, `mapping/`, `robot/`, `final_matrix_creation/` 등 하위 패키지에 들어 있고, 최상위 파일들은 그 복잡한 모듈들을 **하나의 쉬운 표면(facade)으로 묶어 학생/초보자에게 노출**하는 역할을 한다.

실행 사슬은 다음과 같다. Erebus 게임 매니저가 컨트롤러를 `robot0Controller.py`로 복사 실행 → **`run.py`** 가 `sys.path`에 진짜 `src/` 경로를 주입 → **`main.py`** (또는 `student_example.py`)가 실행 예시 중 하나를 호출 → **`rescue_robot.RescueRobot`** facade가 `Executor`(메인 루프+상태머신), `Mapper`(센서→픽셀 지도), `Robot`(하드웨어 추상화), `FinalMatrixCreator`(픽셀 지도→규정 맵 행렬) 4대 핵심을 조립하고 한 줄 API로 감싼다. 나머지 최상위 파일은 보조 인프라다: **`flags.py`** (전역 디버그·기능 토글), **`utilities.py`** (각도/좌표/리스트/이미지 공용 함수 + HSV 필터 튜너), **`map_visualizer.py`** (OpenCV 실시간 지도 창).

---

## 2. 파일별 상세 분석

### 2.1 `run.py` — Erebus 컨트롤러 진입점 (경로 주입)

**목적**
Erebus는 제출된 컨트롤러 파일을 `robot0Controller.py`라는 이름으로 **복사**해서 실행한다. 이 때문에 `__file__` 기반 경로가 원본 `src/` 폴더를 가리키지 못해 `import main`, `from mapping... ` 같은 패키지 import가 깨질 수 있다. `run.py`는 이를 막기 위해 **진짜 `src/` 절대 경로를 `sys.path` 맨 앞에 강제 삽입**한 뒤 `main`을 불러오는 얇은 부트스트랩이다.

**핵심 로직** (`run.py:7`–`15`)
- `src_dir = r'/Users/pysw/Downloads/Guide/src'` — 이 PC의 실제 `src/` 절대 경로를 하드코딩(`run.py:7`).
- `if not os.path.isdir(src_dir): src_dir = os.path.dirname(os.path.abspath(__file__))` — 직접 실행하는 경우를 위한 fallback(`run.py:10`–`11`).
- `sys.path.insert(0, src_dir)` 후 `import main`(`run.py:13`,`15`) — import만으로 `main.py` 최하단의 실행 예시가 즉시 구동된다.

**사용 알고리즘/기능**: 알고리즘 없음. 표준 라이브러리 `sys`, `os`만 사용한 모듈 검색 경로 조작.

**입출력 데이터 흐름**: 입력 없음 → `sys.path` 부수효과 → `main` 모듈 로드(=프로그램 본체 시작).

**주의점(이식성)**: `src_dir` 경로가 하드코딩이므로 **다른 PC로 옮기면 이 줄을 반드시 수정**해야 한다(`run.py:6`의 주석이 명시). 경로가 틀리면 `isdir` 체크가 실패해 fallback이 동작하지만, Erebus 복사 실행 환경에서는 fallback 경로가 복사본 위치(`src/`가 아님)를 가리켜 import가 깨질 수 있다.

---

### 2.2 `main.py` — 실행 예시 4종 (실질적 진입 함수)

**목적**
`RescueRobot` facade를 쓰는 **4가지 사용 패턴을 함수로 제공**하고, 파일 최하단에서 그 중 하나를 호출한다. `student_example.py`와 내용이 사실상 동일한 "출제자 기준" 버전이다.

**핵심 함수 4종**
| 함수 | 패턴 | 핵심 호출 |
|---|---|---|
| `example_autonomous()` (`main.py:7`) | 완전 자율, 한 줄 | `robot.run_autonomous()` |
| `example_autonomous_with_custom_code()` (`main.py:15`) | 자율 + 사용자 코드 | `while robot.is_running(): robot.step()` + 상태 출력 |
| `example_manual_control()` (`main.py:36`) | 수동 제어 | 조건 분기로 `report_victim`/`go_to_start`/`go_to_next_target` 직접 호출 |
| `example_explore_then_return()` (`main.py:63`) | 탐색→귀환 커스텀 루프 | `exploration_complete` 감지 후 `go_to_start`→`finish_mission` |

파일 끝(`main.py:90`)에서 `example_autonomous()`만 활성화, 나머지 3개는 주석 처리(`main.py:91`–`93`). 즉 **기본 동작은 완전 자율 주행**.

**사용 알고리즘/기능**: 자체 알고리즘 없음. 전적으로 `RescueRobot` API에 위임. 패턴은 (1) 단발 호출, (2)~(4) `while robot.is_running():` 메인 루프 + 분기 제어.

**입출력 데이터 흐름**: `from rescue_robot import RescueRobot` → `RescueRobot` 인스턴스 생성 → 선택된 예시 함수가 facade 메서드 반복 호출. 콘솔 `print`로 위치/시간/조난자 상태 출력.

**규정 연관성**: 메인 루프 구조 자체가 규정의 **8분(480초) 제한 시간** 내 탐색·보고·복귀 사이클을 구현. 방법 3·4는 `is_time_almost_up`/`exploration_complete` 기반으로 **탈출 보너스(EB)** — 토큰 보고 후 시작 타일 복귀 — 시나리오를 직접 코딩하는 예시.

**상호작용**: `run.py`가 `import main`으로 구동 → `RescueRobot`을 호출(아래 2.3).

---

### 2.3 `rescue_robot.py` — `RescueRobot` facade (★ 진입점 지도의 핵심)

**목적**
하위 패키지의 복잡한 내부를 모르고도 로봇을 제어할 수 있게 하는 **초보자 전용 facade**. 위치/시간 조회, 이동 명령, 센서 읽기, 조난자 탐지·보고, 지도 전송, 자율 항법을 모두 **단순한 property/메서드 한 겹**으로 감싼다.

#### 2.3.1 4대 핵심 모듈 조립 — `__init__` (`rescue_robot.py:42`–`53`)

```
self._robot   = Robot(time_step=32)
self._mapper  = Mapper(tile_size=0.12,
                       robot_diameter=self._robot.diameter,
                       camera_distance_from_center=self._robot.diameter/2)
self._executor = Executor(self._mapper, self._robot)
self._final_matrix_creator = FinalMatrixCreator(
                       self._mapper.tile_size,
                       self._mapper.pixel_grid.resolution)
```

이 5줄이 **전체 시스템의 조립도**다.
- **`Robot`** — 하드웨어 추상화. `time_step=32`(Webots 기본 32ms 틱)로 모든 센서/모터를 묶는다. `diameter`(로봇 지름)를 후속 모듈에 전달.
- **`Mapper`** — 센서→픽셀 그리드 지도. **타일 크기 0.12m**(규정의 12cm 타일)와 로봇 지름, 카메라가 로봇 중심에서 떨어진 거리(`diameter/2`)를 받아 좌표 보정에 사용.
- **`Executor`** — `Mapper`와 `Robot`을 **둘 다 주입받아** 메인 루프·상태머신·매핑·LoP 감지를 총괄. facade가 위임하는 거의 모든 "두뇌" 동작의 실제 주인.
- **`FinalMatrixCreator`** — `Mapper`의 `tile_size`와 `pixel_grid.resolution`을 받아 **픽셀 그리드를 규정 맵 행렬로 변환**(전송용).

의존 방향: `Robot` → (`Mapper`가 사용) → (`Executor`가 `Mapper`+`Robot` 사용) → (`FinalMatrixCreator`가 `Mapper` 산출물 변환). facade는 이 4개를 멤버로 들고 메서드마다 적절한 객체로 넘겨준다.

#### 2.3.2 카테고리별 API

**위치 정보** (`rescue_robot.py:59`–`93`): `x`, `y`, `direction`(`orientation.degrees`), `location`, `start_location`(`_mapper.start_position`), `distance_to_start`(미등록 시 `inf`). 모두 `_robot.position`/`_mapper`에서 읽기.

**시간** (`:99`–`112`): `elapsed_time`=`_robot.time`, `remaining_time`=`_executor.max_time_in_run(480s) - elapsed`, `is_time_almost_up`=남은 시간 30초 미만.

**이동 명령** (`:118`–`186`): `move_forward/backward/stop/turn_left/turn_right`는 `_robot.move_wheels(L,R)` 직접 호출. `go_to(x,y)`/`go_to_start()`는 `_robot.move_to_coords(...)`(도착 시 True), `face(deg)`는 `_robot.rotate_to_angle(...)`. 회전 부호 규약: 좌회전=`(-speed, speed)`, 우회전=`(speed, -speed)`.

**센서** (`:192`–`220`): `has_obstacle_ahead`=`_robot.point_is_close`(거리센서/라이다), `is_shaky`=`_robot.is_shaky()`(IMU 기반 기울기/충돌), `swamp_nearby`=`_mapper.is_close_to_swamp()`, `get_camera_image(camera)`=center/left/right 카메라의 `image.image`(H×W×4 BGRA numpy).

**조난자 탐지·보고** (`:226`–`270`):
- `victim_visible` — 3개 카메라 이미지를 `_executor.fixture_detector.find_fixtures(img)`에 넣어 하나라도 검출되면 True.
- `victim_letter` — 첫 검출 픽스처를 `classify_fixture(...)`로 분류해 글자 반환(H/S/U/P/F/C/O).
- `already_reported` — `_mapper.has_detected_victim_from_position()`(같은 위치 중복 보고 방지).
- `report_victim(letter=None)` — letter 미지정 시 자동 판별 → `_robot.comunicator.send_victim(raw_position, letter)`로 게임 매니저 전송 + `_mapper.fixture_mapper.map_detected_fixture(...)`로 기록.

**지도** (`:276`–`287`): `is_at_start`(출발점 4cm 이내), `enable_mapping()`/`disable_mapping()`(`_executor.mapping_enabled` 토글).

**미션 종료** (`:293`–`305`):
- `send_final_map()` — `_final_matrix_creator.pixel_grid_to_final_grid(pixel_grid, start_position)`로 규정 행렬 생성 → `_robot.comunicator.send_map(final_matrix)`.
- `finish_mission()` — `send_final_map()` 후 `_robot.comunicator.send_end_of_play()`.

**자율 항법(에이전트)** (`:311`–`336`):
- `go_to_next_target()` — `_executor.agent.update()` → `get_target_position()` → `_robot.move_to_coords(...)`.
- `auto_target` — 에이전트 추천 목표 (x,y).
- `exploration_complete` — `_executor.agent.do_end()`(탐색 종료·귀환 판단).

#### 2.3.3 루프 제어 — 가장 중요한 3개 메서드

**`is_running()` (`rescue_robot.py:342`–`378`)** — `while` 루프 조건이자 **한 프레임 진행 + 전체 센서/상태 업데이트**의 심장.
1. `if not self._robot.do_loop(): return False` — Webots 한 스텝 진행, 시뮬 종료 시 False.
2. `_executor._tick_real_elapsed()` — 실시간 경과 갱신(규정의 실시간 10분 한계 추적).
3. `_robot.update()` — 모든 센서 읽기.
4. `delay_manager.update(time)`, `stuck_detector.update(position, previous_position, wheel_avg_angular_velocity)` — **LoP(20초 정지 자동 복귀) 감지를 위한 끼임 추적**.
5. **수동 루프 보정**(`:369`–`374`): `run_autonomous()`의 `state_init`을 거치지 않는 수동 모드에서, `mapping_enabled`가 꺼져 있으면 `calibrate_position_offsets()`로 시작점 오차를 보정하고 매핑·조난자보고를 켠다. `start_position`이 None이면 `register_start(position)`로 현재 위치를 출발점 등록.
6. `do_mapping()` + `check_map_sending()` — 매 프레임 지도 갱신과 (시간 임박 시) 자동 지도 전송.

**`step()` (`:380`–`390`)** — `_executor.state_machine.run()` 한 번. 자율 두뇌(상태머신)만 1틱 굴린다. `is_running()`(센서·매핑)과 짝지어 쓰면 "자율 주행 + 내 코드" 패턴(방법 2)이 된다.

**`run_autonomous()` (`:392`–`406`)** — `self._executor.run()` 단 한 줄에 위임. 초기화→자세교정→탐색·보고→복귀→최종 지도 전송 전 과정을 Executor 상태머신이 자동 수행. **방법 1(완전 자율)의 실체**.

> **수동 루프 vs 자율 루프의 분기점**: `run_autonomous()`는 Executor 내부 루프가 `state_init`부터 돌지만, 방법 2~4의 `while robot.is_running()` 수동 루프는 `state_init`을 건너뛰므로 `is_running()` 안의 보정 코드(`:369`–`374`)가 그 초기화를 대신한다. 이 차이를 모르면 수동 모드에서 매핑/출발점이 비는 버그가 생긴다.

**사용 알고리즘/기능**: 자체 알고리즘은 없고 facade(위임) 패턴. 실제 알고리즘(A*, 상태머신, HSV 필터, 그리드 매핑)은 모두 주입된 4대 모듈 안에 있음. numpy(카메라 이미지), Webots 컨트롤러 API는 `_robot` 경유 간접 사용.

**규정 연관성**:
- 조난자 보고(`send_victim`) → 토큰 식별 TI/종류 TT 점수, `already_reported`로 **오식별/중복(TMI -5)** 방지.
- `is_time_almost_up`/`exploration_complete`/`go_to_start` → **8분 제한·탈출 보너스(EB)**.
- `send_final_map`(`FinalMatrixCreator`) → **매핑 보너스(MB)** — 규정 맵 행렬 형식 전송.
- `stuck_detector` → **LoP(20초 정지 자동 복귀)**.
- `swamp_nearby`/`is_shaky`/GPS 노이즈 대응 → 늪·경사·v26 GPS 노이즈 강건성.

**상호작용**: `main.py`/`student_example.py`가 호출. 내부적으로 `Executor`·`Mapper`·`Robot`·`FinalMatrixCreator`와 그 하위(`fixture_detector`, `comunicator`, `fixture_mapper`, `agent`, `state_machine`, `stuck_detector`, `delay_manager`)를 호출.

---

### 2.4 `flags.py` — 전역 디버그·기능 토글

**목적**
프로그램 전체의 디버그 시각화·로깅·저장·실시간 지도를 켜고 끄는 **전역 상수 모음**(0=off, 1=on).

**핵심 플래그** (`flags.py`)
- `SHOW_FIXTURE_DEBUG`(`:7`), `SHOW_DEBUG`(`:8`) — 조난자 탐지 시각화/일반 로그.
- `SHOW_GRANULAR_NAVIGATION_GRID`(`:10`), `SHOW_PATHFINDING_DEBUG`(`:11`), `SHOW_BEST_POSITION_FINDER_DEBUG`(`:12`) — **A\* 경로/탐색 디버그**.
- `SHOW_MAP_AT_END`(`:14`) — 최종 행렬 콘솔 출력.
- `DO_WAIT_KEY`(`:16`) — OpenCV 프레임 키 대기(슬로우모션).
- `DO_SLOW_DOWN`/`SLOW_DOWN_S=0.032`(`:18`–`19`) — 메인 루프 인위 지연.
- `TUNE_FILTER`(`:21`) — HSV 트랙바 튜너 활성화(→ `utilities.ColorFilterTuner`).
- `DO_SAVE_FIXTURE_DEBUG`/`DO_SAVE_FINAL_MAP`/`DO_SAVE_DEBUG_GRID` + 각 저장 경로(`:23`–`30`) — 디버그/지도 이미지 파일 저장.
- **`SHOW_LIVE_MAP = 1`**(`:48`) — `map_visualizer.MapVisualizer` 실시간 OpenCV 지도 창 활성화(기본 켜짐). 주석(`:35`–`47`)에 색상 범례 명시.

**사용 알고리즘/기능**: 없음(상수 정의). 다른 모듈이 `from flags import SHOW_...` 형태로 참조.

**규정 연관성**: 직접 구현 아님. 단, 저장 경로 기본값이 `/home/iitaadmin/...`(`:24`,`:27`,`:30`) 리눅스 절대경로라 **현 macOS 환경에선 저장 플래그를 켜면 실패** — 경로 수정 필요.

**상호작용**: 거의 모든 디버그 분기에서 import. `map_visualizer`/`fixture_detection`/`pathfinding`/메인 루프가 소비.

---

### 2.5 `utilities.py` — 공용 수학·이미지 유틸 + HSV 필터 튜너

**목적**
각도/좌표 변환, 리스트 원소 연산, 디버그 이미지 그리기, 격자 분할, 이미지 리사이즈 등 **여러 모듈이 공유하는 순수 함수 모음** + HSV 색상 필터 튜닝 도구.

**각도/좌표 함수**
- `normalizeRads(rad)`(`:17`) — 라디안 0~2π 정규화. **주의**: 음수 보정이 `rad += 2 + math.pi`(`:21`)로 되어 있어 `2*math.pi`가 의도였다면 **버그 가능성**(상수 `2 + π ≈ 5.14` vs `2π ≈ 6.28`).
- `degsToRads`/`radsToDegs`(`:25`,`:29`) — 도↔라디안.
- `mapVals(val,in_min,in_max,out_min,out_max)`(`:33`) — 선형 범위 매핑(센서값→모터출력).
- `getCoordsFromRads`/`getCoordsFromDegs`(`:38`,`:44`) — 각도·거리→(x,y). **규약: `y=distance*cos`, `x=distance*sin`**(앞=cos, 측면=sin). 라이다 포인트→좌표 변환 등에 쓰이는 핵심 좌표 규약.

**리스트 원소 연산** (`:51`–`77`): `multiplyLists/sumLists/substractLists/divideLists` — zip 기반 원소별 산술(벡터 연산 대용).

**이미지/그리드 시각화 (cv2 + numpy)**
- `save_image`(`:12`) — `images/`에 cv.imwrite.
- `draw_grid`(`:80`) — 픽셀 단위 격자선(이중 for, 느림 — 디버그 전용).
- `draw_poses`(`:90`) — 포즈 목록을 이미지에 점 표기. `xx_yy_format` True=(x배열,y배열), False=(N,2) 배열. **numpy 불리언 마스크로 경계 밖 포즈 제거**(`:98`–`104`, `:113`) 후 fancy indexing으로 색/배경픽셀 일괄 대입 → 벡터화로 빠름.
- `draw_squares_where_not_zero`(`:122`) — 0 아닌 값 있는 타일에 `cv.rectangle` 테두리(`np.count_nonzero`로 판정).
- `get_squares`(`:137`) — 이미지를 타일 격자 `[min_x,max_x,min_y,max_y]` 목록으로 분할.
- `resize_image_to_fixed_size`(`:155`) — 비율 유지 리사이즈(`cv.resize`, 축소 시 `INTER_NEAREST`). 디버그 창용.
- `divide_into_chunks`(`:187`) — 리스트 n개씩 청크 제너레이터.

**`ColorFilterTuner` 클래스** (`:193`–`228`)
- 목적: OpenCV 트랙바로 **HSV 필터 하한/상한(H,S,V 6개)을 실시간 조정**하는 디버그 도구.
- `__init__(color_filter, activate=False)`(`:199`) — activate 시 `filter_tuner` 창에 6개 트랙바 생성(`cv.createTrackbar`, 범위 0~255).
- `tune(image)`(`:217`) — 트랙바 현재값을 읽어 `ColorFilter((min_h,min_s,min_v),(max_h,max_s,max_v))` 재구성, `filter.filter(image)` 마스크를 `imshow`. 콘솔에 lower/upper 출력.

**사용 알고리즘/기능**: HSV 색공간 필터(`fixture_detection.color_filter.ColorFilter` 사용), numpy 불리언 마스킹·fancy indexing, OpenCV 그리기/리사이즈/트랙바, 삼각함수 좌표 변환.

**규정 연관성**: 좌표 변환·격자 분할은 12cm 타일 그리드/맵 행렬 구성의 저수준 빌딩블록. HSV 튜너는 **조난자 글자/인지표적(해즈맷) 색 인식** 정밀화에 사용(`flags.TUNE_FILTER`와 연동).

**상호작용**: `import cv2`, `numpy`, `fixture_detection.color_filter.ColorFilter`. 매핑/픽스처탐지/디버그 모듈이 이 함수들을 호출. `ColorFilterTuner`는 `flags.TUNE_FILTER`가 켜질 때 사용.

---

### 2.6 `map_visualizer.py` — OpenCV 실시간 지도 시각화

**목적**
`Mapper`의 픽셀 그리드 레이어들을 **컬러로 합성해 실시간 OpenCV 창**(`Live Map — 탐색 현황`)에 보여주는 모니터링 도구. `flags.SHOW_LIVE_MAP=1`일 때 활성.

**색상 정의** `_COLORS`(`map_visualizer.py:26`–`40`) — BGR. 벽(흰), traversed(파랑), path(시안), candidate(녹), victim(주황), swamp(갈), hole(빨), robot(빨), target(노), start(마젠타) 등. 표시 크기 `_DISPLAY_SIZE=600`(`:42`).

**`MapVisualizer` 클래스**
- `__init__(mapper)`(`:56`) — Mapper 참조 보관, `_render_every=6`(**6프레임마다 1회 렌더** — 성능 최적화).
- `set_path(path)`(`:64`)/`set_target(target_array_index)`(`:68`) — A* 경로(grid index 목록)·목표 설정.
- `update(path=None, target=None)`(`:72`) — 매 프레임 호출. 프레임 카운터로 6프레임당 1회만 `_render()`→`cv.resize(...,INTER_NEAREST)`→`cv.imshow`→`cv.waitKey(1)`. `robot_position`이 None이면 스킵.
- **`_render()` (`:98`–`167`) — 레이어 합성 순서**(numpy 불리언 마스크 인덱싱):
  1. 배경: `arrays["discovered"]` 마스크로 탐색 완료/미탐색 밝기 구분(`:105`–`106`).
  2. `arrays["traversed"]` 지나간 경로(`:109`).
  3. `arrays["fixture_distance_margin"]` 탐색 후보(`:112`).
  4. `arrays["swamps"]`/`arrays["holes"]` 특수 지형(`:115`–`116`).
  5. `arrays["occupied"]` 벽/장애물(`:119`).
  6. `arrays["victims"]`를 `np.argwhere`로 찾아 각 위치에 `cv.circle`(주황+흰 테두리)(`:122`–`125`).
  7. A* 경로: 노드를 `grid.grid_index_to_array_index`로 변환해 `cv.line` 연결 + 노드 점(`:128`–`137`).
  8. 출발점: `coordinates_to_array_index`로 변환 후 마젠타 `cv.drawMarker(MARKER_STAR)`(`:140`–`144`).
  9. 목표: 노란 `cv.circle`(`:147`–`150`).
  10. 로봇: 빨간 원 + `robot_orientation.degrees`→라디안, **`sin`/`cos`로 방향 화살표 tip 계산**(`:157`–`165`, x=sin·len, y=cos·len — utilities와 동일 좌표 규약) → `cv.arrowedLine`.
- `close()`(`:169`) — `cv.destroyWindow`.

**사용 알고리즘/기능**: numpy 불리언 마스크 fancy indexing(레이어 즉시 합성), OpenCV 그리기(circle/line/rectangle/drawMarker/arrowedLine)·resize·imshow, 좌표↔그리드 인덱스 변환(`grid.coordinates_to_array_index`, `grid_index_to_array_index`), 프레임 스키핑.

**규정 연관성**: 직접 점수 구현 아님(시각화). 단, 표시 레이어가 규정 요소와 1:1 대응 — 벽/구멍/늪/체크포인트성 후보/조난자/시작타일/매핑 진행도. 디버깅 시 **맵 행렬(MB)·조난자 보고 위치 검증**에 유용.

**상호작용**: `from mapping.mapper import Mapper`. `Mapper.pixel_grid`(arrays, array_shape, resolution, 인덱스 변환)·`robot_position`·`robot_orientation`·`start_position` 읽기. 메인 루프/Executor가 `update(path, target)`로 A* 경로·목표를 주입.

---

### 2.7 `student_example.py` — 학생용 예제 진입점

**목적**
학생이 직접 수정해 자기 로봇 프로그램을 만드는 **출발 템플릿**. 내용·구조는 `main.py`와 사실상 동일한 4종 예시(`example_autonomous`/`_with_custom_code`/`_manual_control`/`_explore_then_return`)이며, 기본은 `example_autonomous()`만 활성(`student_example.py:102`).

**`main.py`와의 차이**: 본질적 코드 차이 없음(상단 docstring이 더 친절하게 "이 파일을 수정하라"고 안내, `:1`–`11`). 둘 다 `from rescue_robot import RescueRobot`만 의존하고 facade API만 사용.

**사용 알고리즘/기능**: 없음. 전적으로 `RescueRobot` facade에 위임.

**규정 연관성**: `main.py`와 동일(8분 루프·조난자 보고·복귀·종료).

**상호작용**: `RescueRobot` 호출. (참고: `run.py`는 `import main`을 하므로 실제 대회 실행 경로는 `main.py`. `student_example.py`는 학생이 직접 실행하거나 `run.py`의 import 대상을 바꿀 때 진입점이 됨.)

---

## 3. 모듈 간 상호작용 요약 (진입점 지도)

```
[Erebus 게임매니저]
      │  (robot0Controller.py로 복사 실행)
      ▼
run.py ──(sys.path에 src/ 주입)──▶ import main
      │
      ▼
main.py / student_example.py ──▶ RescueRobot()  (facade)
      │
      ├── Robot(time_step=32)              ── 하드웨어: 카메라/라이다/GPS/IMU/모터/comunicator/drive_base
      ├── Mapper(tile=0.12, …)             ── 센서→픽셀 그리드 지도, start_position, fixture_mapper
      ├── Executor(mapper, robot)          ── 메인 루프·상태머신·agent·fixture_detector·stuck_detector·delay_manager
      └── FinalMatrixCreator(tile, res)    ── 픽셀 그리드→규정 맵 행렬

  보조 인프라:
    flags.py          ── 전역 토글 (SHOW_LIVE_MAP=1 등)  → 모든 디버그 분기가 참조
    utilities.py      ── 각도/좌표/리스트/이미지 공용 함수 + ColorFilterTuner  → 매핑/픽스처/디버그가 사용
    map_visualizer.py ── Mapper.pixel_grid를 OpenCV 창으로 실시간 표시  ← Executor가 update() 호출
```

**호출 관계 한눈에**
- `run.py` → `main`(import).
- `main`/`student_example` → `RescueRobot`.
- `RescueRobot` → `Executor`/`Mapper`/`Robot`/`FinalMatrixCreator` 및 하위(`agent`, `state_machine`, `fixture_detector`, `comunicator`, `fixture_mapper`, `stuck_detector`, `delay_manager`).
- `MapVisualizer` ← `Mapper` 읽기 + 메인 루프가 경로/목표 주입.
- `flags`/`utilities` ← 거의 모든 모듈이 import.

**규정 매핑 총괄**
| 규정 항목 | 구현 위치(최상위 관점) |
|---|---|
| 조난자 토큰 식별/종류(TI/TT) | `RescueRobot.report_victim`→`comunicator.send_victim`, `fixture_detector` |
| 오식별/중복 방지(TMI) | `already_reported`(`mapper.has_detected_victim_from_position`) |
| 맵 행렬·매핑 보너스(MB) | `send_final_map`→`FinalMatrixCreator.pixel_grid_to_final_grid`→`comunicator.send_map` |
| 8분 제한·탈출 보너스(EB) | `remaining_time`/`is_time_almost_up`/`go_to_start`/`finish_mission`, `Executor.max_time_in_run=480` |
| LoP(20초 정지 복귀) | `is_running()` 내 `stuck_detector.update(...)` |
| 늪·경사·GPS 노이즈 강건성 | `swamp_nearby`(`is_close_to_swamp`), `is_shaky`(IMU) |
| 12cm 타일 그리드 | `Mapper(tile_size=0.12)`, `utilities`의 좌표·격자 함수 |

---

## 4. 발견한 주의·잠재 이슈

1. **`run.py:7` 경로 하드코딩** — `src_dir`가 이 PC 절대경로. 다른 PC/제출 환경 이식 시 반드시 수정. Erebus 복사 실행 시 fallback이 복사본 위치를 가리켜 import가 깨질 위험.
2. **`utilities.normalizeRads` (`:21`)** — 음수 보정이 `rad += 2 + math.pi`. `2*math.pi`가 의도였다면 각도 래핑 버그. (실제 각도 정규화는 `data_structures/angle.py`를 별도로 쓰는지 확인 권장 — 이 함수 호출처 점검 필요.)
3. **`flags.py` 저장 경로**(`:24`,`:27`,`:30`) — 리눅스 `/home/iitaadmin/...` 고정. macOS에서 `DO_SAVE_*` 플래그를 켜면 저장 실패.
4. **수동 루프 초기화 의존성** — 방법 2~4(`while robot.is_running()`)는 `state_init`을 건너뛰므로 `is_running()` 내부 보정(`rescue_robot.py:369`–`374`)에 전적으로 의존. 이 블록이 매핑·출발점 등록·자세 보정을 모두 떠안고 있어, 수동 모드 동작의 숨은 핵심.
