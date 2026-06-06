# `executor` 모듈 상세 분석

RoboCupJunior Rescue Simulation 2026 / Webots·Erebus 자율 로봇 코드

---

## 1. 폴더/모듈 개요

`executor`는 로봇의 **최상위 두뇌(top-level brain)**다. 시스템 아키텍처상 `RescueRobot` facade가 조립하는 4대 핵심(Executor / Mapper / Robot / FinalMatrixCreator) 중 **전체 미션 흐름을 총괄하는 컨트롤 타워**에 해당한다. 매 시뮬레이션 타임스텝마다 돌아가는 메인 무한 루프(`Executor.run`)를 보유하며, 이 루프 안에서 (1) 하드웨어 갱신 → (2) 유틸리티(지연·끼임 감지) 갱신 → (3) 매핑 → (4) 종료시간 체크 → (5) **상태머신 실행** 순서를 매 프레임 반복한다.

`executor`가 직접 길을 찾거나 지도를 그리지는 않는다. 대신 `Agent`(다음 목표 좌표 계산), `Mapper`(센서→지도), `Robot`(모터·센서 추상화), `FinalMatrixCreator`(규정 맵 행렬 생성), `FixtureClasiffier`(비전 토큰 인식)를 **조율(orchestrate)** 한다. 핵심 설계는 `init→explore→report_fixture/send_map/stuck→end`로 이어지는 **상태머신(state machine)** 이며, 연속 동작(회전→대기→전진 등)은 `Sequencer`로 비동기 실행한다. 폴더는 단 두 파일(`executor.py`, `stuck_detector.py`)로 구성된다.

---

## 2. 파일별 상세 분석

### 2.1 `executor.py`

#### 목적
미션 전체 생명주기를 상태머신으로 구동하는 메인 컨트롤러. 초기 캘리브레이션, 미로 탐색, 조난자(fixture) 보고, 시간 임박 시 강제 지도 전송, 끼임 탈출, 최종 지도 제출 및 게임 종료까지 모든 고수준 의사결정을 담당한다.

#### 핵심 클래스/함수와 시그니처

- `class Executor` (`executor.py:33`)
- `__init__(self, mapper: Mapper, robot: Robot)` (`executor.py:36`) — 하위 모듈(Agent, Mapper, Robot, DelayManager, StuckDetector, Sequencer, FixtureClasiffier, FinalMatrixCreator) 조립 + 상태머신·시퀀서 이벤트 래핑 등록.
- `run(self)` (`executor.py:93`) — 메인 루프. `while self.robot.do_loop():` 동안 매 프레임 6단계 실행.
- `do_mapping(self)` (`executor.py:126`) — 매핑 활성화 시 지도 갱신.
- `check_swamp_proximity(self)` (`executor.py:145`) — 늪 근접 시 방향 센서를 자이로 전용으로 전환.
- `_tick_real_elapsed(self)` (`executor.py:153`) — 일시정지 제외 실제 경과 시간 누적.
- `check_map_sending(self)` (`executor.py:163`) — 종료 2초 전 `send_map` 강제 전환.
- 상태 함수 6종: `state_init`(`:180`), `state_explore`(`:210`), `state_end`(`:302`), `state_send_map`(`:308`), `state_report_fixture`(`:316`), `state_stuck`(`:399`). 모두 `(self, change_state_function)` 시그니처.
- 보조: `mini_calibrate`(`:263`), `align_with_fixture`(`:270`), `calibrate_position_offsets`(`:416`), `seq_calibrate_robot_rotation`(`:420`).

#### 상태머신 구조 (init → explore → ... → end)

상태머신은 `StateMachine("init")`로 초기 상태 `init`에서 출발한다. `create_state(이름, 함수, {허용 전환 집합})` 형식으로 6개 상태와 전환 규칙이 등록된다(`executor.py:49-54`):

| 상태 | 함수 | 허용된 다음 상태 |
|------|------|------------------|
| `init` | `state_init` | `{explore}` |
| `explore` | `state_explore` | `{end, report_fixture, send_map, stuck}` |
| `report_fixture` | `state_report_fixture` | `{explore, send_map}` |
| `send_map` | `state_send_map` | `{explore, end}` |
| `stuck` | `state_stuck` | `{explore, send_map, end}` |
| `end` | `state_end` | (없음, 종료) |

**상태 전환 조건 정리:**

- **init → explore**: 초기화 시퀀스(영점 보정, 시작점 등록, 자이로 캘리브레이션, 90°·180° 회전 360° 스캔) 완료 후 `change_state("explore")`(`executor.py:204-206`).
- **explore → report_fixture**: 카메라 이미지에서 fixture를 발견하고 `classify_fixture`가 유효 글자를 반환할 때(`executor.py:243-258`).
- **explore → end**: `self.agent.do_end()`가 True일 때(탐색 완료, `executor.py:228-229`).
- **explore → send_map**: 직접 코드는 없고, 메인 루프의 `check_map_sending()`이 시간 임박 시 어떤 상태에서든 강제 전환(`executor.py:170-174`).
- **report_fixture → explore**: 보고 완료, 또는 재확인 실패(오탐)/글자 인식 실패 시 복귀(`executor.py:348/356/396`).
- **send_map → explore**: 지도 전송 후 항상 탐색 재개(`executor.py:313`).
- **stuck → explore**: 탈출 동작 후 복귀(`executor.py:413`).

#### 각 상태 동작 심층 분석

**`state_init` (`:180`)** — 시퀀서 기반 초기화:
0.5초 대기 → `calibrate_position_offsets`(시작점 좌표 오차 계산, 후술) → `mapper.register_start`(시작 타일 등록) → `seq_calibrate_robot_rotation`(GPS로 자이로 영점) → `mapping_enabled`/`victim_reporting_enabled` = True → 제자리 90°·180° 좌회전으로 초기 360° 스캔 → `explore` 전환.

**`state_explore` (`:210`)** — 메인 탐색 루프:
1. `agent.update()`로 다음 목표 좌표 재계산 → `mini_calibrate()`(20스텝마다 미세 자세 교정) → `seq_move_to_coords(agent.get_target_position())`로 이동.
2. `agent.do_end()` True면 `end` 전환.
3. 조난자 탐지: `fixture_detection_cooldown > 0`이면 카운트다운만 하고 감지 스킵(오탐 직후 억제). 쿨다운이 0이고 `victim_reporting_enabled`이며 아직 현 위치에서 victim 미탐지면, 각 카메라 이미지에 `find_fixtures` → `classify_fixture`. 유효 글자면 `letter_to_report`/`report_orientation` 저장 후 `report_fixture` 전환. `classify_fixture`가 `None`(이미 처리됨)이면 다음 카메라로 continue.

**`state_report_fixture` (`:316`)** — 토큰 보고(규정 핵심):
정지 → `mapping_enabled=False`(보고 중 지도 정지) → `report_orientation` 방향으로 회전 → `align_with_fixture`로 카메라 중앙 정렬 → 재확인용 `find_fixtures`. 재확인 실패 시 해당 위치를 fixture_mapper에 블랙리스트 등록하고 `fixture_detection_cooldown = 150`프레임(≈4.8초) 설정 후 explore 복귀(오탐 방지). 성공 시 0.6 속도로 0.1초 전진해 접근 → 정지 → **1.5초 대기**(규정의 "토큰 앞 최소 1초 정지" 충족) → `comunicator.send_victim(raw_position, letter)`로 보고. 글자가 `F/P/C/O`(해즈맷)면 픽셀 그리드 `hazmats` 배열에 `skimage.draw.disk`로 반경 4 원형 마킹. 이후 0.1초 대기 → fixture_mapper 마킹 → 후진 → `mapping_enabled=True` 재개 → explore 복귀.

**`state_send_map` (`:308`)** — 시간 임박 강제 제출:
`final_matrix_creator.pixel_grid_to_final_grid()`로 규정 맵 행렬 생성 → `comunicator.send_map()` → `map_sent=True` → explore 복귀(계속 탐색).

**`state_end` (`:302`)** — 정상 종료:
최종 맵 행렬 전송 → `comunicator.send_end_of_play()`로 게임 종료 신호.

**`state_stuck` (`:399`)** — 끼임 탈출:
후진(-0.6,-0.6) 0.2초 → 우측 바퀴 역회전(0.6,-0.6)으로 제자리 턴 1초 → explore 복귀.

#### do_mapping / check_map_sending / 위치 보정 로직

- **`do_mapping` (`:126`)**: `mapping_enabled`이 True일 때만 동작. `robot.is_shaky()`로 흔들림 판정 — 안정적이면 LiDAR 포인트클라우드 + 카메라 + 위치/방향 전부로 `mapper.update`, 흔들릴 때는 LiDAR를 빼고 카메라·위치만 대략 갱신(노이즈로 인한 지도 오염 방지). 규정의 GPS 노이즈/노이즈 강건성 요구와 직결.
- **`check_map_sending` (`:163`)**: `sim_elapsed`(시뮬 시간)와 `__real_elapsed`(일시정지 제외 누적 리얼타임) 중 **큰 값**을 기준으로 `max_time_in_run - 2`(478초) 초과 시 `send_map` 강제 전환. `_tick_real_elapsed`(`:153`)는 한 스텝 delta가 1초 초과면 일시정지로 간주해 누적에서 제외 → Webots Pause 중 오작동 방지. 규정 8분(480초) 제한 대비 안전 마진.
- **위치 보정**:
  - `calibrate_position_offsets`(`:416`): `robot.position % (quarter_tile_size * 2)`로 시작 위치가 타일 중앙이 아닐 때의 격자 오프셋을 계산해 보정.
  - `seq_calibrate_robot_rotation`(`:420`): 전방에 블랙홀(구멍) 가능성이 있어 **후진 5cm**(-1,-1, 0.4초)로 안전하게 이동, GPS heading 취득 후 `+π`로 실제 전방 heading 산출, 자이로에 세팅. 이후 전진 복귀. GPS baseline을 두 번 리셋해 복귀 이동이 heading 계산을 오염시키지 않게 함.
  - `mini_calibrate`(`:263`): `StepCounter(20)` — 20스텝마다 잠깐 직진(1,1)+0.1초 대기로 자세 미세 교정. 데드레커닝 누적 오차 억제.
  - `check_swamp_proximity`(`:145`): 늪 근접 시 GPS 오차가 커지므로 `orientation_sensor`를 GYROSCOPE 전용으로 강제(단, 메인 루프에서 호출되지는 않음 — 아래 발견사항 참조).

#### align_with_fixture 비전 제어 (`:270`)
카메라 중앙 픽셀과 검출된 fixture 중심 픽셀의 차이 `diff` 계산. `|diff.y| > 4`면 전후진(`vel = diff.y*0.1`), `|diff.x| > 6`면 제자리 회전(부호에 따라 좌우 바퀴 반대). 둘 다 임계 이하면 정렬 완료(True). 규정의 "로봇 중심이 토큰에서 반 타일 이내" 정밀 정렬을 달성하는 비례(P) 제어.

#### 사용 알고리즘
- **유한 상태머신(FSM)**: `StateMachine` + 허용 전환 집합으로 미션 단계 제어.
- **시퀀서 기반 비동기 동작 스크립팅**: `Sequencer`의 simple/complex event로 "동작→대기→동작" 블로킹 없이 프레임 분할 실행.
- **비례 제어(P control)**: `align_with_fixture`의 픽셀 오차 기반 속도 결정.
- **모듈러 격자 정렬**: `position % (quarter_tile_size*2)`로 오프셋 산출.
- **GPS heading + π 후진 캘리브레이션**: 블랙홀 회피형 자이로 영점.
- **원형 디스크 래스터화**: `skimage.draw.disk`로 해즈맷 영역 마킹.
- **시간 중 최댓값 듀얼클록**: 시뮬/리얼 경과 중 큰 값으로 타임아웃 판정.

#### 사용 기능/라이브러리
- `numpy`(np.sign, np.pi 등), `cv2`(import만 — 직접 사용은 fixture_detector 내부), `skimage.draw.disk`(해즈맷 디스크 마킹), `time`(리얼타임 누적·디버그 슬로다운).
- 내부: `data_structures`(Angle, Position2D), `flow_control`(Sequencer, StateMachine, DelayManager, StepCounter), `robot`(Robot, RotationCriteria), `mapping.mapper`, `agent`, `fixture_detection`(FixtureClasiffier, ColorFilter, NonFixtureFilter), `final_matrix_creation`, `flags`(DO_SLOW_DOWN, SLOW_DOWN_S, TUNE_FILTER).

#### 입력/출력 데이터 흐름
- **입력**: `robot`의 센서값(GPS 위치, 방향, LiDAR 포인트클라우드, 카메라 이미지, 바퀴 각속도), `agent`의 목표 좌표, `mapper`의 상태(늪 근접, victim 탐지 여부, 시뮬 시간).
- **출력**: `robot.move_wheels`/`move_to_coords`/`rotate_to_angle`(모터 명령), `mapper.update`(지도 갱신), `comunicator.send_victim`/`send_map`/`send_end_of_play`(Erebus 게임매니저 통신).

---

### 2.2 `stuck_detector.py`

#### 목적
"바퀴는 돌고 있는데 로봇이 실제로 안 움직이는" 상태(벽 끼임)를 감지. 규정의 **LoP(Lack of Progress, 20초 정지)** 페널티를 피하기 위해, 페널티가 발동되기 전에 자체적으로 끼임을 인지하고 탈출 동작을 유발하려는 목적의 모듈이다.

#### 핵심 클래스/함수와 시그니처
- `class StuckDetector` (`stuck_detector.py:3`)
- `__init__(self)` (`:5`) — `stuck_counter=0`, `stuck_threshold=50`, `minimum_distance_traveled=0.001`.
- `update(self, position, previous_position, wheel_direction)` (`:15`) — 매 스텝 호출, 끼임이면 카운터++, 아니면 0으로 리셋.
- `is_stuck(self)` (`:27`) — `stuck_counter > stuck_threshold`(50) 반환.
- `__is_stuck_this_step(self)` (`:31`) — private 판정.

#### 사용 알고리즘
**누적 카운터 + 임계값(debounce) 기반 끼임 감지.** 한 스텝의 끼임 판정은 두 조건의 AND:
1. `is_rotating_wheels = wheel_direction > 0` — 바퀴 평균 각속도가 양수(움직이려 함).
2. `distance_traveled = position.get_distance_to(previous_position) < 0.001` — 실제 이동 거리가 1mm 미만(유클리드 거리).

둘 다 참인 스텝이 연속 **50스텝 초과** 누적되면 `is_stuck()` True. 한 스텝이라도 정상 이동하면 카운터 0 리셋(노이즈로 인한 오탐 방지 디바운싱).

#### 사용 기능/라이브러리
- `data_structures.vectors.Position2D`(`get_distance_to`로 두 위치 간 유클리드 거리). 외부 라이브러리 없음.

#### 입력/출력 데이터 흐름
- **입력**: `Executor.run`의 매 프레임 `robot.position`, `robot.previous_position`, `robot.drive_base.get_wheel_average_angular_velocity()`(`executor.py:105-107`).
- **출력**: `is_stuck()` 불리언.

---

## 3. 규정 연관성 (RoboCupJunior Rescue Sim 2026)

- **토큰 식별(TI/TT) + "앞 최소 1초 정지"**: `state_report_fixture`의 1.5초 대기 후 `send_victim`이 직접 구현(`executor.py:362-370`). `align_with_fixture`가 "로봇 중심 반 타일 이내" 정밀 위치를 보장.
- **오식별(TMI, -5) 방지**: 보고 직전 재확인(`find_fixtures` 재호출)과 실패 시 블랙리스트+150프레임 쿨다운(`executor.py:342-351`)으로 같은 오탐 반복 억제.
- **해즈맷(Cognitive target F/P/C/O)**: 보고 시 픽셀 그리드 `hazmats` 배열에 원형 마킹(`executor.py:371-376`).
- **매핑 보너스(MB)**: `state_end`/`state_send_map`에서 `pixel_grid_to_final_grid`로 규정 맵 행렬을 만들어 `send_map`. 시작 타일 기준 정렬을 위해 `mapper.start_position` 전달.
- **시간(8분/480초)**: `max_time_in_run = 8*60`, `check_map_sending`이 478초에 강제 제출 → 시간 초과로 맵 미제출되는 사고 방지.
- **LoP(20초 정지, -5)**: `StuckDetector`가 끼임 감지 목적. `state_stuck`이 탈출(후진+제자리 턴) 동작 보유.
- **노이즈 강건성(v26 GPS 노이즈)**: `do_mapping`의 `is_shaky` 분기, `check_swamp_proximity`의 자이로 전환, `mini_calibrate`/`seq_calibrate_robot_rotation` 위치·방향 보정이 GPS 노이즈 대응.
- **구멍(hole/블랙홀) 회피**: 초기 캘리브레이션을 후진으로 수행해 전방 구멍 추락 방지(`executor.py:420-449`).

---

## 4. 다른 모듈과의 상호작용

**Executor가 호출하는 것 (downstream):**
- `Agent`: `update()`, `get_target_position()`, `do_end()` — 다음 목표 좌표와 탐색 완료 판정.
- `Mapper`: `update()`, `register_start()`, `is_close_to_swamp()`, `has_detected_victim_from_position()`, `fixture_mapper.map_detected_fixture()`, `pixel_grid`(coordinates_to_array_index, arrays["hazmats"]).
- `Robot`: `do_loop()`, `update()`, `move_wheels`/`move_to_coords`/`rotate_to_angle`/`rotate_slowly_to_angle`, `get_point_cloud`/`get_lidar_detections`/`get_camera_images`, `is_shaky()`, `position`/`raw_position`/`orientation`, `gps`/`gyroscope`(캘리브레이션), `drive_base.get_wheel_average_angular_velocity()`, `comunicator.send_victim`/`send_map`/`send_end_of_play`.
- `FixtureClasiffier`: `find_fixtures()`, `classify_fixture()`.
- `FinalMatrixCreator`: `pixel_grid_to_final_grid()`.
- `flow_control`: `Sequencer`(동작 스크립팅), `StateMachine`(상태 전이), `DelayManager`, `StepCounter`.
- `StuckDetector`: `update()`.

**Executor를 호출하는 것 (upstream):**
- `RescueRobot` facade가 Executor를 조립하고, 최종적으로 `main.py`→`run.py` 흐름에서 `Executor.run()`이 진입점으로 실행된다(아키텍처 문서 기준).

---

## 5. 주목할 발견사항 (코드 정합성)

1. **`StuckDetector.is_stuck()`가 실제로 호출되지 않는다 (잠재적 데드코드/버그).** 메인 루프에서 `stuck_detector.update(...)`는 매 프레임 호출되지만(`executor.py:105-107`), 전체 `src/`를 grep한 결과 `is_stuck()`를 읽어 `change_state("stuck")`을 부르는 코드가 **어디에도 없다**. 즉 `state_stuck`과 explore→stuck 전환이 상태머신에 등록(`executor.py:50,54`)되어 있고 탈출 로직도 구현돼 있지만, 트리거가 연결되지 않아 **실전에서 stuck 상태로 진입할 수 없다**. 끼임 감지 → 탈출 파이프라인이 끊긴 상태이므로 LoP 대응이 의도대로 동작하지 않을 가능성이 높다.

2. **`check_swamp_proximity()`도 정의만 되어 있고 `run()` 루프에서 호출되지 않는다(`executor.py:145`).** 늪지대 자이로 전환 로직이 활성화되지 않는다.

3. **`state_send_map`은 explore로 복귀하지만 `map_sent=True`가 되어** `check_map_sending`이 재트리거되지 않으므로 1회만 강제 제출된다. 정상 동작이며, 시간이 더 흘러 `agent.do_end()`로 `end` 도달 시 `state_end`가 다시 `send_map`을 보낸다(중복 전송 가능하나 규정상 무해).

---
