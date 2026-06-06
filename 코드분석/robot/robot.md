# `robot` 모듈 상세 분석 — 하드웨어 추상화 계층 (HAL)

> RoboCupJunior Rescue Simulation 2026 — Webots/Erebus 자율 로봇 코드
> 대상 폴더: `src/robot/`

---

## 1. 폴더/모듈 개요

`robot` 모듈은 **하드웨어 추상화 계층(Hardware Abstraction Layer)** 이다. Webots 컨트롤러 API(`controller.Robot`, `Motor`, `Lidar`, `GPS`, `Gyro`, `Camera`, `Emitter`, `Receiver`)를 직접 다루는 유일한 계층으로, 상위 로직(Executor / Agent / Mapper)은 Webots API를 전혀 몰라도 되게 만든다. 상위 계층은 "전방에 장애물이 가까운가?", "이 좌표로 가라", "조난자 코드 'H'를 보내라" 같은 **고수준 명령**만 쓰고, 그 명령이 실제 모터 속도/센서 버퍼/struct 패킷으로 번역되는 과정은 전부 이 폴더 안에 숨어 있다.

구조는 3층이다. (1) 최상위 통합 클래스 `Robot`(`robot.py`)이 모든 디바이스를 생성·연결하고 깔끔한 facade API를 제공한다. (2) 제어/추정 관리자 — `DriveBase`(모터 제어, 좌표 이동, 회전 PID), `PoseManager`(GPS+자이로 융합으로 위치/방향 추정). (3) `devices/` 하위 — 각 Webots 센서/액추에이터를 1:1로 감싸는 얇은 래퍼(`camera`, `lidar`, `gps`, `gyroscope`, `wheel`, `sensor`)와, 게임매니저와 통신하는 `comunicator`. 이 폴더는 규정의 **센서 항목(GPS 노이즈, 거리/LiDAR, RGB 카메라, IMU)** 과 **토큰 보고/맵 제출/exit 프로토콜**을 직접 구현하는 핵심부다.

---

## 2. 최상위 통합 — `robot.py`

### 목적
모든 하드웨어를 생성·소유하고, 매 타임스텝 갱신을 조율하며, Executor에게 단일 진입점 API를 제공한다.

### 핵심 클래스: `Robot`
- `__init__(self, time_step)` — 모든 디바이스를 생성. 주요 상수:
  - `self.diameter = 0.074` (로봇 직경 7.4cm) — `robot.py:41`
  - `self.camera_distance_from_center = 0.0310` (카메라-중심 거리 3.1cm) — `robot.py:60`
  - `max_wheel_speed = 6.28` rad/s — `robot.py:87`
- Webots 장치 바인딩 (`self.robot.getDevice(...)`):
  - `"gps"`, `"gyro"` → `Gps`, `Gyroscope` (`robot.py:46-47`)
  - `"lidar"` → `Lidar`, **6 타임스텝마다** 갱신, 레이어 `(2,)`만 수평 스캔 사용 (`robot.py:53-57`)
  - `"camera1/2/3"` → 전방/우측/좌측 `Camera`, **3 타임스텝마다** 갱신. 좌측은 `rotate180=True` (`robot.py:62-79`)
  - `"emitter"`, `"receiver"` → `Comunicator` (`robot.py:82-84`)
  - `"wheel1 motor"`, `"wheel2 motor"` → `Wheel` × 2 → `DriveBase` (`robot.py:88-91`)

### 핵심 함수 / 시그니처
- `update(self)` (`robot.py:93`) — **갱신 순서가 중요**:
  1. `self.__time = self.robot.getTime()`
  2. `pose_manager.update(평균 각속도, 좌우 속도차)` — 바퀴 속도로 직진/회전을 판단해 GPS↔자이로 선택
  3. `drive_base.orientation/position = 최신 추정값` — DriveBase가 제어에 쓸 위치/방향 주입
  4. `lidar.set_orientation(orientation)` → `lidar.update()` — 포인트 좌표를 전역 방향으로 회전
  5. 카메라 3개 `update(orientation)`
- `do_loop(self)` (`robot.py:119`) — `self.robot.step(time_step) != -1`. 시뮬레이션을 한 스텝 전진, 종료 시 False.
- `set_start_time()` / `time` 프로퍼티 — 미션 경과 시간(초) 계산. 규정의 480초/600초 제한 추적용.
- DriveBase 래퍼: `move_wheels`, `rotate_to_angle`(도 입력→`Angle`로 변환), `rotate_slowly_to_angle`, `move_to_coords`.
- LiDAR 래퍼: `point_is_close`, `get_point_cloud`, `get_out_of_bounds_point_cloud`, `get_lidar_detections`.
- 카메라 래퍼: `get_camera_images()` — 주기 도래 시에만 `[우, 전, 좌]` 3개 반환; `get_last_camera_images()` — 주기 무관 마지막 이미지.
- Pose 래퍼: `position`(오프셋 보정), `raw_position`(GPS 원시, 서버 전송용), `orientation`, `is_shaky()`, 센서 선택 프로퍼티(`auto_decide_orientation_sensor`, `orientation_sensor`, 상수 `GPS`/`GYROSCOPE`).

### 사용 알고리즘
- **컴포지트 + 파사드 패턴**: 디바이스 객체들을 소유하고 단순 API로 위임.
- **데시메이션(decimation)**: `StepCounter`로 LiDAR는 1/6, 카메라는 1/3 주기로만 무거운 처리 수행 → 실시간성 확보.
- **좌표계 정렬**: 카메라/LiDAR의 로봇 기준 방향에 현재 `orientation`을 더해 전역 방향 산출.

### 사용 라이브러리/API
- `controller.Robot`(Webots), `cv2`(import만), 내부 모듈(`Angle`, `Position2D`, `StepCounter`).

### 입출력 흐름
- 입력: Webots 시뮬레이터의 센서 버퍼. 출력: Executor가 호출하는 고수준 데이터/명령.

### 상호작용
- **호출됨**: `RescueRobot` facade가 생성, Executor 메인 루프가 `update()`/`do_loop()` 매 스텝 호출.
- **호출함**: 모든 `devices/*`, `DriveBase`, `PoseManager`.

---

## 3. 모터 제어 / 이동 — `drive_base.py`

### 목적
좌우 두 바퀴를 통합해 (a) 제자리 회전, (b) 목표 좌표로의 부드러운 이동을 수행. 실제 제어는 내부 관리자 3개에 위임한다.

### 핵심 클래스

#### `Criteria(Enum)` (`drive_base.py:10`)
회전 방향 기준: `LEFT`, `RIGHT`, `CLOSEST`(최단), `FARTHEST`(반대).

#### `DriveBase` (`drive_base.py:17`)
- 보유: `rotation_manager`(일반 회전), `slow_rotation_manager`(`max_velocity=0.4`, 조난자 정렬용), `movement_manager`(`SmoothMovementToCoordinatesManager`).
- `move_wheels(left_ratio, right_ratio)` — 바퀴에 비율 직접 전달.
- `rotate_to_angle(angle, criteria)` / `rotate_slowly_to_angle(...)` → bool(완료 여부).
- `move_to_position(position)` → bool.
- `orientation` setter — **세 관리자 모두에 방향 동기화** (`drive_base.py:69-74`).
- `get_wheel_average_angular_velocity()` (`:76`), `get_wheel_velocity_difference()` (`:82`) — PoseManager의 직진/회전 판단 입력.

#### `RotationManager` (`drive_base.py:87`) — **P 제어(비례 제어) 기반 회전**
- `rotate_to_angle(target_angle, criteria)`:
  - 오차 크기 → 속도: `mapVals(절대오차, accuracy(2°), 90°, min_velocity, max_velocity)` (`:130`). 오차 90°→최대속도, 2°→최소속도로 선형 매핑(비례 제어).
  - `velocity_reduction_threshold=10°` 이하면 속도 ×0.5 (목표 근처 감속, 오버슈트 억제).
  - 속도를 `[min_velocity_cap=0.2, max_velocity_cap=1]`로 클램프(최소 하한으로 정지 방지).
  - `accuracy=2°` 이내면 완료 → 양 바퀴 정지.
  - 회전 방향은 좌/우 바퀴 부호 반대로 **제자리 회전**.
- `__get_direction(...)` (`:151`) — 각도 차의 부호와 ±180° 래핑을 따져 최단/최원 방향 계산.

#### `MovementToCoordinatesManager` (`drive_base.py:169`) — **레거시(미사용)**
회전 완료 후 직진하는 단순 방식. 주석상 `Smooth...`로 대체됨.

#### `SmoothMovementToCoordinatesManager` (`drive_base.py:222`) — **현재 사용되는 곡선 이동**
목표까지의 각도 오차에 따라 4단계 모드(`move_to_position`, `:254`):
- 오차 < `angle_error_margin(3°)` → **직진**(양 바퀴 동일 속도).
- 오차 > `strong_rotation_start(45°)` → **제자리 회전**으로 방향 전환.
- 오차 < `light_rotation_start(30°)` → 한쪽 바퀴 0.8배 **약한 곡선**.
- 그 사이 → 각도 오차 비례 동적 속도(`angle_speed = absolute_angle_diff.radians * angle_weight(5)`)로 **곡선 이동**.
- 도달 판정: `error_margin=0.003m`.

### 사용 알고리즘
- **비례(P) 제어** 회전, **각도 오차 임계값 기반 상태 전환식 모션 컨트롤러**, 각도 래핑(±180°) 방향 결정.

### 사용 라이브러리/API
- `utilities.mapVals`(선형 매핑), `data_structures.angle.Angle`, `Position2D/Vector2D`, `math`. Webots API는 직접 쓰지 않고 `Wheel`을 통해 간접 제어.

### 입출력 흐름
- 입력: 목표 각도/좌표 + (Robot이 주입한) 현재 위치/방향. 출력: `Wheel.move()` 호출.

### 상호작용
- **호출됨**: `Robot.update()`(위치/방향 주입), Robot 래퍼 메서드. **호출함**: `Wheel.move()`.

---

## 4. 위치·방향 융합 — `pose_manager.py`

### 목적
GPS와 자이로스코프를 **상황별로 선택·융합**해 신뢰할 수 있는 위치(Position2D)와 방향(Angle)을 추정한다. 규정의 **v26 GPS 노이즈** 대응 핵심.

### 핵심 클래스: `PoseManager` (`pose_manager.py:9`)
- 상수 `GPS=0`, `GYROSCOPE=1`.
- 임계값: `maximum_angular_velocity_for_gps = 1°`, `shaky_threshold = 5°`, `maximum_angular_velocity_change_for_shaky = 1°`.
- `update(average_wheel_velocity, wheel_velocity_difference)` (`:44`):
  1. `gps.update()`, `gyroscope.update()`
  2. 이전/현재 위치 갱신(`__position = gps.get_position()`)
  3. 자동 모드면 `decide_orientation_sensor(...)`
  4. `calculate_orientation()`
- `decide_orientation_sensor(...)` (`:64`): 직진이면 GPS, 아니면 자이로 선택 + **GPS 방향 기준선 리셋**(`gps.reset_orientation_baseline()`) — 기준선이 회전 구간을 가로질러 오방향 계산하는 것 방지.
- `robot_is_going_straight(...)` (`:74`): `각속도<1°` AND `평균 각속도≥1` AND `좌우 속도차<3` 일 때 직진.
- `calculate_orientation()` (`:80`): 자이로 선택 시(또는 GPS가 None이면) 자이로 각도 사용; GPS 선택 시 GPS 각도를 쓰고 **자이로를 그 값으로 교정(`gyroscope.set_orientation`)** → 드리프트 제거.
- 프로퍼티: `position`(오프셋 보정), `raw_position`(원시), `previous_position`.
- `is_shaky()` (`:108`): (방향 변화 > 5°) OR (각속도 부호 반전) OR (각속도 변화 > 1°) → True. 흔들릴 때 Mapper가 LiDAR 갱신을 일부 생략하게 함.

### 사용 알고리즘
- **상보 융합(complementary fusion) / 센서 선택 전략**: 직진=GPS(노이즈 평균화로 정확), 회전·늪=자이로(누적 적분, 순간 정확). GPS로 자이로 드리프트를 주기적 교정하는 **드리프트 보정 루프**.
- **흔들림 감지(shake detection)** 휴리스틱 — 신뢰도 게이팅에 사용.

### 사용 라이브러리/API
- 내부 모듈만(`Gps`, `Gyroscope`, `Angle`, `Position2D`). Webots API는 디바이스 래퍼를 통해 간접 사용.

### 입출력 흐름
- 입력: 바퀴 평균 각속도/속도차(DriveBase), GPS·자이로 센서값. 출력: `position`/`orientation` 추정값 → Robot → Mapper/Agent.

### 상호작용
- **호출됨**: `Robot.update()`, Robot의 pose 래퍼. **호출함**: `Gps`, `Gyroscope`.

---

## 5. `devices/` 하위 디바이스 래퍼

### 5.1 `sensor.py` — 센서 추상 기반 클래스

- **목적**: 모든 센서의 공통 부모. Webots 장치 enable과 갱신 인터페이스 정의.
- `Sensor(ABC)` (`sensor.py:3`): `__init__`에서 `device.enable(time_step)` 호출. `update()`는 no-op.
- `TimedSensor(Sensor)` (`sensor.py:13`): `StepCounter`를 보유해 **n 스텝마다 1회** 처리. `update()`가 `step_counter.increase()`만 수행. → Camera, Lidar가 상속.
- **알고리즘**: 추상 기반 클래스(ABC) + 데시메이션. **API**: `abc.ABC`, Webots `device.enable`.

### 5.2 `wheel.py` — 모터 액추에이터

- **목적**: Webots `Motor`(바퀴) 하나의 속도 제어.
- `Wheel.__init__` (`wheel.py:3`): `wheel.setPosition(float("inf"))`로 **속도 제어 모드** 설정, `setVelocity(0)`.
- `move(ratio)` (`wheel.py:10`): 비율을 `[-1,1]`로 클램프 → `velocity = ratio * maxVelocity`(6.28 rad/s) → `wheel.setVelocity(...)`. 현재 속도를 저장해 PoseManager가 읽음.
- **API**: Webots `Motor.setPosition`, `setVelocity`.

### 5.3 `gps.py` — GPS 센서 (**v26 노이즈 대응 핵심**)

- **목적**: 전역 위치 추적 + 직진 시 진행 방향 추정.
- `Gps(Sensor)` (`gps.py:5`):
  - `get_position()` (`:36`): `device.getValues()` → `Position2D(x, z*mult)`. **Webots z축 → 코드 y축** 변환.
  - **노이즈 대책(`:17-22`)**: v26 GPS에 가우시안 노이즈(σ≈2.5mm) 추가됨. 연속 두 점 차이는 노이즈가 신호를 압도하므로, `min_baseline_distance = 0.04m`(노이즈의 약 16배) 이상 이동한 과거 위치를 기준선으로 사용.
  - `__position_history`(최대 40개) — 직진 구간 위치 이력 누적.
  - `reset_orientation_baseline()` (`:31`): 회전 시 이력 초기화.
  - `get_orientation()` (`:41`): 이력을 역순 탐색해 `min_baseline_distance` 이상 떨어진 가장 최근 점→현재로의 각도 반환. **긴 기준선으로 노이즈 평균화**. 이동 부족 시 `None` → PoseManager가 자이로로 대체.
- **알고리즘**: 위치 이력 기반 **베이스라인 방향 추정(노이즈 평균화)**, 좌표계 변환.
- **API**: Webots `GPS.getValues`.

### 5.4 `gyroscope.py` — IMU(자이로)

- **목적**: 각속도를 적분해 절대 방향 추적.
- `Gyroscope(Sensor)` (`gyroscope.py:6`):
  - `update()` (`:20`): `time_elapsed = time_step/1000`(ms→s), `sensor_y = getValues()[index]`(Y축=수직축 회전), `angular_velocity = Angle(sensor_y * time_elapsed)`(스텝당 회전량), `orientation += angular_velocity` 후 `normalize()`.
  - `get_angular_velocity()` — 절대값(크기). `set_orientation(angle)` — GPS로 영점 교정.
  - `previous_angular_velocity` 보관 → `PoseManager.is_shaky()`의 부호 반전 감지.
- **알고리즘**: **각속도 수치 적분(dead-reckoning 방향)** + 0~2π 래핑.
- **API**: Webots `Gyro.getValues`.

### 5.5 `lidar.py` — LiDAR 거리 센서

- **목적**: 360° 수평 스캔을 2D 포인트 클라우드로 변환. 벽 매핑·열린공간 판별·조난자 위치 추정의 원천.
- `Lidar(TimedSensor)` (`lidar.py:10`):
  - 파라미터: `max_detection_distance = 0.06*8 ≈ 0.48m`, `min_detection_distance = 0.06*0.6 ≈ 0.036m`(근접 노이즈 제거), `is_point_close_threshold = 0.03m`, `distance_bias = 0.005m`, `layers_used=(2,)`(수평 레이어만).
  - `__update_point_clouds()` (`:78`): `device.getRangeImage()` → `divide_into_chunks`로 레이어 분할. 사용 레이어만 처리:
    - 수평 각도 시작점에 **로봇 방향 반영**: `(2π) - orientation.radians` (`:96`).
    - 거리 ≥ max 또는 inf → `out_of_bounds_point_cloud`(열린 공간).
    - min~max → `point_cloud`(벽) + `distance_detections`(Vector2D, 방향 보정 `π - direction`).
    - 전방 근거리 & distance < 0.03 → `is_point_close=True`.
  - `__normalize_distance()` (`:133`): `distance * cos(vertical_angle)`(수직 경사 제거) + `distance_bias`.
  - `__normalize_point()` (`:139`): Webots Y축 반전 보정.
  - 빈 클라우드 방지용 더미 `[[0,0]]` 삽입.
- **알고리즘**: **극좌표→직교좌표 변환**(`getCoordsFromRads`), 레이어 청크 분할, 임계값 기반 in/out-of-bounds 분류, 삼각함수 경사 보정.
- **API**: Webots `Lidar.getRangeImage/getFov/getHorizontalResolution/getNumberOfLayers`.

### 5.6 `camera.py` — RGB 카메라

- **목적**: 전/우/좌 3대 카메라 이미지를 numpy 배열로 캡처. fixture_detection의 입력(글자 조난자/인지 표적 탐지).
- `CameraData`(dataclass, `:13`): 해상도·FOV·로봇기준/전역 방향·중심거리 메타데이터.
- `CameraImage`(`:26`): BGRA numpy 이미지 + 메타데이터 컨테이너.
- `Camera(TimedSensor)` (`:32`):
  - `__init__`: 해상도·`horizontal_fov`(`getFov`), 종횡비로 `vertical_fov` 계산(`2*atan(tan(hfov/2)*h/w)`), 장착 방향, `rotate180`.
  - `update(robot_orientation)` (`:79`): 전역 방향 = 장착 방향 + 로봇 방향. 주기 도래 시 `getImage()` → `np.frombuffer(...).reshape((h,w,4))`. 좌측 카메라는 `np.rot90(...,2)`로 180° 보정.
  - `get_image()`(주기에만), `get_last_image()`(항상), `get_data()`.
- **알고리즘**: RGBA 버퍼→numpy 변환, FOV 기하 계산, 데시메이션.
- **API**: Webots `Camera.getImage/getWidth/getHeight/getFov`, `numpy`, `cv2`(import).

### 5.7 `comunicator.py` — Erebus 게임매니저 통신 (**규정 직결**)

- **목적**: Webots `Emitter`/`Receiver`로 대회 서버와 바이너리 프로토콜 통신. 토큰 보고·맵 제출·종료·점수 조회·LoP 처리.
- `Comunicator(Sensor)` (`comunicator.py:8`):
  - `send_victim(position, victimtype)` (`:31`): 위치 m→cm 변환(`multiplyLists(...,[100,100])`), 글자코드 1바이트 → `struct.pack("i i c", x, y, letter)` → `emitter.send`. **규정의 토큰 식별/종류 전송**(TI/TT). 위치는 cm로 보내 "반 타일 이내" 판정에 사용됨.
  - `send_lack_of_progress()` (`:41`): `struct.pack('c', 'L')` 전송 — 스스로 진행 불능 신고.
  - `send_end_of_play()` (`:48`): `struct.pack('c', b'E')` 전송 — **미션 종료/탈출(exit)**. 탈출 보너스(EB) 트리거.
  - `send_map(np_array)` (`:55`): `struct.pack('2i', *shape)` + `','.join(flatten())`의 UTF-8 바이트로 직렬화 전송 → 별도로 `'M'`(맵 평가 요청) 전송. **규정의 맵 제출/매핑 보너스(MB)**.
  - `request_game_data()` (`:74`): `'G'` 전송.
  - `update()` (`:80`): 수신 큐 처리. `'G'` 패킷(`struct.unpack('c f i')`) → `game_score`, `remaining_time` 갱신. `'L'` 패킷 → `lack_of_progress=True`(규정의 20초 정지 LoP 페널티 수신).
- **알고리즘**: `struct` 바이너리 패킹/언패킹, 단순 큐 폴링 상태 토글(`do_get_world_info`).
- **API**: Webots `Emitter.send`, `Receiver.enable/getQueueLength/getBytes/nextPacket`, `struct`.

---

## 6. 규정(RoboCup Rescue Sim 2026) 연관성

| 규정 항목 | 구현 위치 |
|---|---|
| **GPS 노이즈(v26 추가)** 강건성 | `gps.py` 베이스라인 방향 추정(`min_baseline_distance=0.04m`), `pose_manager.py` GPS↔자이로 선택 |
| **IMU(자이로+가속도)** | `gyroscope.py` 각속도 적분, 회전·늪 구간 방향 추정 |
| **LiDAR(주변 거리)** | `lidar.py` 포인트 클라우드 → 벽/열린공간 분류 |
| **RGB 카메라(글자/표적 탐지)** | `camera.py` 3대 이미지 캡처 → fixture_detection 입력 |
| **거리센서(근접 장애물)** | `lidar.py` `is_point_close`(0.03m) |
| **토큰 식별 TI / 종류 TT, 위치 반 타일 이내** | `comunicator.send_victim`(cm 좌표+글자코드) |
| **체크포인트/늪/구멍 회피** | 직접 구현은 상위(Mapper/floor) 이지만 위치 추정·이동을 본 모듈이 지원 |
| **맵 제출 / 매핑 보너스(MB)** | `comunicator.send_map`('M' 평가 요청) |
| **탈출 보너스(EB) / exit** | `comunicator.send_end_of_play`('E') |
| **LoP(20초 정지, 'L' 신호)** | `comunicator` `send_lack_of_progress`/`update`의 'L' 수신, `pose_manager.previous_position`(StuckDetector 지원) |
| **시간(480/600초)** | `robot.time` 경과 시간 |
| **데드레커닝 금지(자율 제어)** | 자이로 단독 적분에 의존하지 않고 GPS로 주기 교정(`pose_manager.calculate_orientation`) |

---

## 7. 다른 모듈과의 상호작용 요약

```
RescueRobot(facade)
   └─ 생성 ─> Robot(robot.py)
                 ├─ PoseManager ──> Gps, Gyroscope
                 ├─ DriveBase ────> Wheel(좌/우)  [RotationManager / SmoothMovement...]
                 ├─ Lidar, Camera×3, Comunicator
                 └─ update()/do_loop()  <── Executor 메인 루프가 매 스텝 호출

데이터 흐름:
  센서(Webots) → devices/* → Robot 래퍼 → Mapper(LiDAR/카메라), Agent(위치/방향)
  Agent/Executor → Robot.move_to_coords/rotate_to_angle → DriveBase → Wheel → Webots Motor
  fixture_detection 결과 → Comunicator.send_victim → 서버
  FinalMatrixCreator 결과 → Comunicator.send_map → 서버
  Executor 종료 판단 → Comunicator.send_end_of_play
```

- **본 모듈을 호출하는 쪽**: `RescueRobot`(조립), `Executor`(루프), `Agent`(이동/방향), `Mapper`(센서 데이터), `FinalMatrixCreator`(맵 전송).
- **본 모듈이 호출하는 쪽**: Webots `controller` API 전체, 내부 `data_structures`(Angle/Position2D/Vector2D), `flow_control`(StepCounter), `utilities`(mapVals, getCoordsFromRads, divide_into_chunks, multiplyLists), `struct`, `numpy`.
