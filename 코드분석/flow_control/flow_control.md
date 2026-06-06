# `flow_control` 모듈 상세 분석

> RoboCupJunior Rescue Simulation 2026 — Webots/Erebus 자율 로봇 코드
> 대상 폴더: `src/flow_control/`

---

## 1. 폴더/모듈 개요

`flow_control`은 **로봇의 시간·순서 제어를 담당하는 유틸리티 묶음**이다. Webots 시뮬레이션은 `robot.step(timestep)`을 반복 호출하는 **단일 메인 루프(이벤트 루프)** 구조이므로, `time.sleep()`처럼 루프를 멈추는 블로킹 코드를 쓰면 센서 갱신·게임매니저 통신이 모두 멈춰버린다. 따라서 "1초 대기", "앞으로 갔다가 회전", "n스텝마다 한 번만 라이다 처리" 같은 동작을 **루프를 멈추지 않고(non-blocking)** 매 프레임 조금씩 진행하는 메커니즘이 필요하다. 이 폴더의 4개 파일이 바로 그 기반을 제공한다.

구성 요소는 서로 조합되어 동작한다.
- **StateMachine** (`state_machine.py`): 로봇의 큰 동작 모드(init→explore→report_fixture→...)를 분리하는 유한 상태 머신.
- **Sequencer** (`sequencer.py`): 한 상태 함수 안에서 여러 동작을 **순서대로** 실행하게 해주는 순차 실행기.
- **DelayManager** (`delay.py`): 시뮬레이션 시간 기반의 **비동기 지연(delay)**. 토큰 앞 1초 정지 같은 타이밍의 핵심.
- **StepCounter** (`step_counter.py`): n 타임스텝에 한 번만 무거운 작업을 실행하는 **주기 카운터**(성능 최적화).

이 모듈은 자체 알고리즘(매핑/경로탐색 등)을 갖지 않으며, **상위 제어 계층인 `Executor`와 `Agent`가 자신의 로직을 표현하는 골격**으로 쓴다.

---

## 2. 파일별 상세 분석

### 2.1 `state_machine.py` — 유한 상태 머신(FSM)

**목적**
로봇의 동작 모드를 명확히 분리하고, 각 상태에서 실행할 함수와 "그 상태에서 이동 가능한 다음 상태들"을 관리한다. 허용되지 않은 상태 전환을 막아 제어 흐름을 안전하게 한다.

**핵심 클래스/함수와 시그니처**

```python
class StateMachine:
    def __init__(self, initial_state, function_on_change_state=lambda: None)
    def create_state(self, name: str, function: Callable, possible_changes=set())
    def change_state(self, new_state)
    def check_state(self, state)
    def run(self)
```

내부 자료구조 (`state_machine.py:11-19`):
- `self.state`: 현재 활성 상태 이름(문자열).
- `self.current_function`: 현재 상태에 대응하는 실행 함수.
- `self.change_state_function`: 상태가 바뀔 때마다 추가로 호출되는 콜백.
- `self.state_functions`: `{상태이름: 실행함수}` 딕셔너리.
- `self.allowed_state_changes`: `{상태이름: {허용된 다음 상태들의 집합}}`.
- `self.possible_states`: 등록된 모든 상태 이름 집합.

**사용 알고리즘 — 상태/전환/콜백 메커니즘 (핵심)**

이 FSM은 **상태 그래프를 명시적 인접 집합(adjacency set)으로 표현**한 전형적 룩업 테이블 기반 상태 머신이다. 세 단계로 동작한다.

1. **상태 등록 (`create_state`, `state_machine.py:21-30`)**
   - 이름이 중복이면 `ValueError`를 던져 등록을 거부한다(상태 이름 유일성 보장).
   - `possible_changes`(집합)는 그 상태에서 전환이 **허용된 목적지 상태들의 화이트리스트**다. 인자를 생략하면 빈 집합 → 어디로도 못 나가는 **종착(terminal) 상태**가 된다(예: Executor의 `"end"`).
   - 등록하는 상태가 초기 상태와 같으면, 그 순간 `current_function`을 바로 채워서 첫 `run()` 호출이 정상 동작하게 한다.

2. **상태 전환 (`change_state`, `state_machine.py:32-45`)**
   - 존재하지 않는 상태로 전환 시도 → `ValueError`.
   - **현재 상태의 허용 집합에 목적지가 들어 있을 때만** 실제 전환: 콜백(`change_state_function`) 호출 → `self.state` 갱신 → `current_function` 교체 → 로그 출력.
   - 허용되지 않은 전환은 **예외를 던지지 않고 경고만 출력 후 무시**한다(`state_machine.py:43-44`). 즉 잘못된 전환 요청이 와도 로봇은 멈추지 않고 현재 상태를 유지한다 — 실시간 제어에서 견고성을 위한 설계.
   - 콜백을 **상태 변경 직전**에 호출하는 점이 중요하다. 콜백은 항상 인자 없이 호출된다(`self.change_state_function()`).

3. **상태 실행 (`run`, `state_machine.py:51-53`)**
   - `self.current_function(self.change_state)` — 즉 **현재 상태 함수에게 `change_state` 메서드 자체를 인자로 넘긴다.** 이 덕분에 각 상태 함수는 내부에서 `change_state_function("explore")`처럼 호출해 **스스로 다음 상태로 넘어갈** 수 있다. 외부에서 상태를 제어할 필요가 없는, 상태 함수 주도형(state-driven) 전환 패턴이다.

`check_state(state)` (`state_machine.py:47-49`)는 단순히 현재 상태가 인자와 같은지 비교(`do_end()` 등 외부 판단에 쓰임).

**사용 기능/라이브러리**
- 표준 라이브러리 `typing.Callable`만 사용. 외부 의존성 없음(순수 파이썬).

**입력/출력 데이터 흐름**
- 입력: 등록된 상태 함수들, 외부 또는 상태 함수 자신의 전환 요청.
- 출력: 부수효과(상태 함수 실행, 콜백 호출). `run()`은 현재 상태 함수의 반환값을 그대로 돌려준다.

---

### 2.2 `sequencer.py` — 순차 동작 시퀀서

**목적**
"매 타임스텝 호출되는 함수" 안에서 여러 동작을 **순서대로** 실행한다. 멀티스레딩/멀티프로세싱 없이, 메인 루프를 중단하지 않으면서 "앞으로 이동 → 0.5초 대기 → 회전" 같은 연속 동작을 구현한다.

**핵심 클래스/함수와 시그니처**

```python
class Sequencer:
    def __init__(self, reset_function=None)
    def reset_sequence(self)
    def seq_reset_sequence(self)
    def start_sequence(self)
    def check(self)
    def next_seq(self)
    def seq_done(self)
    def simple_event(self, function=None, *args, **kwargs)
    def complex_event(self, function, *args, **kwargs)
    def make_simple_event(self, function)
    def make_complex_event(self, function)
```

**사용 알고리즘 — 프로그램 카운터 방식 협조적 시퀀싱**

핵심 아이디어는 **두 개의 카운터**다(`sequencer.py:17-21`).
- `line_pointer`: "지금 실행해야 할 명령은 몇 번째인가"를 기억(프로그램 카운터 역할). 프레임이 넘어가도 **유지**된다.
- `line_identifier`: 매 프레임 `start_sequence()`에서 0으로 리셋되고, 명령을 만날 때마다 1씩 증가하며 각 명령에 순번을 부여.

동작 순서:
1. 상태 함수 진입 시 `start_sequence()` 호출 → `line_identifier=0`, `done=False` (`sequencer.py:40-46`).
2. 각 순차 명령은 `check()`를 호출한다. `check()`는 `line_identifier += 1` 후 `line_identifier == line_pointer`이면 `True`를 반환(`sequencer.py:48-56`). 즉 **이번 프레임에 실행할 차례인 단 하나의 명령만 True**가 된다.
3. 명령이 완료되면 `next_seq()`로 `line_pointer += 1`, `done=True` → 다음 프레임에는 그다음 명령이 실행된다(`sequencer.py:58-61`).

이렇게 매 프레임 위에서부터 명령들을 훑되 **포인터가 가리키는 한 개만 실제 실행**하고, 그 명령이 끝나야 포인터가 전진한다 → 결과적으로 명령이 **한 줄씩 순서대로** 진행된다. OS의 협조적 멀티태스킹(코루틴/제너레이터)과 같은 발상을 카운터로 구현한 것.

**이벤트 종류 두 가지**
- **simple_event** (`sequencer.py:67-78`): 한 프레임에 끝나는 1회성 동작. `check()`가 True면 함수(있으면) 실행 후 즉시 `next_seq()`. `function=None`이면 "이 차례에 도달했을 때 True를 반환하는 게이트"로 쓰여, `if self.sequencer.simple_event():` 블록 안에 임의 코드를 순차 실행할 수 있다(Executor가 즐겨 쓰는 패턴).
- **complex_event** (`sequencer.py:80-90`): 여러 프레임에 걸쳐 진행되는 동작. 인자 함수가 **True를 반환할 때까지** 같은 명령에 머물고, True가 되어야 `next_seq()`로 전진. 이동 완료/회전 완료/지연 완료 같은 "끝나면 True" 함수와 결합한다.

**래퍼 팩토리**
`make_simple_event`/`make_complex_event` (`sequencer.py:92-116`)는 일반 함수를 받아 위 로직으로 감싼 클로저를 돌려준다. Executor가 `self.seq_move_to_coords = self.sequencer.make_complex_event(self.robot.move_to_coords)`처럼 미리 만들어 두고, 상태 함수 안에서는 `self.seq_move_to_coords(...)` 한 줄로 순차 명령을 기술한다(`executor.py:70-76`).

**리셋 메커니즘**
- `reset_sequence()` (`sequencer.py:23-31`): `line_pointer`를 1로 되돌려 시퀀스를 처음부터 재시작. 생성 시 받은 `reset_function`(있으면)을 **먼저** 호출한다 — Executor는 여기에 `DelayManager.reset_delay`를 연결해(`executor.py:58`) **시퀀스 리셋 시 진행 중이던 지연 상태도 함께 초기화**한다.
- `seq_reset_sequence()` (`sequencer.py:33-38`): 위를 `check()`로 감싼 순차판. 시퀀스의 마지막 명령으로 두면, 모든 명령이 끝난 뒤 차례가 돌아와 리셋이 실행되고 시퀀스가 처음부터 다시 돈다(상태가 반복 실행되는 explore 류에서 사용).
- `seq_done()` (`sequencer.py:63-65`): 직전 명령이 `next_seq`로 완료를 표시했는지 확인.

**사용 기능/라이브러리**
- `flags.SHOW_DEBUG` 플래그로 디버그 출력만 제어. 외부 라이브러리 없음.

**입력/출력 데이터 흐름**
- 입력: 상태 함수가 기술한 순차 명령 나열.
- 출력: 부수효과(차례가 된 명령 실행). 각 이벤트 함수는 "이번 프레임에 실행됐는가"를 bool로 반환.

---

### 2.3 `delay.py` — 비동기 지연 관리자

**목적**
시뮬레이션을 멈추지 않고 "n초 대기"를 구현한다. `time.sleep()`은 루프 전체를 멈춰 센서 업데이트가 끊기므로 쓸 수 없다(`delay.py:7-9`). 이 클래스는 Sequencer의 `complex_event`와 결합해 비블로킹 지연을 제공한다.

**핵심 클래스/함수와 시그니처**

```python
class DelayManager:
    def __init__(self) -> None
    def update(self, time)
    def delay_seconds(self, delay)   # 완료 시 True (complex_event용)
    def reset_delay(self)
```

**사용 알고리즘 — 시뮬레이션 시간 기준 경과시간 비교**

- `update(time)` (`delay.py:16-18`): 매 타임스텝 Webots의 현재 시뮬레이션 시각(초)을 저장. Executor 루프(`executor.py:104`)와 RescueRobot(`rescue_robot.py:360`)에서 매 프레임 호출한다.
- `delay_seconds(delay)` (`delay.py:20-37`): 상태를 가진 함수다.
  - **첫 호출**: `delay_first_time`이 True → 현재 시각을 `delay_start`에 기록하고 플래그를 끈 뒤 `False` 반환(아직 대기 중).
  - **이후 호출**: `self.time - self.delay_start >= delay`이면 플래그를 다시 True로 되돌리고(재사용 대비) `True` 반환(완료). 아니면 `False`.
  - 실제 벽시계가 아닌 **시뮬레이션 시간 차이**로 판정하므로, 시뮬 속도와 무관하게 정확한 "초"가 보장된다.
- `reset_delay()` (`delay.py:39-41`): `delay_first_time=True`로 되돌려 지연을 처음 상태로 초기화. Sequencer의 `reset_function`으로 등록되어 시퀀스 리셋과 동기화된다.

**Sequencer와의 결합**
Executor는 `self.seq_delay_seconds = self.sequencer.make_complex_event(self.delay_manager.delay_seconds)`로 묶는다(`executor.py:75`). `delay_seconds`가 True를 반환할 때까지 시퀀서가 같은 명령에 머무르므로, 그 사이 메인 루프는 계속 돌며 센서/통신을 갱신한다. → 정확히 "루프를 살린 채 n초 대기".

**사용 기능/라이브러리**
- `flags.SHOW_DEBUG`만. 외부 라이브러리 없음.

**입력/출력 데이터 흐름**
- 입력: 매 프레임의 시뮬 시각(`update`), 대기 목표 초(`delay_seconds`의 인자).
- 출력: 완료 여부 bool.

---

### 2.4 `step_counter.py` — 주기 스텝 카운터

**목적**
무거운 처리(라이다 포인트클라우드 생성, 카메라 이미지 디코딩, 경로 재계산 등)를 매 스텝이 아니라 **n 스텝에 한 번만** 실행해 연산량을 줄인다.

**핵심 클래스/함수와 시그니처**

```python
class StepCounter:
    def __init__(self, interval)
    def increase(self)
    def check(self)
```

**사용 알고리즘 — 모듈러 순환 카운터**

- 내부 `__current_step`을 0부터 시작(`step_counter.py:11`).
- `increase()` (`step_counter.py:14-18`): 매 프레임 호출, 1 증가 후 `interval`에 도달하면 0으로 리셋(0 ~ interval-1 순환).
- `check()` (`step_counter.py:20-22`): 카운터가 0일 때 True → **interval 프레임마다 정확히 한 번** True. 즉 `if check(): 무거운작업()` 다음 `increase()`를 두면 주기 실행이 된다.

**사용 기능/라이브러리**
- 순수 파이썬. 의존성 없음.

**입력/출력 데이터 흐름**
- 입력: 매 프레임의 `increase()` 호출.
- 출력: `check()`의 bool(주기 도래 여부).

---

## 3. 규정 연관성 (RoboCupJunior Rescue Simulation 2026)

- **토큰 식별 "앞에서 최소 1초 정지" 규정 (규정 17번)**: 토큰 종류 전송이 인정되려면 토큰 앞에서 최소 1초 정지해야 한다. 이 타이밍을 `DelayManager`+`Sequencer`가 구현한다. Executor의 `state_report_fixture`는 정렬 후 정지 → **1.5초 대기**(`executor.py:365`, 규정의 1초 이상을 안전 마진으로 충족) → `send_victim` 전송(`executor.py:370`) 순서를, 시뮬레이션을 멈추지 않고(센서·통신 유지) 진행한다. 만약 블로킹 sleep을 썼다면 그 1초간 게임매니저 통신/카메라가 멈춰 보고가 누락될 위험이 있다.
- **시간 제한(8분/480초) 및 탐색→복귀 전환**: `Agent`의 단계 머신이 `max_time = 8*60`(`agent.py:93`)을 기준 삼아 탐색/복귀를 전환한다 — 제한 시간 내 시작 타일 복귀로 **탈출 보너스 EB(+10%)**를 노리는 흐름의 골격.
- **오식별 패널티(TMI, -5점) 회피**: `state_report_fixture`는 보고 직전 카메라로 토큰을 재확인하고, 실패 시 블랙리스트 등록 + 쿨다운 후 explore로 복귀한다(`executor.py:342-359`). 이 다단계 검증 흐름 전체가 Sequencer의 순차 게이트로 구성된다.
- **LoP(20초 정지) / 끼임 탈출**: Executor의 `"stuck"` 상태가 StateMachine으로 분리되어 후진→대기→제자리 턴 시퀀스(`executor.py:399-414`)를 수행, 20초 정지로 인한 자동 LoP(-5점, 규정 19번)를 예방한다.
- **노이즈 강건성 / 성능**: v26에서 GPS 노이즈가 추가된 환경에서 라이다·카메라 같은 무거운 센서 처리를 `StepCounter`로 주기 실행(라이다 6스텝, 카메라 3스텝 — `robot.py:53-76`)해 실시간 10분 제약 안에서 메인 루프를 가볍게 유지한다.

직접적인 점수 계산이나 맵 행렬 생성은 이 모듈이 하지 않는다 — 어디까지나 **타이밍·순서·상태 제어의 하부 골격**이다.

---

## 4. 다른 모듈과의 상호작용

**StateMachine**
- **Executor** (`executor.py:47-54`): 메인 상태 머신. `init`→`explore`→{`report_fixture`, `send_map`, `stuck`, `end`} 전환 그래프를 등록. 전환 화이트리스트를 명시(예: `explore`는 `{"end","report_fixture","send_map","stuck"}`로만 이동 가능). `end`는 화이트리스트 없는 종착 상태. 매 프레임 `state_machine.run()` 호출(`executor.py:120`, `rescue_robot.py:390`), 조건 충족 시 `change_state(...)` 또는 상태 함수가 받은 `change_state_function(...)`로 전환.
- **Agent** (`agent.py:84-89`): 2단계 머신 `explore`↔`return_to_start`. 생성 시 콜백으로 `__set_force_calculation`을 등록 → **단계가 바뀔 때마다 `do_force_calculation=True`**로 만들어(`agent.py:143-145`) 다음 프레임 경로 재계산을 강제한다. 이는 StateMachine 콜백 메커니즘의 대표 활용 예다. `__stage_explore`는 목표가 일정 프레임(120) 연속 없으면 `change_state_function("return_to_start")`로 복귀 전환(`agent.py:121-125`). `do_end()`는 `check_state` 대신 `self.state` 직접 비교로 복귀 완료를 판정(`agent.py:111-114`).

**Sequencer**
- **Executor 전용** (`executor.py:58`). `reset_function=self.delay_manager.reset_delay`로 DelayManager와 묶임. 모든 상태 함수(`state_init`, `state_explore`, `state_report_fixture`, `state_stuck`, 캘리브레이션 루틴 등)가 `start_sequence()`로 시작해 `seq_*` 이벤트로 동작을 나열하고 `seq_reset_sequence()`/`reset_sequence()`로 끝낸다. 미리 래핑된 이벤트들(`seq_move_wheels`, `seq_rotate_to_angle`, `seq_move_to_coords`, `seq_delay_seconds`, `seq_align_with_fixture` 등 — `executor.py:70-76`)이 Robot/DelayManager의 메서드를 호출한다.

**DelayManager**
- **Executor**가 소유(`executor.py:42`), 매 프레임 `update(robot.time)` 호출(`executor.py:104`). `RescueRobot`도 facade에서 `self._executor.delay_manager.update(self._robot.time)`로 갱신(`rescue_robot.py:360`). Sequencer를 통해 `seq_delay_seconds(...)`로 소비됨.

**StepCounter** (가장 널리 재사용됨)
- **robot.py**: 라이다(interval 6), 전/좌/우 카메라(interval 3) 각각에 주입(`robot.py:56,64,70,76`). `TimedSensor.update()`가 매 프레임 `step_counter.increase()`(`sensor.py:24`), 각 센서의 데이터 접근 메서드가 `step_counter.check()`로 주기를 맞춤(`camera.py`, `lidar.py`).
- **executor.py**: `mini_calibrate_step_counter = StepCounter(20)` — 20스텝마다 위치 보정 트리거(`executor.py:88, 265-268`).
- **agent.py**: `PathTimeCalculator` 갱신(300스텝)·복귀 위치 로그(200스텝) 주기 제어(`agent.py:95,99`).
- **mapping/array_filtering.py**: 고립점 제거(20스텝)·들쭉날쭉 엣지 정리(100스텝)(`array_filtering.py:35-56`).
- **data_structures/compound_pixel_grid.py**에서도 import(`compound_pixel_grid.py:7`).

**호출 관계 요약**
- `flow_control`은 **누구도 호출하지 않는 순수 도구 계층**이다(상위에서 import해 사용만 됨).
- 호출하는 쪽: `Executor`(4개 전부), `Agent`(StateMachine, StepCounter), `Robot`/디바이스(StepCounter), `Mapper` 계열(StepCounter), `RescueRobot`(DelayManager/StateMachine 간접).
