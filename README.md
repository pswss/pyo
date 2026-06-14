# Rescuemind (pyo)

**RoboCupJunior Rescue Simulation 2026** 자율 구조 로봇 컨트롤러 (Webots / Erebus).

미로를 자율 탐색하며 지도를 만들고, 벽에 붙은 토큰(조난자·위험물질)을 식별·보고한 뒤,
완성한 지도 행렬을 제출하고 시작점으로 복귀하는 **SLAM + 의사결정** 로봇이다.

---

## 시스템 개요

```
run.py  (Erebus 진입점, src 경로 주입)
  └─ main.py  (실행 예시 선택)
       └─ RescueRobot  (facade API · rescue_robot.py)
            ├─ Robot              하드웨어 추상화 (센서/모터/통신)
            ├─ Mapper             센서 → 픽셀그리드 지도 (SLAM 매핑)
            ├─ Executor           메인 루프 + 최상위 상태머신
            │    └─ Agent         "어디로 갈까" 의사결정 (탐색/복귀)
            │         ├─ Subagents      우선순위 폴백 (조난자 > 벽 > 미탐색)
            │         └─ Pathfinding    A*/BFS 경로 계산
            └─ FinalMatrixCreator 픽셀그리드 → 규정 맵 행렬 (매핑 보너스)
```

핵심 자료구조는 **하나의 공유 픽셀 그리드**(`CompoundExpandablePixelGrid`)다.
모든 매퍼가 여기에 레이어를 쓰고, 모든 에이전트·경로탐색이 여기서 읽는다.

### 매 타임스텝(32ms) 데이터 흐름

```
센서(GPS/LiDAR/카메라/자이로)
  → Robot.update()            원시값 읽기 + pose 추정(GPS↔자이로 융합)
  → Mapper.update()           매퍼들로 픽셀그리드 레이어 갱신
  → Agent.update()            그리드 위 BFS(어디로) + A*(어떻게) → 목표 좌표
  → DriveBase.move_to_coords  비례(P)제어로 모터 구동
  → 토큰 발견 시 정지 → Comunicator.send_victim()
  → 종료 시 FinalMatrixCreator → send_map() + send_end_of_play()
```

---

## 실행 방법

**요구사항**
- [Webots](https://cyberbotics.com/) + [Erebus](https://erebus.rcj.cloud/) (대회 시뮬레이터)
- Python 3 + 패키지: `numpy`, `opencv-python`, `scikit-image`, `imutils`

```bash
pip install numpy opencv-python scikit-image imutils
```

**실행**
1. Webots에서 Erebus 월드를 연다.
2. `robot0` 컨트롤러가 이 저장소의 `src/main.py`를 import 하도록 설정한다.
   진입점 `run.py`(또는 Erebus 컨트롤러 스텁)가 `src/` 경로를 `sys.path`에 주입한다.
3. 시뮬레이션을 실행하면 `main.py`의 `example_autonomous()`가 초기화 → 탐색 →
   조난자 보고 → 지도 전송까지 전부 자동으로 수행한다.

`main.py`에는 세 가지 사용 방식이 있다: 완전 자율, 자율 + 사용자 코드 추가, 완전 수동 제어.

**디버그 토글** — `src/flags.py`

| 플래그 | 설명 |
|--------|------|
| `SHOW_LIVE_MAP` | 실시간 탐색 지도 창 표시 |
| `SHOW_DEBUG` | 일반 디버그 메시지 출력 |
| `SHOW_FIXTURE_DEBUG` | 조난자 탐지 과정 시각화 |
| `SHOW_GRANULAR_NAVIGATION_GRID` | A* 경로가 그려진 픽셀 그리드 표시 |
| `SHOW_PATHFINDING_DEBUG` | 경로 탐색 디버그 메시지 |
| `DO_SLOW_DOWN` | 메인 루프를 느리게 실행 (관찰용) |

---

## 프로젝트 구조

| 폴더 | 역할 |
|------|------|
| `src/executor/` | 메인 루프, 최상위 상태머신, 끼임(LoP) 감지 |
| `src/agent/` | 의사결정(탐색/복귀), 서브에이전트, 경로탐색 |
| `src/algorithms/` | A*/BFS (numpy 불리언 그리드) |
| `src/mapping/` | 센서 → 픽셀그리드 SLAM 매핑 (벽/바닥/로봇/조난자) |
| `src/fixture_detection/` | 카메라 토큰 탐지·분류 (victim / 위험물질 / 가짜 구별) |
| `src/final_matrix_creation/` | 픽셀그리드 → 규정 맵 행렬 |
| `src/robot/` | 하드웨어 추상화 (센서/모터/Erebus 통신) |
| `src/data_structures/` | angle / vectors / 픽셀 그리드 자료구조 |
| `src/flow_control/` | 상태머신 / 시퀀서 / 지연 / 스텝카운터 |
| `tests/` | 단위 테스트 (`python3 tests/test_*.py`) |
| `코드분석/` | 모듈별 상세 분석 문서 (한국어) |

---

## 핵심 기술

- **numpy** — 모든 그리드 연산(불리언/값 레이어, 좌표 변환).
- **OpenCV** — HSV 색 필터링, 컨투어 검출, IPM 역투시변환(`warpPerspective`), 모폴로지/컨볼루션(`filter2D`), 시각화.
- **scikit-image** — Bresenham 직선(LiDAR 빔 궤적, 레이캐스팅).
- **알고리즘** — A*(옥타일 휴리스틱 + 벽회피 패널티), BFS(목표 미상 최근접 탐색), 유한상태머신, 비례(P)제어.

---

## 테스트

```bash
# 개별 실행
python3 tests/test_<name>.py

# 전체 실행
for t in tests/test_*.py; do python3 "$t"; done
```

---

## 문서

모듈별 상세 분석은 [`코드분석/README.md`](코드분석/README.md) 참조.
