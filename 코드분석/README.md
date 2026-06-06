# RoboCup Rescue Simulation 2026 — `src/` 코드 분석

Webots/Erebus 자율 구조 로봇 컨트롤러(`src/`, 약 7,500줄 Python)를 모듈 폴더 단위로 상세 분석한 문서 모음. 폴더 분류 체계는 `src/`와 동일하게 미러링.

대회: RoboCupJunior Rescue Simulation 2026 (규정: `RCJRescueSimulation2026-final.pdf`, 최종 갱신 2026-02-11).

---

## 1. 한눈에 보는 시스템

미로를 자율 탐색 → 벽 토큰(조난자/해즈맷) 식별·보고 → 지도 행렬 제출 → 시작점 복귀(exit) 하는 SLAM + 의사결정 로봇.

```
run.py  (Erebus 진입점, src 경로 주입)
  └─ main.py  (실행 예시 4종 중 1개 선택)
       └─ RescueRobot  (초보자용 facade API · rescue_robot.py)
            ├─ Robot              하드웨어 추상화 (센서/모터/통신)
            ├─ Mapper             센서 → 픽셀그리드 지도 (SLAM 매핑)
            ├─ Executor           메인 루프 + 최상위 상태머신
            │    └─ Agent         "어디로 갈까" 의사결정 (탐색/복귀)
            │         ├─ Subagents      우선순위 폴백 (조난자>벽>미탐색)
            │         └─ Pathfinding    A*/BFS 경로 계산
            └─ FinalMatrixCreator 픽셀그리드 → 규정 맵 행렬 (매핑보너스)
```

핵심 데이터 구조는 **하나의 공유 픽셀 그리드**(`CompoundExpandablePixelGrid`, 24개 레이어). 모든 매퍼가 여기에 쓰고, 모든 에이전트/경로탐색이 여기서 읽는다.

### 매 타임스텝(32ms) 데이터 흐름

```
센서(GPS/LiDAR/카메라/자이로)
  → Robot.update()           원시값 읽기 + pose 추정(GPS↔자이로 융합)
  → Mapper.update()          7개 매퍼로 픽셀그리드 레이어 갱신
  → Agent.update()           그리드 위에서 BFS(어디로)+A*(어떻게) → 목표 좌표
  → DriveBase.move_to_coords 비례제어로 모터 구동
  → 토큰 발견 시 1초+ 정지 → Comunicator.send_victim()
  → 종료 시 FinalMatrixCreator → Comunicator.send_map() + send_end_of_play()
```

---

## 2. 모듈별 문서

| 폴더 | 역할 | 문서 |
|------|------|------|
| `_top_level` | 진입점·facade·플래그·시각화·유틸 | [_top_level.md](_top_level/_top_level.md) |
| `executor` | 메인 루프, 최상위 상태머신, LoP 감지 | [executor.md](executor/executor.md) |
| `agent` | 의사결정(탐색/복귀), 서브에이전트, 경로탐색 | [agent.md](agent/agent.md) |
| `algorithms` | A*/BFS (numpy 불리언 그리드) | [algorithms.md](algorithms/algorithms.md) |
| `mapping` | 센서→픽셀그리드 SLAM 매핑 | [mapping.md](mapping/mapping.md) |
| `fixture_detection` | 카메라 토큰 탐지·분류 (victim/해즈맷/가짜) | [fixture_detection.md](fixture_detection/fixture_detection.md) |
| `final_matrix_creation` | 픽셀그리드→규정 맵 행렬 | [final_matrix_creation.md](final_matrix_creation/final_matrix_creation.md) |
| `robot` | 하드웨어 추상화 (센서/모터/Erebus 통신) | [robot.md](robot/robot.md) |
| `data_structures` | angle/vectors/그리드 기반 자료구조 | [data_structures.md](data_structures/data_structures.md) |
| `flow_control` | 상태머신/시퀀서/지연/스텝카운터 유틸 | [flow_control.md](flow_control/flow_control.md) |

---

## 3. 규정 ↔ 코드 매핑 요약

| 2026 규정 항목 | 구현 위치 |
|---------------|-----------|
| 벽 토큰 식별(TI) + 종류(TT) | `fixture_detection/` (HSV+컨투어+동심원 색합산) |
| 토큰 앞 1초+ 정지 후 보고 | `executor` state_report_fixture(1.5초) + `flow_control` Delay |
| 오식별(TMI -5점) 회피 | `fixture_detection` 불확실 시 None 반환, 가짜 3D victim 레이캐스팅 제거 |
| 매핑 보너스(MB) | `final_matrix_creation/` (쿼터타일 4×4 노드 인코딩) |
| 체크포인트(CN)/늪/구멍 | `mapping/floor_mapper` (바닥색 HSV 분류) |
| 탈출 보너스(EB) | `agent` return_to_start 단계 + `executor` end 상태 |
| 8분(480초) 제한 | `executor` 478초 듀얼클록 타임아웃 |
| LoP(20초 정지) | `executor/stuck_detector` (⚠ 아래 버그 참고) |
| GPS 노이즈 강건성(v26) | `robot/pose_manager` GPS↔자이로 융합 + `mapping` 누적임계화·고립점 제거 |
| 데드레커닝 금지 | GPS로 자이로 드리프트 주기 교정 |

---

## 4. 분석 중 발견한 주요 이슈

분석 과정에서 확인된 실제 코드 문제(제출 전 검토 권장):

1. **LoP 대응 파이프라인 단절** — `StuckDetector.update()`는 매 프레임 호출되지만, `is_stuck()`를 읽어 stuck 상태로 전환하는 코드가 `src/` 전체에 없음. 규정의 LoP(20초 정지) 자동 탈출이 실제로 작동하지 않음. (`executor/`)
2. **맵 행렬 규정 누락** — 토큰 글자코드(H/S/U/F/P/C/O), 장애물 `x`, Area4 `*`가 최종 행렬에 인코딩되지 않음. 통로도 규정의 `b/y/g/p/o/r` 대신 숫자 6/7/8 사용 → MB 정확도 손실 가능. (`final_matrix_creation/`)
3. **딕셔너리 키 중복** — `FloorMatrixCreator.__floor_color_ranges`에 `"0"` 키 중복 정의로 일반 바닥색 범위가 void 정의에 덮임. (`final_matrix_creation/final_matrix_creator.py:168,174`)
4. **늪 자이로 전환 비활성** — `check_swamp_proximity()`가 정의만 되고 `run()`에서 미호출. (`executor/`)
5. **이식성 위험** — `run.py:7` 절대경로 하드코딩, `flags.py` 리눅스 고정 저장경로(macOS 저장 시 실패), `utilities.normalizeRads`의 `rad += 2 + math.pi`(2π 의도 의심). (`_top_level/`)
6. **미사용/레거시 코드** — `algorithms/a_star.py`(efficient 버전만 실사용), `PathTimeCalculator`(생성만 되고 호출 흐름 없음), `data_structures/tile_color_grid`(`get_colored_grid` 미구현), `mapping/data_extractor` 일부 템플릿매칭·HSV(주석상 레거시).

각 이슈의 상세 근거는 해당 모듈 문서 참조.

---

## 5. 사용된 핵심 기술

- **numpy** — 모든 그리드 연산의 기반(불리언/값 레이어, 좌표 변환).
- **OpenCV (cv2)** — HSV 색공간 필터링, 컨투어 검출, IPM 역투시변환(`warpPerspective`), 모폴로지/컨볼루션 커널(`filter2D`), 실시간 시각화.
- **scikit-image** — Bresenham 직선(LiDAR 빔 궤적, 레이캐스팅).
- **Webots 컨트롤러 API** — Robot/Motor/Lidar/GPS/Camera/Gyro/Emitter/Receiver.
- **struct** — Erebus 게임매니저 바이너리 프로토콜 패킹.
- **알고리즘** — A*(옥타일 휴리스틱+벽회피 패널티), BFS(목표 미상 최근접 탐색), 유한상태머신, 비례(P)제어, 동심원 색값 합산.
