# 단계1 — 환경 입문: 프로젝트 구조와 실행 원리

## 목적
단계2 이후 수업에서 "어디를 보면 되는지", "왜 이렇게 되는지"를 이해하기 위한  
프로젝트 전체 구조 파악 단계입니다.

---

## 수업 구성 (총 75분)

| Step | 제목 | 시간 | 예제 파일 |
|------|------|------|----------|
| 01 | 프로젝트 구조 파악 | 25분 | 예제00_구조탐색.py |
| 02 | 시뮬레이션 루프 | 25분 | 예제00b_루프시뮬레이션.py |
| 03 | 데이터 흐름 추적 | 25분 | 예제00c_데이터흐름추적.py |
| 04 | 실행과 배포 (run.py vs compiled.py) | 20분 | 예제00d_빌드시스템이해.py |

---

## 예제 실행 순서

```bash
# Step 01 — 폴더 구조와 클래스 관계 시각화
cd curriculum/단계1_환경입문/step01_프로젝트구조
python 예제00_구조탐색.py

# Step 02 — 시뮬레이션 루프와 상태기계 동작
cd ../step02_시뮬레이션루프
python 예제00b_루프시뮬레이션.py

# Step 03 — 센서→지도→결정→행동 데이터 흐름
cd ../step03_데이터흐름
python 예제00c_데이터흐름추적.py
```

---

## 이 단계를 마치면 알 수 있는 것

- `main.py`가 Robot, Mapper, Executor를 만들어 연결하는 방식
- 매 32ms마다 `robot.update() → mapper.update() → state_machine.run()` 순서로 실행됨
- 6가지 상태(init/explore/report_fixture/send_map/stuck/end)의 역할과 전환 조건
- GPS/라이다/카메라 데이터가 어떤 형태로 가공되어 바퀴 속도로 이어지는지
- `compiled.py`는 절대 수정하지 않는다 — 개발은 `src/`에서

---

## 핵심 파일 참조

| 파일 | 핵심 내용 |
|------|----------|
| [src/main.py](../../src/main.py) | Robot + Mapper + Executor 조립 |
| [src/executor/executor.py](../../src/executor/executor.py) | 메인 루프 + 6가지 상태 함수 |
| [src/agent/agent.py](../../src/agent/agent.py) | 서브에이전트 우선순위 조합 |
| [src/robot/robot.py](../../src/robot/robot.py) | 모든 센서/바퀴 래퍼 |
| [src/mapping/mapper.py](../../src/mapping/mapper.py) | 지도 시스템 조율 |
| [src/flags.py](../../src/flags.py) | 디버그 옵션 ON/OFF |
