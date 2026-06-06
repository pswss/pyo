# RoboCup 세계대회 출전반 — 교육 커리큘럼

## 대상: 고등학생 (파이썬 기초 수준)
## 목표: 자율주행 구조 로봇 코드 완벽 이해 및 응용

---

## 폴더 구조

```
curriculum/
├── 단계1_환경입문/         ← NEW: 프로젝트 구조 파악
│   ├── step01_프로젝트구조/  (예제00_구조탐색.py)
│   ├── step02_시뮬레이션루프/ (예제00b_루프시뮬레이션.py)
│   └── step03_데이터흐름/   (예제00c_데이터흐름추적.py)
├── 단계2_로봇제어/
│   ├── step01_바퀴제어/
│   ├── step02_좌표계와GPS/
│   └── step03_각도와회전/
├── 단계3_센서와인식/
│   ├── step01_라이다/
│   ├── step02_카메라기초/
│   └── step03_피해자감지/
├── 단계4_지도만들기/
│   ├── step01_격자지도/
│   ├── step02_벽감지와매핑/
│   └── step03_탐색영역추적/
├── 단계5_길찾기알고리즘/
│   ├── step01_BFS탐색/
│   ├── step02_A스타알고리즘/
│   └── step03_경로최적화/
└── 단계6_통합과전략/
    ├── step01_상태기계/
    ├── step02_에이전트전략/
    └── step03_모의대회준비/
```

## 수업 진행 원칙

1. **코드를 보기 전에 개념을 먼저** — 그림/손으로 먼저 이해
2. **항상 실행해보기** — 이론보다 실습이 먼저
3. **왜?를 항상 질문하기** — 단순 암기 금지
4. **우리가 고친 버그를 기억하기** — 실수에서 배운다

## 각 단계 구성

각 step 폴더 안에:
- `교안.md` — 수업 내용 (개념 설명 + 질문 + 토론)
- `예제XX_이름.py` — 단계별 실습 코드
- `퀴즈.md` — 이해도 확인 문제

---

## 프로젝트 실행 전 체크리스트

```bash
# 1. 패키지 설치 확인
python -m pip list | findstr numpy
python -m pip list | findstr opencv
python -m pip list | findstr scikit

# 2. 컴파일
C:\Users\snowbot\AppData\Roaming\Python\Python314\Scripts\stickytape.exe src/main.py --add-python-path src/ | Out-File -Encoding utf8 compiled.py

# 3. Erebus에서 compiled.py 로드 후 실행
```
