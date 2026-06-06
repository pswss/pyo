# 구조 로봇 소스 코드 이해를 위한 파이썬 학습 자료

이 폴더는 `src/` 소스 코드를 읽고 수정할 수 있는 수준이 되기 위한 단계별 학습 자료입니다.
함수(def)까지는 이미 알고 있다고 가정합니다.

---

## 학습 순서

| 회차 | 주제 | 파일 | 예상 시간 |
|------|------|------|-----------|
| 1 | 클래스와 객체 | `01_클래스와_객체.md` | 1시간 |
| 2 | 모듈·임포트·f-string | `02_모듈_임포트_fstring.md` | 1시간 |
| 3 | @property·이름 규칙·상속 | `03_property_상속_이름규칙.md` | 1시간 |
| 4 | 타입 힌트·람다·None 패턴 | `04_타입힌트_람다_None.md` | 1시간 |
| 5 | NumPy 배열·OpenCV 기초 | `05_numpy_opencv.md` | 1시간 |

---

## 이 자료를 끝내면 할 수 있는 것

- `src/rescue_robot.py` — 로봇 API 전체를 읽고 이해
- `src/executor/executor.py` — 상태 머신 흐름 추적
- `src/map_visualizer.py` — 시각화 코드 수정·색상 변경
- `src/mapping/fixture_mapper.py` — 중복 보고 반경 조정
- `src/agent/agent.py` — 탐색 완료 조건 수정

---

## 실습 파일 실행 방법

각 회차마다 `XX_practice.py` 파일이 있습니다.
Webots 없이 일반 파이썬으로 실행 가능합니다.

```bash
# 예시: 1회차 실습 실행
python study/01_practice.py
```

회차 5의 실습은 numpy와 opencv가 필요합니다:
```bash
pip install numpy opencv-python
```
