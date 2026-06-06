# 코드 설명: WallTokenDetection (벽면 토큰 감지 시스템)

이 코드는 **Webots 시뮬레이터**에서 로봇이 벽에 붙어있는 토큰(표지판)을 찾아서 식별하고 보고하는 프로그램이에요.

---

## 🤖 전체 흐름 (한눈에 보기)

```
탐색 → 토큰 발견 → 접근 → 정렬 → 식별 → 보고 → 다시 탐색
```

---

## 1. 초기화 부분

```python
camera = robot.getDevice("camera_centre")  # 카메라
emitter = robot.getDevice("emitter")        # 데이터 송신기
receiver = robot.getDevice("receiver")      # 데이터 수신기
front_sensor = robot.getDevice('ps0')       # 전방 거리센서
left_motor / right_motor                    # 바퀴 모터
```

로봇의 센서와 모터들을 연결하는 부분이에요.

---

## 2. 토큰 감지 함수 `detect_wall_tokens()`

카메라 이미지에서 토큰을 찾는 핵심 함수예요.

### 처리 과정:

```
원본 이미지 → 여러 마스크 생성 → 합치기 → 윤곽선 찾기 → 토큰 추출
```

### 사용하는 마스크들:

| 마스크 | 목적 |
|--------|------|
| `mask_bright` | 밝은 영역 감지 |
| `mask_otsu` | 자동 임계값으로 감지 |
| `mask_white` | 흰색 토큰 |
| `mask_red` | 빨간색 토큰 (위험물 표시) |
| `mask_yellow` | 노란색 토큰 |

### 토큰 조건:
- 면적: 100 ~ 3000 픽셀
- 가로세로 비율: 0.6 ~ 1.4 (대략 정사각형)

---

## 3. 토큰 분류 함수 `classify_token()`

찾은 토큰이 **무엇인지** 판별해요.

### 분류 기준:

```python
# Letter Victim (문자 표시) - 흰 배경 + 검은 글자
if 밝기 > 140 and 채도 < 100:
    검은픽셀 비율로 H/S/U 구분
    - 35% 이상 → "H"
    - 25% 이상 → "S"  
    - 12% 이상 → "U"

# Hazmat (위험물 표시) - 색상으로 구분
if 채도 > 80:
    빨강 → "F" (Flammable, 인화성)
    노랑 → "O" (Oxidizer, 산화제)

# 기타
흰색 배경 → "P" (Poison, 독극물)
어두움 → "C" (Corrosive, 부식성)
```

---

## 4. 상태 머신 (State Machine)

로봇의 행동을 7단계로 나눠서 관리해요.

```
STATE_SEARCHING (0)    → 토큰 찾는 중
        ↓
STATE_APPROACHING (1)  → 토큰에 접근 중
        ↓
STATE_BACKING (2)      → 너무 가까우면 후진
        ↓
STATE_CENTERING (3)    → 토큰을 화면 중앙에 정렬
        ↓
STATE_IDENTIFYING (4)  → 토큰 종류 식별
        ↓
STATE_STOPPED (5)      → 잠시 멈춤
        ↓
STATE_REPORTING (6)    → 결과 전송
```

---

## 5. 거리 기준값

```python
TOO_CLOSE = 0.08        # 8cm - 너무 가까움! 후진 필요
SAFE_DISTANCE = 0.12    # 12cm - 적정 거리
APPROACH_DISTANCE = 0.20 # 20cm - 접근 시작 거리
```

---

## 6. 메인 루프 핵심 동작

### 탐색 중 (SEARCHING)
```python
# 전진하면서 토큰 찾기
left_motor.setVelocity(2.0)
right_motor.setVelocity(2.0)

if valid_tokens:  # 토큰 발견!
    → 접근 상태로 전환
```

### 접근 중 (APPROACHING)
```python
# 토큰 중심과 화면 중심의 차이로 방향 조절
error = token['center_x'] - center_x

if error > 0:   # 토큰이 오른쪽에 있음
    → 오른쪽으로 틀기
else:           # 토큰이 왼쪽에 있음
    → 왼쪽으로 틀기
```

### 보고 (REPORTING)
```python
# 식별 결과를 송신기로 전송
message = struct.pack('c', current_token['type'].encode())
emitter.send(message)
```

---

## 📊 요약

| 구성요소 | 역할 |
|----------|------|
| `detect_wall_tokens()` | 이미지에서 토큰 위치 찾기 |
| `classify_token()` | 토큰 종류 판별 (H/S/U/F/O/P/C) |
| 상태 머신 | 로봇 행동 단계별 제어 |
| 거리 센서 | 벽/토큰과의 거리 측정 |

이 코드는 **RoboCup Rescue 시뮬레이션** 같은 대회에서 피해자 표시나 위험물 표지를 인식하는 데 사용되는 전형적인 패턴이에요! 🤖