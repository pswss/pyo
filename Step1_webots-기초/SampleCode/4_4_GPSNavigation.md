
## 전체 개요

이 코드는 **로봇이 GPS를 사용해서 목표 지점까지 자동으로 찾아가는 프로그램**이에요. 마치 자동차 내비게이션처럼 작동하죠!

## 1단계: 준비 단계

### 로봇 초기화
```python
from controller import Robot
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())
```

**설명:**
- `Robot()`: Webots에서 로봇을 제어하는 기본 객체
- `timestep`: 시뮬레이션이 한 번 업데이트되는 시간 간격 (보통 32ms)
- `math`: 거리, 각도 계산에 필요한 수학 함수들

**비유:** 게임을 시작하기 전에 캐릭터를 생성하는 것과 같아요.

### GPS 센서 활성화
```python
gps = robot.getDevice('gps')
gps.enable(timestep)
```

**설명:**
- `getDevice('gps')`: 로봇에 달린 GPS 센서를 찾음
- `enable(timestep)`: GPS를 켜서 위치 정보를 받을 준비

**비유:** 스마트폰의 위치 서비스를 켜는 것과 같아요.

### 모터 설정
```python
left_motor = robot.getDevice('wheel1 motor')
right_motor = robot.getDevice('wheel2 motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
```

**설명:**
- 왼쪽, 오른쪽 바퀴 모터를 가져옴
- `setPosition(float('inf'))`: 무한 회전 모드 설정
  - 이렇게 하면 속도로 제어 가능 (안 하면 각도로만 제어됨)

**비유:** 자동차를 주행 모드로 바꾸는 것 (P → D)

## 2단계: 설정값 정하기

### 목표 좌표와 속도
```python
# 목표 좌표
TARGET_X = 0.2
TARGET_Y = 0.03
TARGET_Z = 0.2

# 속도 설정
BASE_SPEED = 3.0      # 기본 전진 속도
TURN_SPEED = 2.0      # 회전 속도

# 도착 판정 거리
ARRIVAL_THRESHOLD = 0.05  # 5cm 이내면 도착
```

**설명:**
- (0.2, 0.03, 0.2) 지점으로 가려고 함
- 속도 3.0은 초당 3미터를 의미하지 않고, 모터의 회전 속도예요
- 목표 지점 5cm 이내에 오면 "도착했다"고 판단

**비유:** 내비게이션에 "서울역"을 입력하고, 차 속도를 60km/h로 설정하는 것

## 3단계: 핵심 함수들

### 거리 계산 함수
```python
def calculate_distance(x1, z1, x2, z2):
    """두 점 사이의 거리 계산"""
    return math.sqrt((x2 - x1) ** 2 + (z2 - z1) ** 2)
```

**설명:**
- 피타고라스 정리를 사용한 직선 거리 계산
- 공식: √[(x2-x1)² + (z2-z1)²]

**예시:**
```python
# A지점(0, 0)에서 B지점(3, 4)까지의 거리
distance = calculate_distance(0, 0, 3, 4)
# 결과: 5.0 (√(3² + 4²) = √25 = 5)
```

**비유:** 지도에서 두 지점 사이의 직선 거리를 자로 재는 것

### 목표 각도 계산
```python
def get_target_angle(current_x, current_z, target_x, target_z):
    """현재 위치에서 목표까지의 각도 계산"""
    dx = target_x - current_x
    dz = target_z - current_z
    return math.atan2(dx, dz)
```

**설명:**
- `atan2`: 두 점을 이용해 각도를 구하는 함수 (라디안으로 반환)
- dx, dz: x축, z축 방향의 차이

**시각적 설명:**
```
         목표(0.2, 0.2)
            ↗ 
           /  각도 45°
          /
    현재(0, 0)
```

**비유:** 나침반에서 목표 방향이 몇 도인지 확인하는 것

### 로봇 방향 추정
```python
def get_robot_heading(prev_x, prev_z, current_x, current_z):
    """이전 위치와 현재 위치를 비교하여 로봇의 방향 추정"""
    dx = current_x - prev_x
    dz = current_z - prev_z
    
    # 이동 거리가 너무 작으면 방향 추정 불가
    if math.sqrt(dx*dx + dz*dz) < 0.001:
        return None
    
    return math.atan2(dx, dz)
```

**설명:**
- 로봇이 어느 방향으로 움직이고 있는지 계산
- 이동 거리가 1mm보다 작으면 방향을 알 수 없음 (오차 범위)

**시각적 설명:**
```
시간 1: 로봇 위치 (0, 0)
         ↓ 이동
시간 2: 로봇 위치 (0.1, 0.1)
         → 방향: 북동쪽 (45°)
```

### 각도 정규화
```python
def normalize_angle(angle):
    """각도를 -π ~ π 범위로 정규화"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
```

**설명:**
- 각도를 -180° ~ 180° 범위로 맞춤
- 예: 270° → -90° (같은 방향이지만 더 짧은 회전)

**실용 예시:**
```python
# 370° → 10° 로 변환
# (한 바퀴 돌고 10°보다 그냥 10° 회전이 빠름)
```

**비유:** "오른쪽으로 270° 돌기"보다 "왼쪽으로 90° 돌기"가 더 빠르죠

## 4단계: 메인 루프

### 상태 머신 개념
```python
STATE_INIT = 0       # 초기화 상태
STATE_NAVIGATE = 1   # 네비게이션 상태
state = STATE_INIT
```

**설명:**
- 로봇은 두 가지 상태를 가짐
- **STATE_INIT**: 처음에 방향을 파악하는 단계
- **STATE_NAVIGATE**: 실제로 목표를 향해 이동하는 단계

**비유:** 게임의 "로딩 중" → "게임 플레이" 같은 단계

### 메인 반복문 시작
```python
while robot.step(timestep) != -1:
    # 현재 위치 읽기
    pos = gps.getValues()
    current_x, current_y, current_z = pos[0], pos[1], pos[2]
```

**설명:**
- `robot.step(timestep)`: 시뮬레이션을 한 스텝 진행 (게임의 프레임과 같음)
- `gps.getValues()`: GPS에서 현재 위치를 읽어옴
- `[x, y, z]` 형태로 3차원 좌표 반환

**비유:** 게임이 초당 60프레임으로 실행되며, 매 프레임마다 캐릭터 위치를 확인

### 시작 위치 저장
```python
if is_first_run:
    start_position = (current_x, current_y, current_z)
    prev_x = current_x
    prev_z = current_z
    is_first_run = False
    print(f"\n📍 시작 위치 저장: X={start_position[0]:.3f}, Z={start_position[2]:.3f}")
```

**설명:**
- 맨 처음에 딱 한 번만 실행됨 (`is_first_run` 플래그 사용)
- 출발점을 기억해둠 (나중에 "얼마나 멀리 왔는지" 계산용)

### 거리 계산
```python
# 1. 시작 위치와 현재 위치 사이의 거리
distance_from_start = calculate_distance(
    start_position[0], start_position[2],
    current_x, current_z
)

# 2. 목표 좌표까지의 거리
distance_to_target = calculate_distance(
    current_x, current_z,
    TARGET_X, TARGET_Z
)
```

**설명:**
- `distance_from_start`: 출발점에서 얼마나 멀리 왔는지
- `distance_to_target`: 목표까지 얼마나 남았는지

**비유:** 
- "서울역에서 100km 왔어요"
- "목적지까지 50km 남았어요"

### 도착 판정
```python
if distance_to_target < ARRIVAL_THRESHOLD:
    print("\n🎉 목표 도착!")
    print(f"최종 위치: X={current_x:.3f}, Z={current_z:.3f}")
    
    # 정지
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    break
```

**설명:**
- 목표와의 거리가 5cm 이내면 도착으로 판단
- 모터 속도를 0으로 설정해서 정지
- `break`: 반복문을 빠져나와 프로그램 종료

## 5단계: 상태별 동작

### STATE_INIT: 초기화 단계
```python
if state == STATE_INIT:
    # 잠시 전진하여 방향 파악
    left_motor.setVelocity(BASE_SPEED)
    right_motor.setVelocity(BASE_SPEED)
    init_counter += 1
    
    print(f"🔄 초기화 중... ({init_counter}/10)")
    
    if init_counter >= 10:
        state = STATE_NAVIGATE
        print("✅ 방향 파악 완료")
```

**설명:**
- 양쪽 바퀴를 같은 속도로 돌려서 직진
- 10프레임(약 0.32초) 동안 전진
- 이동하면서 방향을 파악할 수 있게 됨

**왜 필요한가?**
- 처음에는 로봇이 어디를 향하고 있는지 모름
- 조금 움직여야 "아, 이쪽으로 가고 있구나" 알 수 있음

**비유:** 길을 잃었을 때 일단 한 방향으로 조금 걸어봐야 방향을 알 수 있는 것과 같음

### STATE_NAVIGATE: 네비게이션 단계

#### 방향 계산
```python
elif state == STATE_NAVIGATE:
    robot_heading = get_robot_heading(prev_x, prev_z, current_x, current_z)
    target_angle = get_target_angle(current_x, current_z, TARGET_X, TARGET_Z)
```

**설명:**
- `robot_heading`: 로봇이 현재 향하고 있는 방향
- `target_angle`: 목표를 향한 방향
- 두 각도를 비교해서 얼마나 틀어졌는지 확인

#### 각도 차이 계산
```python
if robot_heading is not None:
    # 회전해야 할 각도
    angle_diff = normalize_angle(target_angle - robot_heading)
```

**시각적 설명:**
```
로봇 방향: → (0°)
목표 방향: ↗ (45°)
각도 차이: 45° (오른쪽으로 45° 틀어야 함)
```

#### 회전 결정
```python
if abs(angle_diff) > math.radians(20):  # 20도 이상 차이
    if angle_diff > 0:
        left_motor.setVelocity(-TURN_SPEED)
        right_motor.setVelocity(TURN_SPEED)
        print("↺ 왼쪽 회전")
    else:
        left_motor.setVelocity(TURN_SPEED)
        right_motor.setVelocity(-TURN_SPEED)
        print("↻ 오른쪽 회전")
```

**설명:**
- 목표 방향과 20° 이상 차이나면 회전
- **왼쪽 회전**: 왼쪽 바퀴는 후진(-), 오른쪽 바퀴는 전진(+)
- **오른쪽 회전**: 왼쪽 바퀴는 전진(+), 오른쪽 바퀴는 후진(-)

**시각적 설명:**
```
왼쪽 회전:
  왼쪽 바퀴 ← (후진)
  오른쪽 바퀴 → (전진)
  결과: ↺ 제자리에서 왼쪽으로 회전

오른쪽 회전:
  왼쪽 바퀴 → (전진)
  오른쪽 바퀴 ← (후진)
  결과: ↻ 제자리에서 오른쪽으로 회전
```

#### 전진 (각도 보정 포함)
```python
else:
    # 방향이 맞으면 전진 (각도 보정 포함)
    correction = angle_diff * 2.0
    
    left_speed = BASE_SPEED - correction
    right_speed = BASE_SPEED + correction
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    print("⬆️ 전진")
```

**설명:**
- 각도 차이가 20° 이내면 전진하면서 방향 조정
- `correction`: 각도 차이를 속도 차이로 변환
- 왼쪽을 더 빠르게/느리게 해서 방향 미세 조정

**예시:**
```python
# 목표가 약간 오른쪽(+5°)에 있을 때
correction = 5 * 2.0 = 10

left_speed = 3.0 - 10 = -7.0 → 제한 후 3.0
right_speed = 3.0 + 10 = 13.0 → 제한 후 3.0

# 실제로는 왼쪽을 조금 느리게, 오른쪽을 조금 빠르게
```

**비유:** 자동차가 커브 길을 갈 때 핸들을 조금씩 조정하는 것

### 이전 위치 업데이트
```python
# 이전 위치 업데이트
prev_x = current_x
prev_z = current_z
```

**설명:**
- 다음 프레임에서 방향을 계산하기 위해 현재 위치를 저장
- 매 프레임마다 업데이트됨

## 전체 흐름도

```
시작
  ↓
GPS, 모터 초기화
  ↓
┌─────────────────┐
│  메인 루프 시작  │ ← 매 프레임마다 반복
└─────────────────┘
  ↓
현재 위치 읽기 (GPS)
  ↓
목표까지 거리 계산
  ↓
도착했나? → YES → 정지 및 종료
  ↓ NO
상태 확인
  ↓
┌─────────────────┐
│  STATE_INIT?    │ → YES → 직진 (10프레임)
└─────────────────┘            ↓
  ↓ NO                    STATE_NAVIGATE로 전환
┌─────────────────┐
│ STATE_NAVIGATE  │
└─────────────────┘
  ↓
로봇 방향 vs 목표 방향 비교
  ↓
각도 차이 20° 이상? → YES → 회전
  ↓ NO
전진 (미세 조정)
  ↓
이전 위치 업데이트
  ↓
(메인 루프로 돌아감)
```

## 실전 예제: 동작 시뮬레이션

```python
# 시작 위치: (0, 0, 0)
# 목표 위치: (0.2, 0.03, 0.2)

# 프레임 1-10: 초기화
"🔄 초기화 중... (1/10)"
"🔄 초기화 중... (2/10)"
...
"✅ 방향 파악 완료, 네비게이션 시작"

# 프레임 11:
"현재: X=0.05, Z=0.01 | 목표거리: 0.26m | 각도차: 25.0° | ↻ 오른쪽 회전"

# 프레임 15:
"현재: X=0.06, Z=0.02 | 목표거리: 0.24m | 각도차: 15.0° | ⬆️ 전진"

# 프레임 50:
"현재: X=0.18, Z=0.18 | 목표거리: 0.05m | 각도차: 5.0° | ⬆️ 전진"

# 프레임 55:
"🎉 목표 도착!"
"최종 위치: X=0.201, Y=0.030, Z=0.199"
```

## 핵심 개념 정리

| 개념 | 설명 | 비유 |
|------|------|------|
| **GPS** | 로봇의 현재 위치 (x, y, z) | 스마트폰 지도 앱 |
| **목표 좌표** | 가고 싶은 위치 | 내비게이션 목적지 |
| **거리 계산** | 두 점 사이의 직선 거리 | 지도에서 자로 재기 |
| **각도 계산** | 어느 방향으로 가야 하는지 | 나침반 확인 |
| **상태 머신** | 로봇의 현재 상태 관리 | 게임의 "로딩"/"플레이" |
| **모터 제어** | 바퀴 속도로 이동/회전 | 자동차 액셀/핸들 |

## 자주 묻는 질문

### Q1: 왜 처음에 초기화가 필요한가요?
**A:** 로봇이 어느 방향을 보고 있는지 모르기 때문이에요. 조금 움직여봐야 "아, 이쪽으로 가고 있구나"를 알 수 있죠.

### Q2: 왜 각도를 정규화하나요?
**A:** 370°와 10°는 같은 방향이지만, 370°로 회전하면 한 바퀴를 더 돌게 돼요. 10°로 바꿔서 효율적으로 회전하기 위해서죠.

### Q3: 왜 20도 이상일 때만 회전하나요?
**A:** 작은 각도 차이는 전진하면서 조정할 수 있어요. 매번 멈춰서 회전하면 비효율적이죠.
