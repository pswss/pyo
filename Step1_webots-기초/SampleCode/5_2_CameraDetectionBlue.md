## 전체 개요

이 코드는 **카메라로 파란색 물체를 실시간으로 찾아내고, 그 위치를 추적하는 프로그램**이에요. 로봇이 파란색 공이나 표지판을 찾아 따라갈 수 있게 만드는 기초 코드죠!

## 1단계: 초기 설정

### 기본 준비
```python
from controller import Robot
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())
```

**필요한 도구들:**
- `Robot`: Webots 로봇 제어
- `numpy`: 배열(이미지 데이터) 처리
- `cv2`: OpenCV (이미지 처리 라이브러리)

### 카메라 초기화
```python
camera = robot.getDevice('camera_centre')
camera.enable(timestep)
width = camera.getWidth()
height = camera.getHeight()
```

**설명:**
- 'camera_centre'라는 이름의 카메라를 찾아서 활성화
- `width`, `height`: 이미지 크기 (예: 64x64 픽셀)

**비유:** 스마트폰 카메라 앱을 켜는 것

## 2단계: 파란색 감지 함수

### detect_blue 함수
```python
def detect_blue(img):
    """파란색 감지 함수"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
```

#### HSV로 변환하는 이유
```python
# BGR (Blue, Green, Red)
# - 색상을 빨강, 초록, 파랑의 조합으로 표현
# - 조명에 민감함

# HSV (Hue, Saturation, Value)
# - H: 색조 (무슨 색인가?)
# - S: 채도 (얼마나 선명한가?)
# - V: 명도 (얼마나 밝은가?)
# - 조명 변화에 강함
```

**시각적 비교:**
```
BGR로 파란색 찾기:
밝은 파랑: (255, 100, 100)
어두운 파랑: (100, 50, 50)
→ 범위가 너무 다양해서 찾기 어려움

HSV로 파란색 찾기:
밝은 파랑: (110, 200, 255)
어두운 파랑: (110, 200, 100)
→ H(색조)는 같고 V(밝기)만 다름 (찾기 쉬움!)
```

### 파란색 범위 설정

코드에 3가지 범위 옵션이 주석으로 있어요:

#### 옵션 1: 일반적인 파란색
```python
lower_blue = np.array([100, 100, 100])
upper_blue = np.array([130, 255, 255])
```
- **사용 시기**: 모든 종류의 파란색 감지
- **장점**: 넓은 범위 커버
- **단점**: 하늘색까지 감지될 수 있음

#### 옵션 2: 짙은 파란색
```python
lower_blue = np.array([100, 150, 50])
upper_blue = np.array([130, 255, 200])
```
- **사용 시기**: 하늘색 제외, 진한 파란색만
- **장점**: 하늘색 필터링
- **단점**: 너무 밝거나 어두운 파란색 놓칠 수 있음

#### 옵션 3: 선명하고 밝은 파란색 (현재 사용 중)
```python
lower_blue = np.array([110, 200, 180])  # 선명하고 밝은 파란색
upper_blue = np.array([125, 255, 255])
```
- **사용 시기**: 특정한 밝은 파란색 타겟
- **장점**: 매우 정확한 감지
- **단점**: 범위가 좁아서 다른 파란색 못 찾을 수 있음

### HSV 값 이해하기

```python
lower_blue = np.array([110, 200, 180])
                       ↓    ↓    ↓
                       H    S    V
```

| 요소 | 범위 | 의미 | 예시 |
|------|------|------|------|
| **H (색조)** | 110~125 | 파란색 영역 | 100=청록, 120=파랑, 130=남색 |
| **S (채도)** | 200~255 | 매우 선명 | 0=회색, 255=순수한 색 |
| **V (명도)** | 180~255 | 밝음 | 0=검정, 255=흰색 |

**시각적 설명:**
```
H (색조) - 색상환:
  0° = 빨강
  60° = 노랑
  120° = 초록
  180° = 청록
  240° = 파랑 ← 우리가 찾는 범위!
  300° = 보라

S (채도):
  0 = 회색 ░░░
  128 = 중간 ▒▒▒
  255 = 순수한 색 ███

V (명도):
  0 = 검정 ■
  128 = 중간 ▒
  255 = 밝음 □
```

### 마스크 생성
```python
mask = cv2.inRange(hsv, lower_blue, upper_blue)
return mask
```

**설명:**
- `cv2.inRange`: 범위 안에 있는 픽셀만 선택
- 범위 안 → 255 (흰색)
- 범위 밖 → 0 (검정)

**시각적 결과:**
```
원본 이미지:          마스크:
🟦🟦⬜⬜           ██  (파란색 부분만 흰색)
🟦🟩⬜⬜    →     █   (나머지는 검정)
⬜⬜⬜⬜           
```

## 3단계: 파란색 객체 찾기

### find_blue_object 함수
```python
def find_blue_object(mask):
    """파란색 객체의 중심과 크기 찾기"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                   cv2.CHAIN_APPROX_SIMPLE)
```

**설명:**
- `findContours`: 마스크에서 윤곽선(테두리) 찾기
- `RETR_EXTERNAL`: 가장 바깥쪽 윤곽선만
- `CHAIN_APPROX_SIMPLE`: 불필요한 점 제거

**예시:**
```python
# 마스크에 3개의 파란색 영역이 있으면
contours = [윤곽선1, 윤곽선2, 윤곽선3]
```

### 가장 큰 객체 선택
```python
if contours:
    # 가장 큰 파란색 객체 선택
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
```

**설명:**
- `cv2.contourArea`: 윤곽선의 면적 계산
- `max(..., key=...)`: 면적이 가장 큰 것 선택

**실전 예시:**
```python
# 3개의 파란색 객체 발견
윤곽선1: 면적 = 50 픽셀   (작은 점)
윤곽선2: 면적 = 500 픽셀  (큰 물체) ← 선택됨!
윤곽선3: 면적 = 100 픽셀  (중간 크기)
```

**왜 가장 큰 것을 선택?**
- 작은 노이즈 무시
- 주요 타겟만 추적
- 여러 개 있어도 가장 중요한 것 선택

### 중심점 계산
```python
# 중심점 계산
M = cv2.moments(largest)
if M["m00"] != 0:
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy), area

return None, 0
```

**모멘트(Moment)란?**
- 도형의 무게중심을 찾는 수학적 방법
- `M["m10"]`, `M["m01"]`: x, y 방향 1차 모멘트
- `M["m00"]`: 면적 (0차 모멘트)

**공식:**
```python
중심 x = M["m10"] / M["m00"]
중심 y = M["m01"] / M["m00"]
```

**시각적 설명:**
```
파란색 객체:
  ■■■
  ■■■  → 중심점: (cx, cy)
  ■■■        ↓
            ●
```

**실전 예시:**
```python
# L자 모양 객체
■■
■
■

# 무게중심은 꼭짓점이 아닌 중간 어딘가
    ●
```

## 4단계: 메인 루프

### 이미지 변환
```python
frame_count = 0
while robot.step(timestep) != -1:
    image = camera.getImage()
    
    if image:
        # 이미지 변환
        img = np.frombuffer(image, np.uint8)
        img = img.reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
```

**단계별 변환:**
```
1. camera.getImage()
   → [255, 100, 80, 255, ...] (1차원 배열)

2. np.frombuffer()
   → numpy 배열로 변환

3. reshape((height, width, 4))
   → (64, 64, 4) 형태의 3차원 배열
   → BGRA (Blue, Green, Red, Alpha)

4. cvtColor(COLOR_BGRA2BGR)
   → (64, 64, 3) 형태
   → BGR (Alpha 채널 제거)
```

### 파란색 감지 및 위치 판단
```python
# 파란색 감지
blue_mask = detect_blue(img)
center, area = find_blue_object(blue_mask)

if center:
    cx, cy = center
    print(f'프레임 {frame_count}: 파란색 발견! 위치=({cx}, {cy}), 크기={area}')
    
    # 화면 중앙 기준으로 방향 판단
    center_x = width // 2
    if cx < center_x - 20:
        print('  → 왼쪽에 있음')
    elif cx > center_x + 20:
        print('  → 오른쪽에 있음')
    else:
        print('  → 중앙에 있음')
```

**방향 판단 로직:**
```python
width = 64  # 카메라 너비
center_x = 64 // 2 = 32  # 화면 중앙

# 파란색 객체의 x 좌표(cx)에 따라:
cx < 12  →  왼쪽 (32 - 20)
12 ≤ cx ≤ 52  →  중앙
cx > 52  →  오른쪽 (32 + 20)
```

**시각적 설명:**
```
카메라 화면 (64 픽셀 너비):
┌────────────────────────────────┐
│        │         │         │    │
│  왼쪽   │   중앙   │   오른쪽  │
│        │         │         │    │
│   0    12    32    52      64  │
└────────────────────────────────┘
         ↑    ↑    ↑
      -20   중심  +20
```

**실전 예시:**
```python
# 프레임 1: 파란색 발견! 위치=(10, 30), 크기=450
#   → 왼쪽에 있음

# 프레임 2: 파란색 발견! 위치=(32, 28), 크기=520
#   → 중앙에 있음

# 프레임 3: 파란색 발견! 위치=(55, 35), 크기=480
#   → 오른쪽에 있음
```

### 객체 없을 때 처리
```python
else:
    print(f'프레임 {frame_count}: 파란색 객체 없음')
```

**발생 상황:**
- 파란색이 카메라 시야에 없음
- 조명 변화로 색상 범위 벗어남
- 너무 작아서 감지 안 됨

### 결과 저장
```python
frame_count += 1

# 100프레임마다 결과 이미지 저장
if frame_count % 100 == 0:
    cv2.imwrite(f'blue_detection_{frame_count}.png', blue_mask)
```

**설명:**
- `frame_count % 100 == 0`: 100의 배수일 때만
- 프레임 100, 200, 300... 에서 마스크 이미지 저장
- 파일명: `blue_detection_100.png`, `blue_detection_200.png` 등

**비유:** 게임에서 100점마다 스크린샷 저장

## 전체 동작 흐름도

```
시작
  ↓
카메라 초기화
  ↓
┌─────────────────┐
│   메인 루프      │ ← 매 프레임 반복
└─────────────────┘
  ↓
이미지 가져오기 (BGRA)
  ↓
BGR로 변환
  ↓
┌─────────────────┐
│ detect_blue()   │ 파란색 감지
└─────────────────┘
  ↓
BGR → HSV 변환
  ↓
색상 범위로 마스크 생성
  ↓
┌─────────────────┐
│find_blue_object│ 객체 찾기
└─────────────────┘
  ↓
윤곽선 검출
  ↓
가장 큰 것 선택
  ↓
중심점 계산
  ↓
위치 판단 (왼쪽/중앙/오른쪽)
  ↓
결과 출력
  ↓
100프레임마다 저장
  ↓
(메인 루프로 돌아감)
```

## 실전 활용 예제

### 예제 1: 로봇 제어와 연결
```python
from controller import Robot
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 카메라 초기화
camera = robot.getDevice('camera_centre')
camera.enable(timestep)
width = camera.getWidth()
height = camera.getHeight()

# 모터 초기화
left_motor = robot.getDevice('wheel1 motor')
right_motor = robot.getDevice('wheel2 motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

BASE_SPEED = 2.0

def detect_blue(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([110, 200, 180])
    upper_blue = np.array([125, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    return mask

def find_blue_object(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                   cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy), area
    return None, 0

# 메인 루프
while robot.step(timestep) != -1:
    image = camera.getImage()
    
    if image:
        img = np.frombuffer(image, np.uint8)
        img = img.reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        
        blue_mask = detect_blue(img)
        center, area = find_blue_object(blue_mask)
        
        if center:
            cx, cy = center
            center_x = width // 2
            
            # 파란색 위치에 따라 로봇 제어
            if cx < center_x - 20:
                # 왼쪽에 있으면 왼쪽으로 회전
                left_motor.setVelocity(BASE_SPEED * 0.5)
                right_motor.setVelocity(BASE_SPEED)
                print('↺ 왼쪽 회전')
            elif cx > center_x + 20:
                # 오른쪽에 있으면 오른쪽으로 회전
                left_motor.setVelocity(BASE_SPEED)
                right_motor.setVelocity(BASE_SPEED * 0.5)
                print('↻ 오른쪽 회전')
            else:
                # 중앙에 있으면 직진
                left_motor.setVelocity(BASE_SPEED)
                right_motor.setVelocity(BASE_SPEED)
                print('⬆️ 직진')
        else:
            # 파란색이 없으면 제자리 회전
            left_motor.setVelocity(-BASE_SPEED * 0.5)
            right_motor.setVelocity(BASE_SPEED * 0.5)
            print('🔍 탐색 중')
```

### 예제 2: 여러 색상 동시 감지
```python
def detect_colors(img):
    """여러 색상 동시 감지"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # 색상별 범위
    colors = {
        'blue': ([110, 200, 180], [125, 255, 255]),
        'red': ([0, 200, 180], [10, 255, 255]),
        'green': ([50, 200, 180], [70, 255, 255])
    }
    
    results = {}
    
    for color_name, (lower, upper) in colors.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        center, area = find_blue_object(mask)  # 같은 함수 재사용
        
        if center:
            results[color_name] = {
                'center': center,
                'area': area
            }
    
    return results

# 사용 예
detected_colors = detect_colors(img)

if 'blue' in detected_colors:
    print(f"파란색: {detected_colors['blue']['center']}")
if 'red' in detected_colors:
    print(f"빨간색: {detected_colors['red']['center']}")
```

### 예제 3: 크기에 따른 거리 추정
```python
def estimate_distance(area):
    """면적으로 거리 추정 (대략적)"""
    if area > 1000:
        return "매우 가까움 (<0.5m)"
    elif area > 500:
        return "가까움 (0.5-1m)"
    elif area > 200:
        return "중간 (1-2m)"
    elif area > 50:
        return "멀음 (2-3m)"
    else:
        return "매우 멀음 (>3m)"

# 사용 예
if center:
    cx, cy = center
    distance = estimate_distance(area)
    print(f'위치: ({cx}, {cy}), 거리: {distance}')
```

## 핵심 개념 정리

| 단계 | 목적 | 입력 | 출력 |
|------|------|------|------|
| **BGR→HSV** | 색상 분리 | 컬러 이미지 | HSV 이미지 |
| **inRange** | 색상 필터링 | HSV + 범위 | 마스크 |
| **findContours** | 윤곽선 찾기 | 마스크 | 윤곽선 리스트 |
| **moments** | 중심 계산 | 윤곽선 | 중심 좌표 |
| **위치 판단** | 방향 결정 | 중심 x좌표 | 왼쪽/중앙/오른쪽 |

## 매개변수 조정 가이드

### HSV 범위 조정

**너무 많이 감지될 때:**
```python
# 채도(S)와 명도(V) 최솟값 높이기
lower_blue = np.array([110, 220, 200])  # S, V 증가
upper_blue = np.array([125, 255, 255])
```

**너무 적게 감지될 때:**
```python
# 범위 넓히기
lower_blue = np.array([100, 150, 150])  # H, S, V 감소
upper_blue = np.array([130, 255, 255])  # H 증가
```

### 위치 판단 민감도 조정

**더 민감하게:**
```python
# ±10 픽셀 범위
if cx < center_x - 10:  # 더 작은 범위
    print('왼쪽')
elif cx > center_x + 10:
    print('오른쪽')
```

**덜 민감하게:**
```python
# ±30 픽셀 범위
if cx < center_x - 30:  # 더 큰 범위
    print('왼쪽')
elif cx > center_x + 30:
    print('오른쪽')
```

## 자주 묻는 질문

**Q1: 왜 BGR을 HSV로 변환하나요?**
**A:** HSV는 색상(H)이 분리되어 있어서 조명이 변해도 같은 색을 쉽게 찾을 수 있어요.

**Q2: 왜 가장 큰 객체만 선택하나요?**
**A:** 작은 노이즈를 무시하고 주요 타겟만 추적하기 위해서예요.

**Q3: moment는 무엇인가요?**
**A:** 도형의 무게중심을 찾는 수학적 방법이에요. 물리학의 질량중심과 비슷한 개념이죠.

**Q4: 왜 100프레임마다 저장하나요?**
**A:** 매 프레임 저장하면 파일이 너무 많아져요. 주기적으로 샘플링해서 저장하는 거죠.
