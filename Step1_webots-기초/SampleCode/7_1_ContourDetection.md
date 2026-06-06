## 전체 개요

이 코드는 **카메라로 찍은 이미지에서 물체의 윤곽선(테두리)을 찾아내는 프로그램**이에요. 마치 색칠공부 책처럼 물체의 외곽선을 그려주는 거죠!

## 1단계: 초기 설정

### 기본 준비
```python
from controller import Robot
import cv2
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())
```

**설명:**
- `cv2`: OpenCV 라이브러리 (이미지 처리 전문 도구)
- `numpy`: 배열(이미지 데이터) 처리 도구
- `Robot()`: Webots 로봇 생성

**비유:** 그림을 그리기 위한 붓과 물감을 준비하는 단계

### 카메라 활성화
```python
camera = robot.getDevice('camera_centre')
camera.enable(timestep)

width = camera.getWidth()
height = camera.getHeight()
```

**설명:**
- 카메라를 켜고 이미지 크기 정보를 가져옴
- 예: 64x64 픽셀, 128x128 픽셀 등

## 2단계: 이미지 가져오기 및 변환

### 카메라에서 이미지 읽기
```python
while robot.step(timestep) != -1:
    image = camera.getImage()
    
    if image:
        # 이미지 변환
        img = np.frombuffer(image, np.uint8)
        img = img.reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
```

**단계별 설명:**

#### 1) 원시 데이터 가져오기
```python
image = camera.getImage()
# 결과: [255, 120, 80, 255, 100, 200, ...] (1차원 숫자 나열)
```

#### 2) numpy 배열로 변환
```python
img = np.frombuffer(image, np.uint8)
# 결과: numpy 배열로 변환 (아직 1차원)
```

#### 3) 3차원으로 재구성
```python
img = img.reshape((height, width, 4))
# 결과: (높이, 너비, 4채널) 형태의 3차원 이미지
# 4채널 = Blue, Green, Red, Alpha
```

#### 4) BGR 형식으로 변환
```python
img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
# 결과: Alpha 채널 제거, BGR 3채널만 남김
```

**비유:** 
- 1) 퍼즐 조각들을 상자에서 꺼냄
- 2) 조각들을 테이블에 펼침
- 3) 조각들을 올바른 위치에 배치
- 4) 완성된 그림 확인

## 3단계: 윤곽선 검출 전처리

### 그레이스케일 변환
```python
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
```

**설명:**
- 컬러 이미지를 흑백으로 변환
- RGB(3채널) → Gray(1채널)
- 각 픽셀이 0(검정) ~ 255(흰색)의 값을 가짐

**시각적 비교:**
```
원본 (컬러):              그레이스케일:
🔴 빨간 사과      →      ⚫ 어두운 회색
🟢 초록 잎       →      ⚪ 밝은 회색
```

**왜 필요한가?**
- 윤곽선은 밝기 차이로 구분되기 때문
- 색상 정보는 필요 없고 명암만 있으면 됨
- 처리 속도가 3배 빨라짐 (3채널 → 1채널)

**비유:** 색칠공부 책에서 선만 필요하지 색은 필요 없는 것과 같음

### 가우시안 블러 (흐림 효과)
```python
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
```

**설명:**
- `(5, 5)`: 5x5 크기의 필터 사용
- `0`: 자동으로 표준편차 계산
- 이미지를 부드럽게 만들어 노이즈 제거

**시각적 효과:**
```
원본:                    블러 적용 후:
█░█░█░█░█      →        ████████
░█░█░█░█░               ████████
█░█░█░█░█               ████████
(거친 점들)              (부드러운 면)
```

**왜 필요한가?**
- 작은 점, 얼룩 같은 노이즈 제거
- 윤곽선을 더 깔끔하게 검출

**실전 예제:**
```python
# 블러 없이
노이즈가 많은 이미지 → 윤곽선 1000개 검출 (대부분 쓸모없음)

# 블러 적용
깨끗한 이미지 → 윤곽선 5개 검출 (정확한 물체만)
```

**비유:** 사진을 찍기 전에 렌즈를 닦는 것

### 이진화 (Binary)
```python
_, binary = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
```

**설명:**
- `127`: 임계값 (기준점)
- `255`: 최대값 (흰색)
- `THRESH_BINARY`: 이진화 방식
- 픽셀값이 127보다 크면 255(흰색), 작으면 0(검정)

**시각적 효과:**
```
그레이스케일:           이진화:
50  100 150 200   →    0   0  255 255
80  130 170 220   →    0  255 255 255
120 160 190 240   →    0  255 255 255
(다양한 회색)          (검정 또는 흰색만)
```

**단계별 변환:**
```python
# 원본 픽셀값들
[10, 50, 80, 127, 150, 200, 255]

# 임계값 127 적용 후
[0,  0,  0,  0,  255, 255, 255]
     ↑ 127보다 작음        ↑ 127보다 큼
```

**왜 필요한가?**
- 물체와 배경을 명확히 구분
- 윤곽선 검출이 훨씬 정확해짐

**비유:** 흐릿한 사진을 선명한 흑백 스케치로 변환

## 4단계: 윤곽선 검출

### 윤곽선 찾기
```python
contours, _ = cv2.findContours(
    binary,
    cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE
)
```

**매개변수 설명:**

#### 1) `binary`: 입력 이미지
- 이진화된 이미지 (흑백만)

#### 2) `cv2.RETR_EXTERNAL`: 검색 모드
```python
# 여러 옵션:
cv2.RETR_EXTERNAL  # 가장 바깥쪽 윤곽선만
cv2.RETR_LIST      # 모든 윤곽선 (평면적)
cv2.RETR_TREE      # 계층 구조로
```

**시각적 비교:**
```
RETR_EXTERNAL:          RETR_LIST:
  ┌────┐                 ┌────┐
  │ ┌─┐│                 │ ┌─┐│
  │ └─┘│   →  외곽만      │ └─┘│   →  모두
  └────┘                 └────┘
    ↑                      ↑ ↑
  이것만 검출          바깥+안쪽 모두
```

#### 3) `cv2.CHAIN_APPROX_SIMPLE`: 근사 방법
```python
cv2.CHAIN_APPROX_NONE    # 모든 점 저장
cv2.CHAIN_APPROX_SIMPLE  # 불필요한 점 제거 (추천)
```

**시각적 비교:**
```
APPROX_NONE:            APPROX_SIMPLE:
●●●●●●●●               ●       ●
●      ●               ●       ●
●      ●      →        
●      ●               ●       ●
●●●●●●●●               ●       ●
(모든 점)              (꼭짓점만)
```

**반환값:**
```python
contours  # 윤곽선 리스트 (각 윤곽선은 점들의 배열)
_         # 계층 정보 (사용 안 함)
```

### 윤곽선 개수 출력
```python
print(f'윤곽선 개수: {len(contours)}')
```

**예시 출력:**
```
윤곽선 개수: 3
# → 이미지에 3개의 물체가 있음
```

## 5단계: 결과 시각화

### 윤곽선 그리기
```python
result = img.copy()
cv2.drawContours(result, contours, -1, (0, 255, 0), 2)
```

**매개변수 설명:**
```python
cv2.drawContours(
    result,     # 그림을 그릴 이미지
    contours,   # 그릴 윤곽선들
    -1,         # 모든 윤곽선 (-1), 또는 특정 인덱스
    (0,255,0),  # 색상 (BGR): 초록색
    2           # 선 두께 (픽셀)
)
```

**색상 예시:**
```python
(0, 255, 0)    # 초록색
(255, 0, 0)    # 파란색
(0, 0, 255)    # 빨간색
(255, 255, 0)  # 청록색
(0, 255, 255)  # 노란색
```

**두께 예시:**
```python
thickness = 1   # 얇은 선 ─
thickness = 2   # 중간 선 ━
thickness = 5   # 굵은 선 ━
thickness = -1  # 채우기 ■
```

### 이미지 저장
```python
cv2.imwrite('contours.png', result)
break
```

**설명:**
- 윤곽선이 그려진 이미지를 'contours.png'로 저장
- `break`: 한 장만 처리하고 종료

## 전체 처리 과정 시각화

```
1. 원본 이미지 (컬러)
   🔴🟢🔵
   ┌──────┐
   │  🍎  │  빨간 사과
   └──────┘

     ↓ 그레이스케일 변환

2. 흑백 이미지
   ⚫⚪⚫
   ┌──────┐
   │  ⚫  │  회색 사과
   └──────┘

     ↓ 가우시안 블러

3. 블러 적용
   ░░░░░░
   ┌──────┐
   │  ░   │  부드러운 사과
   └──────┘

     ↓ 이진화

4. 이진 이미지
   ██████
   ┌──────┐
   │  ██  │  검정/흰색만
   └──────┘

     ↓ 윤곽선 검출

5. 윤곽선 그리기
   
   ┌──────┐
   │  🟢  │  초록 테두리
   └──────┘
```

## 실전 예제 코드

### 예제 1: 여러 색상으로 윤곽선 그리기
```python
from controller import Robot
import cv2
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice('camera_centre')
camera.enable(timestep)

width = camera.getWidth()
height = camera.getHeight()

while robot.step(timestep) != -1:
    image = camera.getImage()
    
    if image:
        img = np.frombuffer(image, np.uint8)
        img = img.reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        
        # 전처리
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
        
        # 윤곽선 검출
        contours, _ = cv2.findContours(
            binary,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        print(f'윤곽선 개수: {len(contours)}')
        
        # 각 윤곽선을 다른 색으로 그리기
        result = img.copy()
        colors = [
            (0, 255, 0),    # 초록
            (255, 0, 0),    # 파랑
            (0, 0, 255),    # 빨강
            (255, 255, 0),  # 청록
            (0, 255, 255)   # 노랑
        ]
        
        for i, contour in enumerate(contours):
            color = colors[i % len(colors)]  # 색상 순환
            cv2.drawContours(result, [contour], -1, color, 2)
            
            # 윤곽선 번호 표시
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.putText(result, str(i+1), (cx, cy),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        cv2.imwrite('colored_contours.png', result)
        break
```

### 예제 2: 면적으로 필터링
```python
# 윤곽선 검출 후
print(f'전체 윤곽선: {len(contours)}개')

# 면적 100 이상인 것만 선택
large_contours = []
for contour in contours:
    area = cv2.contourArea(contour)
    if area > 100:  # 100 픽셀 이상
        large_contours.append(contour)
        print(f'  - 면적: {area:.0f} 픽셀')

print(f'큰 윤곽선: {len(large_contours)}개')

# 큰 것만 그리기
result = img.copy()
cv2.drawContours(result, large_contours, -1, (0, 255, 0), 2)
```

### 예제 3: 윤곽선 정보 상세 출력
```python
for i, contour in enumerate(contours):
    # 면적
    area = cv2.contourArea(contour)
    
    # 둘레
    perimeter = cv2.arcLength(contour, True)
    
    # 중심점
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = 0, 0
    
    # 외접 사각형
    x, y, w, h = cv2.boundingRect(contour)
    
    print(f'\n윤곽선 {i+1}:')
    print(f'  면적: {area:.0f} 픽셀')
    print(f'  둘레: {perimeter:.1f} 픽셀')
    print(f'  중심: ({cx}, {cy})')
    print(f'  크기: {w}x{h}')
```

## 핵심 개념 정리

| 단계 | 목적 | 결과 |
|------|------|------|
| **그레이스케일** | 색상 제거 | 흑백 이미지 |
| **가우시안 블러** | 노이즈 제거 | 부드러운 이미지 |
| **이진화** | 명확한 구분 | 검정/흰색만 |
| **윤곽선 검출** | 테두리 찾기 | 윤곽선 좌표들 |
| **그리기** | 시각화 | 결과 이미지 |

## 매개변수 조정 가이드

### 블러 크기 조정
```python
# 작은 블러 (섬세함)
blurred = cv2.GaussianBlur(gray, (3, 3), 0)

# 중간 블러 (균형) - 추천
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# 큰 블러 (부드러움)
blurred = cv2.GaussianBlur(gray, (9, 9), 0)
```

### 임계값 조정
```python
# 어두운 물체 검출
_, binary = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY)

# 일반적인 경우
_, binary = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

# 밝은 물체 검출
_, binary = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)
```

### 자동 임계값 (Otsu)
```python
# 자동으로 최적 임계값 찾기
_, binary = cv2.threshold(blurred, 0, 255, 
                          cv2.THRESH_BINARY + cv2.THRESH_OTSU)
```

## 자주 묻는 질문

**Q1: 왜 그레이스케일로 변환하나요?**
**A:** 윤곽선은 밝기 차이로 구분되기 때문에 색상 정보는 필요 없어요. 처리 속도도 3배 빨라지죠.

**Q2: 블러를 왜 적용하나요?**
**A:** 작은 점이나 얼룩 같은 노이즈를 제거해서 정확한 윤곽선만 찾기 위해서예요.

**Q3: 임계값 127은 왜 사용하나요?**
**A:** 0~255 범위의 중간값이에요. 상황에 따라 조정할 수 있어요.

**Q4: RETR_EXTERNAL vs RETR_LIST 차이는?**
**A:** EXTERNAL은 가장 바깥쪽만, LIST는 안쪽 윤곽선까지 모두 찾아요.
