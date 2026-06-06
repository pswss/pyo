# 코드 설명: 벽면 토큰 감지 시스템

이 코드는 **Webots 시뮬레이터**에서 로봇 카메라로 벽에 붙은 토큰(표지판)을 찾아내는 프로그램이에요.

---

## 🔄 전체 흐름

```
카메라 촬영 → 색상 마스크 생성 → 윤곽선 찾기 → 토큰 정보 출력
```

---

## 1. 이미지 변환

```python
# 카메라 원본 (BGRA 4채널) → BGR (3채널)
img = np.frombuffer(image_data, np.uint8).reshape((height, width, 4))
img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

# BGR → HSV 변환
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
```

### HSV 색상 공간

| 요소 | 의미 | 범위 |
|------|------|------|
| **H** (Hue) | 색조 | 0~180 |
| **S** (Saturation) | 채도 | 0~255 |
| **V** (Value) | 명도 | 0~255 |

RGB보다 **특정 색상을 찾기 쉬워서** 컴퓨터 비전에서 자주 써요.

---

## 2. 색상별 마스크 생성

### 흰색 감지
```python
lower_white = np.array([0, 0, 180])    # 채도 낮고 밝은 것
upper_white = np.array([180, 50, 255])
mask_white = cv2.inRange(hsv, lower_white, upper_white)
```

### 빨간색 감지
```python
# 빨간색은 H값이 0 근처와 180 근처 두 곳에 있음
mask_red1 = cv2.inRange(hsv, [0, 100, 100], [10, 255, 255])
mask_red2 = cv2.inRange(hsv, [170, 100, 100], [180, 255, 255])
mask_red = cv2.bitwise_or(mask_red1, mask_red2)
```

HSV 색상환:
```
      노랑(30)
        |
초록(60) --- 빨강(0/180) ← 경계에 걸쳐있음!
        |
      파랑(120)
```

### 분홍색 감지
```python
mask_pink = cv2.inRange(hsv, [140, 50, 50], [170, 255, 255])
```

### 마스크 합치기
```python
mask = cv2.bitwise_or(mask_red, mask_white)
mask = cv2.bitwise_or(mask, mask_pink)
```

→ **흰색 OR 빨간색 OR 분홍색** 영역만 남은 이진 이미지

---

## 3. 토큰 필터링

```python
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for c in contours:
    area = cv2.contourArea(c)
    
    if 200 < area < 2000:                    # 조건 1: 적절한 크기
        x, y, w, h = cv2.boundingRect(c)
        aspect_ratio = float(w) / h
        
        if 0.8 <= aspect_ratio <= 1.2:       # 조건 2: 정사각형
            # 토큰으로 인정!
```

| 조건 | 이유 |
|------|------|
| 면적 200~2000 | 너무 작으면 노이즈, 너무 크면 벽 전체 |
| 비율 0.8~1.2 | 토큰은 대부분 정사각형 |

---

## 4. 메인 루프

```python
while robot.step(timeStep) != -1:
    img = camera.getImage()
    
    if img:
        tokens = detect_wall_tokens(img, camera)
        
        if tokens:
            print(f"🎯 토큰 {len(tokens)}개 발견!")
            for i, token in enumerate(tokens):
                print(f"  위치: ({token['x']}, {token['y']})")
```

---

## 📊 요약

| 구성요소 | 역할 |
|----------|------|
| HSV 변환 | 색상 감지 쉽게 |
| 색상 마스크 | 흰색/빨강/분홍 영역 추출 |
| 윤곽선 분석 | 토큰 위치와 크기 파악 |
| 필터링 | 노이즈 제거 (크기, 비율) |

이 코드는 **토큰 감지만** 담당하고, 로봇 이동이나 토큰 분류 기능은 없는 기본형이에요! 🤖