# 5회차 — NumPy 배열 · OpenCV 기초

> 목표: NumPy 2D 배열로 지도를 표현하고 조작할 수 있다.
>       OpenCV 로 배열에 도형을 그리고 창에 표시할 수 있다.
>       `map_visualizer.py` 의 동작 원리를 이해한다.
> 실습 파일: `05_practice.py`
> 필요: `pip install numpy opencv-python`

---

## 1. NumPy 배열 — 지도의 기반

### 배열이란?

파이썬 리스트와 비슷하지만 훨씬 빠르고 수학 연산에 특화되어 있습니다.

```python
import numpy as np

# 1D 배열
a = np.array([1, 2, 3, 4, 5])
print(a)          # [1 2 3 4 5]
print(a.shape)    # (5,)

# 2D 배열 (행렬 / 그리드)
grid = np.array([[0, 1, 0],
                 [1, 0, 1],
                 [0, 0, 1]])
print(grid.shape)   # (3, 3)  → 3행 3열
```

### 특정 값으로 채운 배열 만들기

```python
# 전부 0
zeros = np.zeros((100, 100))
print(zeros.shape)   # (100, 100)

# 전부 특정 값
wall_grid = np.full((200, 200), False, dtype=bool)   # 전부 False (벽 없음)

# 이미지 (가로×세로×색상채널)
image = np.zeros((300, 300, 3), dtype=np.uint8)       # 검정 이미지
image = np.full((300, 300, 3), 50, dtype=np.uint8)    # 어두운 회색 이미지
```

### 인덱싱

```python
grid = np.zeros((5, 5))

# 단일 원소
print(grid[2, 3])         # 2행 3열
grid[2, 3] = 1.0          # 값 설정

# 슬라이싱
print(grid[0:3, :])       # 0~2행 전체
print(grid[:, 1])         # 1열 전체
```

### 불리언 마스킹 — 소스 코드의 핵심 기법

```python
# 조건에 맞는 곳을 한 번에 변경
image = np.zeros((100, 100, 3), dtype=np.uint8)

wall_layer = np.zeros((100, 100), dtype=bool)
wall_layer[20:30, 10:80] = True   # 직사각형 영역을 벽으로

# 벽인 픽셀 전체를 흰색으로
image[wall_layer] = (220, 220, 220)
```

소스 코드 (`src/map_visualizer.py`) 와 비교:
```python
# 발견된 영역을 밝은 회색으로
image[arrays["discovered"]] = _COLORS["discovered_bg"]

# 벽을 흰색으로
image[arrays["occupied"]] = _COLORS["wall"]

# 로봇이 지나간 경로를 갈색으로
image[arrays["traversed"]] = _COLORS["traversed"]
```

모두 **불리언 레이어를 인덱스로 써서 해당 픽셀의 색상을 한 번에 바꾸는** 패턴입니다.

### argwhere — True 인 위치 찾기

```python
layer = np.zeros((5, 5), dtype=bool)
layer[1, 2] = True
layer[3, 4] = True

positions = np.argwhere(layer)
print(positions)
# [[1 2]
#  [3 4]]

# 소스에서:
victim_positions = np.argwhere(arrays["victims"])
for pos in victim_positions:
    cv.circle(image, (pos[1], pos[0]), 5, color, -1)
#                     ↑col    ↑row     ← OpenCV 는 (x=col, y=row) 순서!
```

---

## 2. BGR vs RGB — OpenCV 의 색상 순서

OpenCV 는 색상을 **BGR (Blue-Green-Red)** 순서로 씁니다.
화면에서 보이는 색과 BGR 값의 관계:

| 색상 | RGB | BGR (OpenCV) |
|------|-----|--------------|
| 빨강 | (255,0,0) | **(0,0,255)** |
| 초록 | (0,255,0) | **(0,255,0)** |
| 파랑 | (0,0,255) | **(255,0,0)** |
| 노랑 | (255,255,0) | **(0,255,255)** |
| 주황 | (255,165,0) | **(0,165,255)** |
| 흰색 | (255,255,255) | **(255,255,255)** |
| 검정 | (0,0,0) | **(0,0,0)** |

---

## 3. OpenCV 도형 그리기

```python
import numpy as np
import cv2 as cv

# 600×600 검정 캔버스
canvas = np.zeros((600, 600, 3), dtype=np.uint8)

# 원: cv.circle(이미지, 중심, 반경, 색상, 두께)
#   두께=-1 이면 채움
cv.circle(canvas, (300, 300), 30, (0, 0, 255), -1)   # 빨간 원 (채움)
cv.circle(canvas, (300, 300), 30, (255, 255, 255), 2) # 흰색 테두리

# 선: cv.line(이미지, 시작점, 끝점, 색상, 두께)
cv.line(canvas, (0, 0), (600, 600), (0, 255, 0), 1)

# 화살표: cv.arrowedLine(이미지, 시작, 끝, 색상, 두께, tipLength=화살촉비율)
cv.arrowedLine(canvas, (300, 300), (350, 250), (0, 0, 255), 2, tipLength=0.4)

# 특수 마커: cv.drawMarker(이미지, 위치, 색상, 마커종류, 크기, 두께)
cv.drawMarker(canvas, (100, 100), (255, 0, 255),
              cv.MARKER_STAR, 12, 1)

# 창에 표시
cv.imshow("테스트", canvas)
cv.waitKey(0)         # 키 입력 대기
cv.destroyAllWindows()
```

### 크기 조정

```python
small_map = np.zeros((100, 100, 3), dtype=np.uint8)
# 100×100 → 600×600 으로 확대 (INTER_NEAREST: 픽셀 경계 선명)
display = cv.resize(small_map, (600, 600), interpolation=cv.INTER_NEAREST)
```

소스 코드 (`map_visualizer.py`):
```python
display = cv.resize(image, (_DISPLAY_SIZE, _DISPLAY_SIZE),
                    interpolation=cv.INTER_NEAREST)
cv.imshow(_WINDOW_NAME, display)
cv.waitKey(1)    # 1ms 대기 → 창이 실시간으로 갱신됨
```

---

## 4. 좌표 주의사항

NumPy 배열과 OpenCV 의 좌표 순서가 **반대**입니다.

```
NumPy:  array[row, col]  = array[y, x]  (행, 열)
OpenCV: cv.circle((x, y))              (열, 행)
```

```python
# NumPy 배열에서 로봇 위치 찾기
robot_row = 150
robot_col = 200

robot_numpy = (robot_row, robot_col)           # NumPy: (행, 열) = (y, x)
robot_cv = (robot_col, robot_row)              # OpenCV: (x, y) = (열, 행)

# 소스 코드에서:
robot_ai = grid.coordinates_to_array_index(mapper.robot_position)
robot_pt = (int(robot_ai[1]), int(robot_ai[0]))   # [1]=col→x, [0]=row→y
cv.circle(image, robot_pt, 4, _COLORS["robot"], -1)
```

---

## ✏️ 실습

`05_practice.py` 를 열고 순서대로 진행하세요.

### 실습 1 — NumPy 배열로 간단한 지도 만들기
미로를 0/1 배열로 표현하고 조작합니다.

### 실습 2 — OpenCV 로 지도 시각화하기
NumPy 배열을 색상 이미지로 변환하고 OpenCV 창에 표시합니다.

### 실습 3 — map_visualizer.py 에 색상 추가하기
`src/map_visualizer.py` 의 `_COLORS` 딕셔너리에 새 색상을 추가하고
렌더링 코드에 적용합니다.

---

## ✅ 이번 회차 체크리스트

- [ ] `np.zeros`, `np.full` 로 배열을 만들 수 있다
- [ ] 불리언 배열을 인덱스로 써서 특정 픽셀 색상을 한 번에 바꿀 수 있다
- [ ] `np.argwhere` 로 True 인 위치 목록을 얻을 수 있다
- [ ] BGR 색상 순서를 이해하고 원하는 색을 튜플로 쓸 수 있다
- [ ] `cv.circle`, `cv.line`, `cv.imshow` 를 사용할 수 있다
- [ ] NumPy (row, col) 와 OpenCV (x, y) 의 순서 차이를 안다
- [ ] `map_visualizer.py` 의 `_render` 함수가 레이어를 어떻게 합성하는지 설명할 수 있다
