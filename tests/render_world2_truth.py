"""world2.wbt 정답 벽 레이아웃 렌더링 (맵 일치율 진단용 1회성 도구).

halfTile/worldTile 블록 파싱 → 쿼터타일 격자에 벽/특수타일 그려 PNG 저장.
벽 존재 판정: 엣지 값 > 0 (1/3/5/15 등은 코너 기하 변형일 뿐 벽 존재).
"""
import re
import sys

import numpy as np
import cv2 as cv

W = "/Users/pysw/Downloads/erebus-26.0.1/game/worlds/world2.wbt"
OUT = "/tmp/world2_truth.png"

text = open(W).read()

# 타일 블록 추출 (top-level DEF TILE halfTile / worldTile)
blocks = re.findall(r"DEF TILE (halfTile|worldTile) \{(.*?)\n          \}", text, re.S)
print(f"타일 블록 {len(blocks)}개")

def fint(body, key, default=0):
    m = re.search(rf"\b{key} (-?\d+)", body)
    return int(m.group(1)) if m else default

def fbool(body, key):
    m = re.search(rf"\b{key} (TRUE|FALSE)", body)
    return m and m.group(1) == "TRUE"

def flist(body, key):
    m = re.search(rf"{key} \[([^\]]*)\]", body)
    return [int(x.strip()) for x in m.group(1).replace(",", " ").split()] if m else [0,0,0,0]

QT = 30          # 쿼터타일 px
TILES_X, TILES_Y = 9, 6
img = np.zeros((TILES_Y*2*QT+2, TILES_X*2*QT+2, 3), np.uint8)
img[:] = (40, 40, 40)

def draw_edge(qx, qy, side, color=(255,255,255), t=3):
    x0, y0 = qx*QT, qy*QT
    if side == 0: cv.line(img, (x0, y0), (x0+QT, y0), color, t)          # top
    elif side == 1: cv.line(img, (x0+QT, y0), (x0+QT, y0+QT), color, t)  # right
    elif side == 2: cv.line(img, (x0, y0+QT), (x0+QT, y0+QT), color, t)  # bottom
    elif side == 3: cv.line(img, (x0, y0), (x0, y0+QT), color, t)        # left

for kind, body in blocks:
    tx, ty = fint(body, "xPos"), fint(body, "zPos")
    qx, qy = tx*2, ty*2
    # 특수 타일 바닥색
    fill = None
    if fbool(body, "trap"): fill = (30, 30, 30)
    elif fbool(body, "swamp"): fill = (60, 110, 150)
    elif fbool(body, "checkpoint"): fill = (190, 190, 190)
    elif fbool(body, "start"): fill = (80, 200, 80)
    room = fint(body, "room", 0)
    if fill is None and room >= 2:
        fill = (70, 60, 40) if room == 2 else (40, 60, 70)
    if fill is not None:
        cv.rectangle(img, (qx*QT+2, qy*QT+2), ((qx+2)*QT-2, (qy+2)*QT-2), fill, -1)
    if not fbool(body, "floor") and not fbool(body, "trap"):
        cv.rectangle(img, (qx*QT+2, qy*QT+2), ((qx+2)*QT-2, (qy+2)*QT-2), (90, 40, 90), -1)

    if kind == "worldTile":
        # 풀타일: 엣지 4개 (2쿼터 길이)
        edges = [fint(body,"topWall"), fint(body,"rightWall"), fint(body,"bottomWall"), fint(body,"leftWall")]
        if edges[0] > 0: [draw_edge(qx+i, qy, 0) for i in range(2)]
        if edges[1] > 0: [draw_edge(qx+1, qy+i, 1) for i in range(2)]
        if edges[2] > 0: [draw_edge(qx+i, qy+1, 2) for i in range(2)]
        if edges[3] > 0: [draw_edge(qx, qy+i, 3) for i in range(2)]
    else:
        # halfTile: 외곽 4 엣지 + 쿼터별 내부 벽
        edges = [fint(body,"topWall"), fint(body,"rightWall"), fint(body,"bottomWall"), fint(body,"leftWall")]
        if edges[0] > 0: [draw_edge(qx+i, qy, 0) for i in range(2)]
        if edges[1] > 0: [draw_edge(qx+1, qy+i, 1) for i in range(2)]
        if edges[2] > 0: [draw_edge(qx+i, qy+1, 2) for i in range(2)]
        if edges[3] > 0: [draw_edge(qx, qy+i, 3) for i in range(2)]
        offs = [(0,0), (1,0), (0,1), (1,1)]
        for n, (ox, oy) in enumerate(offs, start=1):
            walls = flist(body, f"tile{n}Walls")
            for side, v in enumerate(walls):
                if v > 0:
                    draw_edge(qx+ox, qy+oy, side, (0, 220, 255), 2)  # 내부 쿼터벽=노랑

# 조난자/해즈맷 (translation x z → 타일좌표, tile=0.12m, 월드 중심 정렬 추정)
for m in re.finditer(r"(Victim|CognitiveTarget)[^{]*\{[^}]*?translation ([\-\d.]+) [\-\d.]+ ([\-\d.]+)", text):
    kind2, x, z = m.group(1), float(m.group(2)), float(m.group(3))
    px = int((x + TILES_X*0.06) / 0.06 * QT)
    py = int((z + TILES_Y*0.06) / 0.06 * QT)
    color = (0, 160, 255) if kind2 == "Victim" else (0, 0, 255)
    cv.circle(img, (px, py), 6, color, -1)

cv.imwrite(OUT, img)
print(f"저장: {OUT}")
