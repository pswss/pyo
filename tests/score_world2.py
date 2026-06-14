"""world2 정답 행렬 생성 + 제출 행렬 오프라인 채점 (셀 단위 진단).

채점 규칙(§5.6.10.b.vii): 시작타일 정렬 → 양쪽 비제로 셀 전부 비교 →
correctness = 일치/(일치+불일치). 카테고리별 불일치 집계로 40%의 내역을 밝힌다.
"""
import re
import numpy as np

W = "/Users/pysw/Downloads/erebus-26.0.1/game/worlds/world2.wbt"
TILES_X, TILES_Y = 9, 6
ROWS, COLS = TILES_Y * 4 + 1, TILES_X * 4 + 1   # 25 x 37

# ---------- 1. 정답 행렬 ----------
text = open(W).read()
blocks = re.findall(r"DEF (?:TILE|START_TILE) (halfTile|worldTile) \{(.*?)\n          \}", text, re.S)

def fint(b, k, d=0):
    m = re.search(rf"\b{k} (-?\d+)\b", b)
    return int(m.group(1)) if m else d

def fbool(b, k):
    m = re.search(rf"\b{k} (TRUE|FALSE)", b)
    return bool(m) and m.group(1) == "TRUE"

def flist(b, k):
    m = re.search(rf"{k} \[([^\]]*)\]", b)
    return [int(x) for x in m.group(1).replace(",", " ").split()] if m else [0, 0, 0, 0]

def fcolor(b):
    m = re.search(r"tileColor ([\d.]+) ([\d.]+) ([\d.]+)", b)
    return tuple(float(x) for x in m.groups()) if m else None

def passage_letter(c):
    if c is None: return None
    r, g, bl = c
    if abs(r-0.635) < 0.01: return None        # 일반 회색
    if bl > 0.7 and r < 0.3: return "b"        # 파랑 1-2
    if r > 0.7 and g < 0.3 and bl < 0.3: return "r"  # 빨강 3-4
    if g > 0.7 and r < 0.3: return "g"         # 초록 1-4
    if r > 0.25 and bl > 0.4 and g < 0.3: return "p" # 보라 2-3
    if r > 0.7 and g > 0.7: return "y"         # 노랑 1-3
    if r > 0.7 and 0.3 < g < 0.7: return "o"   # 주황 2-4
    return None

truth = np.full((ROWS, COLS), "0", dtype=object)

def edge_cells(r0, c0, side, span):
    """타일/쿼터 (r0,c0) 기준 엣지 셀 목록 (꼭짓점 포함). span=쿼터 수(1 or 2)."""
    n = span * 2
    if side == 0: return [(r0, c0 + i) for i in range(n + 1)]
    if side == 1: return [(r0 + i, c0 + n) for i in range(n + 1)]
    if side == 2: return [(r0 + n, c0 + i) for i in range(n + 1)]
    return [(r0 + i, c0) for i in range(n + 1)]

curved_quarters = 0
for kind, body in blocks:
    tx, ty = fint(body, "xPos"), fint(body, "zPos")
    r0, c0 = ty * 4, tx * 4
    edges = [fint(body, "topWall"), fint(body, "rightWall"),
             fint(body, "bottomWall"), fint(body, "leftWall")]
    for side, v in enumerate(edges):
        if v > 0:
            for (r, c) in edge_cells(r0, c0, side, 2):
                truth[r, c] = "1"
    if kind == "halfTile":
        offs = [(0, 0), (0, 1), (1, 0), (1, 1)]   # tile1 TL, tile2 TR, tile3 BL, tile4 BR
        curve = flist(body, "curve")
        for n, (oy, ox) in enumerate(offs, start=1):
            walls = flist(body, f"tile{n}Walls")
            rq, cq = r0 + oy * 2, c0 + ox * 2
            for side, v in enumerate(walls):
                if v > 0:
                    for (r, c) in edge_cells(rq, cq, side, 1):
                        truth[r, c] = "1"
            if curve[n - 1] > 0:
                curved_quarters += 1
                # 곡선벽: 코너 양쪽 엣지 '1', 꼭짓점 '0' (근사)
                d = curve[n - 1]   # 1=TL,2=TR,3=BR,4=BL (추정)
                pair = {1: (0, 3), 2: (0, 1), 3: (2, 1), 4: (2, 3)}[d]
                for side in pair:
                    for (r, c) in edge_cells(rq, cq, side, 1):
                        truth[r, c] = "1"
                vert = {1: (rq, cq), 2: (rq, cq + 2), 3: (rq + 2, cq + 2), 4: (rq + 2, cq)}[d]
                truth[vert] = "0"
    # 바닥 코드 (쿼터 중심 4셀)
    code = None
    if fbool(body, "trap"): code = "2"
    elif fbool(body, "swamp"): code = "3"
    elif fbool(body, "checkpoint"): code = "4"
    elif fbool(body, "start"): code = "5"
    else: code = passage_letter(fcolor(body))
    if code:
        for (dr, dc) in [(1, 1), (1, 3), (3, 1), (3, 3)]:
            truth[r0 + dr, c0 + dc] = code

print(f"정답: 타일 {len(blocks)}개, 곡선쿼터 {curved_quarters}개, 비제로 {np.count_nonzero(truth != '0')}셀")

# ---------- 2. 제출 행렬 (44x58 콘솔 출력 중 내용 행 9~33 그대로) ----------
SUB = """
0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 1 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 4 1 4 1 0 1 0 1 0 0 0 1 0 0 0 0 2 0 2 1 0 0 0 0 0 1 0 1 3 1 3 1 2 0 2 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 1 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 4 0 4 0 0 0 0 1 0 0 0 0 0 0 0 0 2 0 2 1 0 0 0 0 0 0 0 0 3 0 3 1 2 0 2 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 1 0 1 0 0 0 1 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 1 1 1 0 1 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 1 0 1 0 0 0 0 0 0 0 0 0 0 0 1 b 0 b 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 1 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 b 0 b 0 0 0 0 0 0 0 0 1 0 0 0 0 0 1 0 1 0 1 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 1 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 1 0 0 0 1 0 1 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 1 p 0 p 0 0 0 0 0 0 0 0 0 4 0 4 1 0 0 0 0 0 0 0 1 5 0 5 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 1 0 0 1 0 0 0 1 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 1 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 1 p 0 p 1 0 0 0 0 0 0 0 0 4 0 4 1 0 0 0 0 0 0 0 1 5 0 5 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 g 0 g 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 0 0 0 1 1 1 1 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 1 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 g 0 g 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 0 0 1 1 0 0 0 0 1 1 1 1 0 0 1 1 1 1 1 0 0 0 1 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 4 0 4 0 0 0 0 0 0 0 0 0 3 0 3 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 4 0 4 0 0 0 0 0 0 0 0 0 3 0 3 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 1 0 0 0 1 1 1 1 1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 2 0 2 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 1 0 0 2 0 2 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0
"""
sub_rows = [line.split() for line in SUB.strip().split("\n")]
sub = np.full((len(sub_rows), len(sub_rows[0])), "0", dtype=object)
for i, row in enumerate(sub_rows):
    for j, v in enumerate(row):
        sub[i, j] = v
print(f"제출: {sub.shape}, 비제로 {np.count_nonzero(sub != '0')}셀")

# ---------- 3. 시작타일 정렬 + 채점 ----------
def five_anchor(m):
    pos = np.argwhere(m == "5")
    assert len(pos), "시작타일 '5' 없음"
    return pos.min(axis=0)

ar, ac = five_anchor(truth)
br, bc = five_anchor(sub)
dr, dc = br - ar, bc - ac
print(f"정렬 오프셋: 제출[{dr}+r, {dc}+c] ↔ 정답[r, c]")

cats = {}
correct = incorrect = 0
diff_cells = []
for r in range(ROWS):
    for c in range(COLS):
        t = truth[r, c]
        rr, cc = r + dr, c + dc
        s = sub[rr, cc] if 0 <= rr < sub.shape[0] and 0 <= cc < sub.shape[1] else "0"
        if t == "0" and s == "0":
            continue
        if t == s:
            correct += 1
        else:
            incorrect += 1
            key = f"정답'{t}'→제출'{s}'"
            cats[key] = cats.get(key, 0) + 1
            diff_cells.append((r, c, t, s))

# 제출에만 있고 정답 범위 밖(고스트) 셀
ghost = 0
for rr in range(sub.shape[0]):
    for cc in range(sub.shape[1]):
        r, c = rr - dr, cc - dc
        if (0 <= r < ROWS and 0 <= c < COLS):
            continue
        if sub[rr, cc] != "0":
            ghost += 1
            incorrect += 1
print(f"\n정답범위 밖 고스트 셀: {ghost}")
print(f"일치 {correct} / 불일치 {incorrect} → correctness = {correct/(correct+incorrect):.3f}")
print("\n불일치 카테고리:")
for k, v in sorted(cats.items(), key=lambda x: -x[1]):
    print(f"  {k}: {v}셀")
print("\n불일치 위치 샘플 (정답행렬 좌표 r,c,정답,제출):")
for d in diff_cells[:40]:
    print(" ", d)
