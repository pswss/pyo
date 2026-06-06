"""
[예제 00d] 빌드 시스템 이해 - stickytape 동작 시뮬레이션
------------------------------------------------------------
터미널에서 실행: python 예제00d_빌드시스템이해.py

stickytape가 여러 파일을 하나로 합치는 과정을 직접 시뮬레이션합니다.
Webots 없이 실행 가능합니다.
"""

import os
import sys
import tempfile
import shutil

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR    = os.path.normpath(os.path.join(SCRIPT_DIR, "../../../src"))
COMPILED   = os.path.normpath(os.path.join(SCRIPT_DIR, "../../../compiled.py"))

print("=" * 65)
print("  빌드 시스템 이해 - run.py vs compiled.py")
print("=" * 65)

# ── 1. run.py 원리 ────────────────────────────────────────────
print("\n[1] run.py 동작 원리")
print("-" * 65)

run_py_path = os.path.join(SRC_DIR, "run.py")
if os.path.exists(run_py_path):
    with open(run_py_path, encoding="utf-8") as f:
        content = f.read()
    print("  src/run.py 내용:")
    for line in content.strip().splitlines():
        print(f"    {line}")
else:
    print("  (run.py 없음)")

print()
print("  동작 방식:")
print(f"    1. __file__ = '{run_py_path}'")
src_dir_resolved = os.path.dirname(os.path.abspath(run_py_path))
print(f"    2. src_dir  = '{src_dir_resolved}'")
print(f"    3. sys.path에 src/ 추가 -> import main 실행")
print(f"    4. main.py가 robot/, mapping/, agent/ 등을 import")
print()

# 현재 src/ 파일 수 세기
py_count = sum(
    len([f for f in files if f.endswith(".py")])
    for _, _, files in os.walk(SRC_DIR)
)
print(f"  결과: src/ 안의 Python 파일 {py_count}개가 모두 로드됨")
print(f"  조건: src/ 폴더가 반드시 같은 위치에 있어야 함!")

# ── 2. compiled.py 원리 ───────────────────────────────────────
print("\n[2] compiled.py 동작 원리")
print("-" * 65)

if os.path.exists(COMPILED):
    size_kb = os.path.getsize(COMPILED) / 1024
    with open(COMPILED, encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    # 번들된 모듈 수 세기
    module_count = sum(1 for line in lines if "__stickytape_write_module" in line)

    print(f"  compiled.py 크기: {size_kb:.0f} KB ({len(lines):,} 줄)")
    print(f"  내부 번들 모듈:   {module_count}개")
    print()
    print("  실행 시 내부 동작:")
    print("    1. 임시 폴더 생성 (tempfile.mkdtemp)")

    # 실제로 tempdir 만들어서 시뮬레이션
    tmpdir = tempfile.mkdtemp()
    print(f"       -> 예: {tmpdir}")
    print("    2. 각 모듈을 임시 폴더에 파일로 저장")
    print("       예: tmpXXX/robot/robot.py")
    print("           tmpXXX/mapping/mapper.py ...")
    print("    3. 임시 폴더를 sys.path에 추가")
    print("    4. main() 실행 (실제 코드와 동일)")
    print("    5. 종료 시 임시 폴더 자동 삭제")
    shutil.rmtree(tmpdir)

    print()
    print("  왜 오류 메시지에 Temp 경로가 뜨는가?")
    print("    OverflowError at 'C:/Users/.../AppData/Local/Temp/tmpXXX/...")
    print("    -> 실제 소스가 아닌 임시 폴더의 파일에서 오류 발생!")
    print("    -> 실제 수정은 compiled.py 안의 해당 모듈 텍스트를 수정해야 함")
else:
    print("  compiled.py를 찾을 수 없습니다.")

# ── 3. 비교표 ─────────────────────────────────────────────────
print("\n[3] run.py vs compiled.py 비교")
print("-" * 65)

headers = ["항목", "run.py", "compiled.py"]
rows = [
    ["파일 수",      "src/ 46개 파일",     "1개 (모두 포함)"],
    ["크기",         "7줄",                f"{size_kb:.0f}KB" if os.path.exists(COMPILED) else "~225KB"],
    ["수정 반영",    "저장 즉시",           "재컴파일 필요"],
    ["src/ 필요",    "필요",               "불필요"],
    ["대회 제출",    "X (src/가 없으면 실패)", "O (파일 하나만 제출)"],
    ["사용 시점",    "개발 / 테스트",       "대회 / 최종 배포"],
]

col_w = [12, 22, 22]
sep = "  +" + "+".join("-" * (w+2) for w in col_w) + "+"
print(sep)
print("  | " + " | ".join(h.ljust(w) for h, w in zip(headers, col_w)) + " |")
print(sep)
for row in rows:
    print("  | " + " | ".join(v.ljust(w) for v, w in zip(row, col_w)) + " |")
print(sep)

# ── 4. stickytape 명령 ────────────────────────────────────────
print("\n[4] 재컴파일 명령 (필요할 때만!)")
print("-" * 65)

stickytape = r"C:\Users\snowbot\AppData\Roaming\Python\Python314\Scripts\stickytape.exe"
print()
print("  # PowerShell에서 실행 (rescate_laberinto-master/ 폴더 기준)")
print()
print(f"  {stickytape} `")
print("      src/main.py `")
print("      --add-python-path src/ `")
print("      | Out-File -Encoding utf8 compiled.py")
print()
print("  각 옵션 의미:")
options = [
    ("src/main.py",              "번들링 시작점 (프로그램 진입점)"),
    ("--add-python-path src/",   "src/ 안의 모듈을 찾을 수 있게 경로 등록"),
    ("| Out-File ...",           "출력을 파일로 저장 (PowerShell 방식)"),
    ("-Encoding utf8",           "UTF-8 인코딩 지정 (필수! 없으면 UTF-16)"),
]
for opt, desc in options:
    print(f"    {opt:<30} <- {desc}")

# ── 5. 직접 패치 방법 ─────────────────────────────────────────
print("\n[5] compiled.py 직접 패치 방법")
print("-" * 65)
print()
print("  재컴파일 없이 compiled.py 내 버그를 고칠 때:")
print()
print("  방법: compiled.py를 텍스트 에디터로 열고")
print("        내부 바이트 문자열 안에서 해당 코드를 찾아 수정")
print()
print("  실제 적용 사례 - uint8 오버플로 버그:")
print()
print("  [찾을 내용]")
print("    return preference_grid[position[0], position[1]]")
print()
print("  [수정 내용]")
print("    return int(preference_grid[position[0], position[1]])")
print()
print("  주의: compiled.py 내부는 \\n 등 이스케이프 형태로 저장되어 있음")
print("        텍스트 그대로 검색하면 찾을 수 있음 (VS Code Ctrl+F)")

# ── 6. 권장 워크플로 ─────────────────────────────────────────
print("\n[6] 권장 개발 워크플로")
print("-" * 65)
print()
steps = [
    ("1. 코드 수정",     "src/ 안의 파일을 VS Code에서 수정"),
    ("2. 빠른 테스트",   "Erebus 컨트롤러 = src/run.py  ->  수정 즉시 확인"),
    ("3. 검증 완료",     "여러 맵에서 충분히 테스트"),
    ("4. 대회 제출",     "compiled.py를 Erebus 컨트롤러로 설정 후 최종 확인"),
    ("5. 파일 제출",     "compiled.py 단일 파일을 USB/이메일로 제출"),
]
for step, desc in steps:
    print(f"  {step:<15} {desc}")

print()
print("  TIP: run.py로 테스트 -> compiled.py로 대회")
print("       src/ 수정은 run.py에서 바로 반영, 재컴파일 불필요!")
