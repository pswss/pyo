"""
1회차 실습 — 클래스와 객체
실행: python study/01_practice.py
"""

print("=" * 50)
print("실습 1: 센서 클래스")
print("=" * 50)

# -------------------------------------------------------
# [개념] 클래스 기본 구조
#   class 이름:
#       def __init__(self, ...): → 생성자
#           self.변수 = 값       → 인스턴스 변수
#       def 메서드(self, ...):   → 메서드
# -------------------------------------------------------

# 완성 예시 — 먼저 읽고 이해하세요
class DistanceSensor:
    """거리 센서 클래스 (라이다, 초음파 센서 등)"""

    def __init__(self, name, max_range_m):
        self.name = name
        self.max_range = max_range_m
        self.last_reading = None      # 가장 최근 측정값

    def measure(self, actual_distance):
        """실제 거리를 측정합니다. 범위 초과 시 None 반환."""
        if actual_distance > self.max_range:
            self.last_reading = None
        else:
            self.last_reading = actual_distance
        return self.last_reading

    def is_obstacle_close(self, threshold=0.15):
        """장애물이 threshold 미터 이내에 있으면 True."""
        if self.last_reading is None:
            return False
        return self.last_reading < threshold

# 사용해보기
lidar = DistanceSensor("전방 라이다", max_range_m=2.0)
print(f"센서 이름: {lidar.name}")
print(f"측정값 (1.5m): {lidar.measure(1.5)}")   # 1.5
print(f"측정값 (3.0m): {lidar.measure(3.0)}")   # None (범위 초과)
print(f"측정값 (0.1m): {lidar.measure(0.1)}")   # 0.1
print(f"장애물 근접?: {lidar.is_obstacle_close()}")  # True

print()

# -------------------------------------------------------
# TODO 1-1: 카메라 센서 클래스를 만드세요.
#
# 요구사항:
#   - 이름(name)과 시야각(fov_degrees)을 받는 __init__
#   - detect(angle) 메서드: 주어진 각도가 시야각 절반 이내면
#     "감지됨" 반환, 아니면 "범위 밖" 반환
#   - 예: fov=60도인 카메라 → -30~+30도 안의 각도만 감지
#
# 힌트: 절댓값은 abs() 함수를 사용하세요.
# -------------------------------------------------------

# 여기에 코드를 작성하세요 ↓

# class Camera:
#     def __init__(self, ...):
#         ...
#     def detect(self, angle):
#         ...


# 작성 후 아래 주석을 해제해서 테스트하세요:
# cam = Camera("전방 카메라", fov_degrees=60)
# print(cam.detect(20))    # 감지됨 (20 < 30)
# print(cam.detect(45))    # 범위 밖 (45 > 30)
# print(cam.detect(-25))   # 감지됨 (-25의 절댓값 25 < 30)


print("=" * 50)
print("실습 2: 로봇 위치 클래스")
print("=" * 50)

# -------------------------------------------------------
# [개념] 메서드 안에서 다른 메서드 호출
#   def stop(self):
#       self.set_speed(0, 0)   ← self.메서드() 로 호출
# -------------------------------------------------------

# 완성 예시
class SimpleRobot:
    """단순한 2D 로봇 위치 추적 클래스"""

    def __init__(self, start_x=0.0, start_y=0.0):
        self.x = start_x
        self.y = start_y
        self.angle = 0.0        # 도(degree), 0=북쪽
        self.speed = 0.0
        self.steps_taken = 0    # 이동 횟수 기록

    def move_forward(self, distance):
        """현재 방향으로 distance만큼 이동."""
        import math
        rad = math.radians(self.angle)
        self.x += math.sin(rad) * distance
        self.y += math.cos(rad) * distance
        self.steps_taken += 1
        print(f"  → 전진 {distance}m: 현재 위치 ({self.x:.2f}, {self.y:.2f})")

    def turn(self, degrees):
        """시계방향으로 degrees 회전."""
        self.angle = (self.angle + degrees) % 360
        print(f"  → {degrees}° 회전: 현재 방향 {self.angle:.0f}°")

    def reset_to_start(self):
        """출발점으로 복귀."""
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        print(f"  → 출발점 복귀 완료")

    def report(self):
        """현재 상태를 출력."""
        print(f"  위치=({self.x:.2f}, {self.y:.2f}), 방향={self.angle:.0f}°, 이동횟수={self.steps_taken}")

# 테스트
bot = SimpleRobot()
bot.report()
bot.move_forward(0.5)
bot.turn(90)           # 동쪽으로
bot.move_forward(0.3)
bot.report()
bot.reset_to_start()
bot.report()

print()

# -------------------------------------------------------
# TODO 1-2: SimpleRobot 에 기능을 추가하세요.
#
# 1) distance_from_start() 메서드 추가
#    - 현재 위치에서 출발점(0, 0)까지의 거리를 반환
#    - 힌트: import math, math.sqrt(x**2 + y**2)
#
# 2) set_speed(speed) 메서드 추가
#    - self.speed 를 변경 (0.0 ~ 1.0 사이로 제한)
#    - 0보다 작으면 0.0, 1보다 크면 1.0 으로 자동 클리핑
#
# 아래에 직접 클래스를 작성하거나, 위의 SimpleRobot 에 추가하세요.
# -------------------------------------------------------

# 여기에 코드를 작성하세요 ↓


print("=" * 50)
print("실습 3: 여러 인스턴스 사용하기")
print("=" * 50)

# -------------------------------------------------------
# [개념] 같은 클래스로 여러 인스턴스를 만들 수 있다.
# 각 인스턴스는 자신만의 데이터(self.변수)를 가진다.
# -------------------------------------------------------

class VictimMarker:
    """발견된 조난자 위치를 기록하는 클래스"""

    total_found = 0   # 클래스 변수: 모든 인스턴스가 공유

    def __init__(self, x, y, letter):
        self.x = x
        self.y = y
        self.letter = letter
        self.reported = False
        VictimMarker.total_found += 1

    def report(self):
        """조난자를 서버에 보고합니다."""
        if self.reported:
            print(f"  [!] 이미 보고함: {self.letter} @ ({self.x}, {self.y})")
            return
        self.reported = True
        print(f"  [OK] 보고 완료: {self.letter} @ ({self.x:.2f}, {self.y:.2f})")

    def __repr__(self):
        """print() 할 때 출력되는 형식 정의"""
        status = "보고됨" if self.reported else "미보고"
        return f"Victim({self.letter}, {status})"

# 조난자 세 명 발견
v1 = VictimMarker(0.12, 0.36, "H")
v2 = VictimMarker(0.48, 0.12, "S")
v3 = VictimMarker(0.60, 0.60, "U")

print(f"발견된 조난자 수: {VictimMarker.total_found}")

v1.report()
v1.report()   # 중복 보고 시도 → 경고 출력
v2.report()

print(f"\n조난자 목록: {[v1, v2, v3]}")

# -------------------------------------------------------
# TODO 1-3: VictimMarker 리스트를 받아서
#           미보고 항목만 모두 보고하는 함수를 작성하세요.
#
# def report_all_unreported(victim_list):
#     ...
# -------------------------------------------------------

# 여기에 코드를 작성하세요 ↓


print()
print("=" * 50)
print("[참고] 실제 소스 코드 연결 확인")
print("=" * 50)
print("""
src/rescue_robot.py 를 열고 확인해 보세요:

1. RescueRobot.__init__ 에서 만드는 객체 3가지:
   → self._robot = Robot(...)
   → self._mapper = Mapper(...)
   → self._executor = Executor(...)

2. stop() 메서드가 내부적으로 호출하는 것:
   → self._robot.move_wheels(0, 0)
      ↑ 다른 클래스의 메서드를 self.변수.메서드() 로 호출

3. elapsed_time 프로퍼티:
   → return self._robot.time
      ↑ Robot 객체의 time 변수를 그대로 전달

이것이 '클래스 합성(Composition)' 패턴입니다.
""")
