"""
1회차 완성본 — 클래스와 객체
실행: python study/complete/01_complete.py
"""

print("=" * 50)
print("실습 1: 센서 클래스")
print("=" * 50)

# -------------------------------------------------------
# [개념] 클래스 기본 구조
#   class 이름:
#       def __init__(self, ...): → 생성자 (인스턴스 만들 때 자동 실행)
#           self.변수 = 값       → 인스턴스 변수 (이 객체만의 데이터)
#       def 메서드(self, ...):   → 메서드 (클래스 안의 함수)
# -------------------------------------------------------

class DistanceSensor:
    """거리 센서 클래스 (라이다, 초음파 센서 등)"""

    def __init__(self, name, max_range_m):
        # self.변수 = 값 형태로 인스턴스 변수를 만든다.
        # 같은 클래스로 만든 각 인스턴스는 자신만의 name, max_range 를 가진다.
        self.name = name
        self.max_range = max_range_m
        self.last_reading = None   # 아직 측정값이 없으므로 None 으로 초기화

    def measure(self, actual_distance):
        """실제 거리를 측정합니다. 범위 초과 시 None 반환."""
        if actual_distance > self.max_range:
            self.last_reading = None   # 범위 밖이면 값 없음
        else:
            self.last_reading = actual_distance
        return self.last_reading

    def is_obstacle_close(self, threshold=0.15):
        """장애물이 threshold 미터 이내에 있으면 True."""
        # last_reading 이 None 이면 측정값이 없는 것 → False
        if self.last_reading is None:
            return False
        return self.last_reading < threshold

lidar = DistanceSensor("전방 라이다", max_range_m=2.0)
print(f"센서 이름: {lidar.name}")
print(f"측정값 (1.5m): {lidar.measure(1.5)}")
print(f"측정값 (3.0m): {lidar.measure(3.0)}")
print(f"측정값 (0.1m): {lidar.measure(0.1)}")
print(f"장애물 근접?: {lidar.is_obstacle_close()}")

print()

# -------------------------------------------------------
# [완성] TODO 1-1: 카메라 센서 클래스
# -------------------------------------------------------

class Camera:
    """카메라 센서 클래스 — 시야각(FOV) 내에 있는 물체만 감지한다."""

    def __init__(self, name, fov_degrees):
        self.name = name
        self.fov_degrees = fov_degrees
        # 시야각의 절반 = 감지 가능한 최대 각도 (중앙 기준 좌우)
        # 예: fov=60 → half=30 → -30~+30 도 범위만 감지 가능
        self.half_fov = fov_degrees / 2

    def detect(self, angle):
        """
        주어진 각도(도)가 시야각 절반 이내이면 '감지됨', 아니면 '범위 밖' 반환.

        abs(angle) 을 쓰는 이유:
          - angle 이 +20 이든 -20 이든 카메라 중앙에서 20도 벗어난 것
          - 부호를 무시하고 얼마나 벗어났는지만 확인
        """
        if abs(angle) <= self.half_fov:
            return "감지됨"
        return "범위 밖"

# 테스트
cam = Camera("전방 카메라", fov_degrees=60)
print(f"[카메라: {cam.name}, 시야각={cam.fov_degrees}도]")
print(f"  각도 20도:  {cam.detect(20)}")    # 20 <= 30 → 감지됨
print(f"  각도 45도:  {cam.detect(45)}")    # 45 > 30  → 범위 밖
print(f"  각도 -25도: {cam.detect(-25)}")   # abs(-25)=25 <= 30 → 감지됨
print(f"  각도 0도:   {cam.detect(0)}")     # 정중앙 → 감지됨

# src/executor/executor.py 의 카메라 감지와 연결
# - 실제 코드에서는 fixture_detector.find_fixtures(cam_image.image) 로
#   카메라 이미지 전체를 분석해서 조난자를 찾는다.
# - 이 클래스는 그 원리를 "각도" 개념으로 단순화한 것이다.


print()
print("=" * 50)
print("실습 2: 로봇 위치 클래스")
print("=" * 50)

# -------------------------------------------------------
# [완성] TODO 1-2: SimpleRobot 에 distance_from_start() 와 set_speed() 추가
# -------------------------------------------------------

import math  # sqrt, radians, sin, cos 를 쓰기 위해

class SimpleRobot:
    """단순한 2D 로봇 위치 추적 클래스 (기능 추가 완성본)"""

    def __init__(self, start_x=0.0, start_y=0.0):
        self.x = start_x
        self.y = start_y
        self.angle = 0.0        # 도(degree), 0=북쪽
        self.speed = 0.0
        self.steps_taken = 0

        # 시작 위치를 기억해 두면 나중에 거리 계산에 사용할 수 있다.
        # rescue_robot.py 의 mapper.start_position 과 같은 역할.
        self._start_x = start_x
        self._start_y = start_y

    def move_forward(self, distance):
        """현재 방향으로 distance만큼 이동."""
        rad = math.radians(self.angle)
        self.x += math.sin(rad) * distance
        self.y += math.cos(rad) * distance
        self.steps_taken += 1
        print(f"  → 전진 {distance}m: 현재 위치 ({self.x:.2f}, {self.y:.2f})")

    def turn(self, degrees):
        """시계방향으로 degrees 회전."""
        # % 360 을 하면 항상 0~359 범위로 유지된다.
        self.angle = (self.angle + degrees) % 360
        print(f"  → {degrees}도 회전: 현재 방향 {self.angle:.0f}도")

    def reset_to_start(self):
        """출발점으로 복귀."""
        self.x = self._start_x
        self.y = self._start_y
        self.angle = 0.0
        print(f"  → 출발점 복귀 완료")

    def report(self):
        """현재 상태를 출력."""
        dist = self.distance_from_start()  # 아래에서 정의한 메서드를 호출
        print(f"  위치=({self.x:.2f}, {self.y:.2f}), "
              f"방향={self.angle:.0f}도, "
              f"출발점까지={dist:.3f}m, "
              f"이동횟수={self.steps_taken}")

    # ---- 추가된 메서드 1 ----
    def distance_from_start(self) -> float:
        """
        현재 위치에서 출발점까지의 직선 거리(미터)를 반환한다.

        피타고라스 정리:  distance = sqrt( dx^2 + dy^2 )
        math.sqrt() 는 제곱근을 계산한다.
        ** 는 거듭제곱 연산자: x**2 = x의 2승 = x*x
        """
        dx = self.x - self._start_x
        dy = self.y - self._start_y
        return math.sqrt(dx**2 + dy**2)

    # ---- 추가된 메서드 2 ----
    def set_speed(self, speed: float):
        """
        속도를 설정한다. 항상 0.0 ~ 1.0 범위로 제한(클리핑)된다.

        max(0.0, min(1.0, speed)) 의 동작:
          - min(1.0, speed): speed 가 1.0 을 넘으면 1.0 으로 낮춤
          - max(0.0, ...):   그 결과가 0.0 미만이면 0.0 으로 올림
          → 항상 [0.0, 1.0] 범위가 보장된다.

        rescue_robot.py 의 move_forward(speed) 에서도
        이런 클리핑 패턴이 내부적으로 사용된다.
        """
        self.speed = max(0.0, min(1.0, speed))
        print(f"  → 속도 설정: {self.speed:.2f}")

# 테스트
bot = SimpleRobot()
bot.report()
bot.move_forward(0.5)
bot.turn(90)
bot.move_forward(0.3)
bot.set_speed(1.5)    # 범위 초과 → 1.0 으로 클리핑
bot.set_speed(-0.2)   # 범위 미달 → 0.0 으로 클리핑
bot.set_speed(0.7)
bot.report()
bot.reset_to_start()
bot.report()


print()
print("=" * 50)
print("실습 3: 여러 인스턴스 사용하기")
print("=" * 50)

class VictimMarker:
    """발견된 조난자 위치를 기록하는 클래스"""

    total_found = 0   # 클래스 변수: 모든 인스턴스가 공유한다

    def __init__(self, x, y, letter):
        self.x = x
        self.y = y
        self.letter = letter
        self.reported = False
        VictimMarker.total_found += 1  # 클래스 변수를 증가

    def report(self):
        """조난자를 서버에 보고합니다."""
        if self.reported:
            print(f"  [!] 이미 보고함: {self.letter} @ ({self.x}, {self.y})")
            return
        self.reported = True
        print(f"  [OK] 보고 완료: {self.letter} @ ({self.x:.2f}, {self.y:.2f})")

    def __repr__(self):
        """print() 할 때 이 객체를 어떻게 표시할지 정의"""
        status = "보고됨" if self.reported else "미보고"
        return f"Victim({self.letter}, {status})"

v1 = VictimMarker(0.12, 0.36, "H")
v2 = VictimMarker(0.48, 0.12, "S")
v3 = VictimMarker(0.60, 0.60, "U")

print(f"발견된 조난자 수: {VictimMarker.total_found}")
v1.report()
v1.report()   # 중복 보고 시도
v2.report()
print(f"\n조난자 목록: {[v1, v2, v3]}")

print()

# -------------------------------------------------------
# [완성] TODO 1-3: 미보고 항목만 모두 보고하는 함수
# -------------------------------------------------------

def report_all_unreported(victim_list):
    """
    VictimMarker 리스트를 받아서 아직 보고 안 된 것만 순서대로 보고한다.

    for 루프로 리스트 전체를 순회하면서
    각 항목의 reported 가 False 인 것만 v.report() 를 호출한다.

    rescue_robot.py 의 report_victim() 을 여러 개 처리하는 것과 같은 흐름이다.
    """
    print("[report_all_unreported] 미보고 조난자 순차 보고 시작")
    count = 0
    for v in victim_list:
        if not v.reported:   # reported == False 인 것만
            v.report()
            count += 1
    print(f"[report_all_unreported] 총 {count}명 보고 완료")

# 테스트: v3 만 아직 미보고 상태
report_all_unreported([v1, v2, v3])

# 한 번 더 호출해도 이미 보고된 항목은 건너뜀
print()
report_all_unreported([v1, v2, v3])


print()
print("=" * 50)
print("[참고] 실제 소스 코드 연결 확인")
print("=" * 50)
print("""
이번 회차 핵심 패턴과 src 코드의 연결:

1. __init__ 에서 다른 객체를 self.변수 에 저장
   → rescue_robot.py: self._robot = Robot(...), self._mapper = Mapper(...)

2. 메서드 안에서 self.다른메서드() 호출
   → rescue_robot.py: stop() 안에서 self._robot.move_wheels(0, 0) 호출

3. 클래스 변수로 전체 상태 추적 (VictimMarker.total_found)
   → executor.py: Executor.fixture_detection_cooldown 이 비슷한 역할

4. __repr__ 으로 print() 출력 형식 정의
   → 디버그 로그에서 객체를 바로 print() 할 때 유용
""")
