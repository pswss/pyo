# GPSNavigation.py
from controller import Robot
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# GPS 활성화
gps = robot.getDevice('gps')
gps.enable(timestep)

# 모터 설정
left_motor = robot.getDevice('wheel1 motor')
right_motor = robot.getDevice('wheel2 motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# 목표 좌표
TARGET_X = 0.2
TARGET_Y = 0.03
TARGET_Z = 0.2

# 속도 설정
BASE_SPEED = 3.0
TURN_SPEED = 2.0

# 도착 판정 거리 (미터)
ARRIVAL_THRESHOLD = 0.05  # 5cm 이내면 도착

# 방향 추정을 위한 최소 이동 거리 (줄임)
MIN_MOVEMENT = 0.0001  # 0.1mm (기존: 0.001m = 1mm)

# 시작 위치 저장 변수
start_position = None
is_first_run = True

# 이전 위치 (방향 추정용) - 여러 프레임 전 위치 저장
position_history = []
HISTORY_SIZE = 5  # 5프레임 전 위치와 비교

def calculate_distance(x1, z1, x2, z2):
    """두 점 사이의 거리 계산 (y는 항상 0.03이므로 x, z만 사용)"""
    return math.sqrt((x2 - x1) ** 2 + (z2 - z1) ** 2)

def get_target_angle(current_x, current_z, target_x, target_z):
    """현재 위치에서 목표까지의 각도 계산"""
    dx = target_x - current_x
    dz = target_z - current_z
    return math.atan2(dx, dz)

def get_robot_heading_from_history(position_history):
    """위치 히스토리를 사용하여 로봇의 이동 방향 추정"""
    if len(position_history) < 2:
        return None
    
    # 가장 오래된 위치와 최신 위치 비교
    old_x, old_z = position_history[0]
    new_x, new_z = position_history[-1]
    
    dx = new_x - old_x
    dz = new_z - old_z
    
    # 이동 거리 계산
    distance = math.sqrt(dx*dx + dz*dz)
    
    # 최소 이동 거리 체크
    if distance < MIN_MOVEMENT:
        return None
    
    return math.atan2(dx, dz)

def normalize_angle(angle):
    """각도를 -π ~ π 범위로 정규화"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

print("=" * 50)
print("GPS 네비게이션 시작 (compass 미사용)")
print(f"목표 좌표: X={TARGET_X}, Y={TARGET_Y}, Z={TARGET_Z}")
print("=" * 50)

# 상태 머신
STATE_INIT = 0          # 초기화 (방향 파악을 위해 전진)
STATE_NAVIGATE = 1      # 네비게이션 중
state = STATE_INIT
init_counter = 0

while robot.step(timestep) != -1:
    # 현재 위치 읽기
    pos = gps.getValues()
    current_x, current_y, current_z = pos[0], pos[1], pos[2]
    
    # 위치 히스토리에 추가
    position_history.append((current_x, current_z))
    if len(position_history) > HISTORY_SIZE:
        position_history.pop(0)  # 오래된 데이터 제거
    
    # 시작 위치 저장 (첫 번째 실행 시)
    if is_first_run:
        start_position = (current_x, current_y, current_z)
        is_first_run = False
        print(f"\n📍 시작 위치 저장: X={start_position[0]:.3f}, Z={start_position[2]:.3f}")
    
    # 1. 시작 위치와 현재 위치 사이의 거리 계산
    distance_from_start = calculate_distance(
        start_position[0], start_position[2],
        current_x, current_z
    )
    
    # 2. 목표 좌표까지의 거리 계산
    distance_to_target = calculate_distance(
        current_x, current_z,
        TARGET_X, TARGET_Z
    )
    
    # 3. 목표 도착 판정
    if distance_to_target < ARRIVAL_THRESHOLD:
        print("\n" + "=" * 50)
        print("🎉 목표 도착!")
        print(f"최종 위치: X={current_x:.3f}, Y={current_y:.3f}, Z={current_z:.3f}")
        print(f"시작점으로부터 총 이동 거리: {distance_from_start:.3f}m")
        print("=" * 50)
        
        # 정지
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break
    
    # 상태 머신 처리
    if state == STATE_INIT:
        # 초기 상태: 충분히 전진하여 방향 파악
        left_motor.setVelocity(BASE_SPEED)
        right_motor.setVelocity(BASE_SPEED)
        init_counter += 1
        
        print(f"🔄 초기화 중... ({init_counter}/30) - 현재 이동: {distance_from_start:.4f}m")
        
        # 충분한 거리를 이동했거나 30프레임이 지나면 네비게이션 시작
        if init_counter >= 30 or distance_from_start > 0.01:  # 1cm 이상 이동
            state = STATE_NAVIGATE
            print("✅ 방향 파악 완료, 네비게이션 시작")
    
    elif state == STATE_NAVIGATE:
        # 위치 히스토리로 로봇 방향 추정
        robot_heading = get_robot_heading_from_history(position_history)
        target_angle = get_target_angle(current_x, current_z, TARGET_X, TARGET_Z)
        
        if robot_heading is not None:
            # 회전해야 할 각도
            angle_diff = normalize_angle(target_angle - robot_heading)
            
            print(f"현재: X={current_x:.3f}, Z={current_z:.3f} | "
                  f"시작거리: {distance_from_start:.3f}m | "
                  f"목표거리: {distance_to_target:.3f}m | "
                  f"각도차: {math.degrees(angle_diff):.1f}°", end=" | ")
            
            # 이동 로직
            if abs(angle_diff) > math.radians(20):  # 20도 이상 차이나면 회전
                if angle_diff > 0:
                    left_motor.setVelocity(-TURN_SPEED)
                    right_motor.setVelocity(TURN_SPEED)
                    print("↺ 왼쪽 회전")
                else:
                    left_motor.setVelocity(TURN_SPEED)
                    right_motor.setVelocity(-TURN_SPEED)
                    print("↻ 오른쪽 회전")
            else:
                # 방향이 맞으면 전진 (각도 보정 포함)
                correction = angle_diff * 2.0
                
                left_speed = BASE_SPEED - correction
                right_speed = BASE_SPEED + correction
                
                # 속도 제한
                left_speed = max(-BASE_SPEED, min(BASE_SPEED, left_speed))
                right_speed = max(-BASE_SPEED, min(BASE_SPEED, right_speed))
                
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                print("⬆️ 전진")
        else:
            # 방향 추정 불가 - 계속 전진
            left_motor.setVelocity(BASE_SPEED)
            right_motor.setVelocity(BASE_SPEED)
            
            # 히스토리 정보 표시
            if len(position_history) >= 2:
                old_x, old_z = position_history[0]
                movement = math.sqrt((current_x - old_x)**2 + (current_z - old_z)**2)
                print(f"⬆️ 전진 중 (이동량: {movement:.5f}m, 최소: {MIN_MOVEMENT:.5f}m)")
            else:
                print(f"⬆️ 전진 중 (히스토리 수집 중: {len(position_history)}/{HISTORY_SIZE})")

print("\n프로그램 종료")