from collections import deque

from data_structures.vectors import Position2D

class StuckDetector:
    """로봇이 실제로 이동하지 않는 상태를 감지하는 클래스입니다. 규칙 2개:

    1) per-step: 바퀴는 돌고 있는데 스텝 이동 <1mm가 연속 50스텝 — 벽에 끼어 헛도는 경우.
    2) window: 최근 window_frames(약 3초) 동안 순이동 < window_min_displacement —
       정지 분기 데드락용. 실런에서 블랙홀/벽 옆 정지 분기에 빠지면 바퀴 0 +
       GPS 노이즈 ±2~4mm 떨림이라 per-step 규칙이 못 잡는 것을 확인
       (①바퀴 0 → 카운터 리셋, ②노이즈 떨림 > 1mm 임계). 순이동 기준은 둘 다 면역.
    """
    def __init__(self) -> None:
        self.stuck_counter = 0          # 끼임 상태가 지속된 스텝(타임스텝) 수

        self.stuck_threshold = 50       # 이 값을 초과하면 '끼임 확정'으로 판단
        self.minimum_distance_traveled = 0.001  # 한 스텝에서 이 거리 미만이면 이동 안 한 것으로 판단

        # 창 기반 정체 감지. 3초간 순이동 2cm 미만 = 정체.
        # GPS 노이즈 σ≈2.5mm로는 3초 순이동 2cm를 못 만든다(오탐 면역). ★시뮬 튜닝
        self.window_frames = 90
        self.window_min_displacement = 0.02
        self.__history = deque(maxlen=self.window_frames)

        self.__position = Position2D(0, 0)
        self.__previous_position = Position2D(0, 0)
        self.__wheel_direction = 0      # 바퀴의 평균 각속도 (0이면 정지 중)

    def update(self, position, previous_position, wheel_direction):
        """매 타임스텝마다 호출하여 끼임 여부를 업데이트합니다."""
        self.__wheel_direction = wheel_direction
        self.__position = position
        self.__previous_position = previous_position
        self.__history.append(Position2D(position.x, position.y))

        # 이번 스텝에 끼인 것으로 보이면 카운터 증가, 아니면 초기화
        if self.__is_stuck_this_step():
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

    def reset(self):
        """이력/카운터 초기화. 의도된 정지 상태(보고/재캘리브/wiggle) 진입·복귀 시 호출해
        직전 정지 이력이 explore 복귀 직후 오탐을 내는 것을 방지한다."""
        self.__history.clear()
        self.stuck_counter = 0

    def is_stuck(self):
        """per-step 카운터 초과 또는 창 순이동 미달이면 True → Executor가 'stuck' 전환"""
        return self.stuck_counter > self.stuck_threshold or self.__is_stuck_window()

    def __is_stuck_window(self):
        """창이 가득 찼고, 창 시작↔끝 순이동이 임계 미만이면 정체로 판단합니다."""
        if len(self.__history) < self.window_frames:
            return False
        return self.__history[0].get_distance_to(self.__history[-1]) < self.window_min_displacement

    def __is_stuck_this_step(self):
        """이번 한 스텝에서 끼인 상태인지 판단: 바퀴가 돌고 있는데 위치 변화가 거의 없으면 끼인 것"""
        distance_traveled = self.__position.get_distance_to(self.__previous_position)
        is_rotating_wheels = self.__wheel_direction > 0
        return is_rotating_wheels and distance_traveled < self.minimum_distance_traveled
