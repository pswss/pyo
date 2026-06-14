from data_structures.vectors import Position2D

from robot.devices.sensor import Sensor

class Gps(Sensor):
    """
    Webots GPS 센서를 이용해 로봇의 전역 위치(x, z → x, y)를 추적하는 클래스입니다.

    또한 두 타임스텝 사이의 위치 차이를 이용해 로봇이 직진할 때의 진행 방향(각도)을
    계산할 수 있습니다. 단, 직진 중일 때만 신뢰도가 높습니다.
    """
    def __init__(self, webots_device, time_step, coords_multiplier=1):
        super().__init__(webots_device, time_step)
        self.multiplier = coords_multiplier   # 좌표 스케일링 인수 (기본값 1)
        self.position = self.get_position()   # 현재 위치 초기화

        # v26부터 GPS에 가우시안 노이즈(표준편차 ≈ 2.5mm)가 추가되었다.
        # 연속한 두 위치의 차이는 노이즈가 신호를 압도하므로(이동 ~4mm vs 노이즈차 ~3.5mm),
        # 노이즈보다 충분히 큰 거리를 이동한 두 점으로만 방향을 계산해야 신뢰할 수 있다.
        self.__position_history = [self.position]   # 방향 계산용 위치 이력 (직진 구간만 누적)
        self.__max_history = 40
        # 방향 계산용 최소 이동 거리(m). 노이즈 σ≈2.5mm → 이 거리에서 heading 각오차 ≈ atan(σ/d).
        # 0.04m면 ±5°로 너무 커서 짧은 baseline(예: report_fixture 접근)에서 chord가 노이즈에
        # 지배돼 자이로를 오염시켰다(로그 확인, 보정 20°+). 0.08m면 ±1.8°로 줄어든다. ★시뮬 튜닝.
        self.min_baseline_distance = 0.08
        # 초기 캘리브레이션 전용 baseline. 캘리브는 ~5cm 후진/전진뿐이라 주행용(0.08m) 게이트로는
        # get_orientation_robust가 None을 뱉어 자이로 영점이 안 잡힌다(→ world1 90° 회전 회귀).
        # 캘리브와 주행은 baseline 요구가 정반대(짧음 vs 노이즈강건)라 분리한다. Follow-up3에서
        # 0.02m 게이트로 캘리브 성공·점수 획득 확인됨. 주행 noise robustness와 독립.
        self.min_calibration_baseline = 0.02

    def update(self):
        """매 타임스텝 호출: GPS 값을 갱신하고 방향 계산용 위치 이력을 누적합니다."""
        self.position = self.get_position()
        self.__position_history.append(self.position)
        if len(self.__position_history) > self.__max_history:
            self.__position_history.pop(0)

    def reset_orientation_baseline(self):
        """직진이 끊겼을 때(회전 등) 호출하여 방향 계산용 위치 이력을 초기화합니다.
        이력이 회전 구간을 가로지르지 않게 하여 직진 구간만으로 방향을 계산합니다."""
        self.__position_history = [self.position]

    # Pj copy 3 pose_manager 호환 별칭 (동일 동작: 위치 이력 초기화).
    def reset_orientation_reference(self):
        self.reset_orientation_baseline()

    def get_position(self):
        """GPS 센서에서 현재 전역 위치(x, y)를 읽어 반환합니다. (Webots의 z축 → y축으로 변환)"""
        vals = self.device.getValues()
        return Position2D(vals[0] * self.multiplier, vals[2] * self.multiplier)

    def get_orientation(self):
        """
        min_baseline_distance 이상 떨어진 가장 최근 과거 위치에서 현재 위치로의
        방향을 반환합니다. 긴 기준선으로 GPS 노이즈를 평균화하여 신뢰할 수 있는
        방향을 얻습니다. 이동이 부족하면 None을 반환 → PoseManager가 자이로스코프로 대체합니다.
        """
        for past_position in reversed(self.__position_history[:-1]):
            if self.position.get_distance_to(past_position) >= self.min_baseline_distance:
                angle = past_position.get_angle_to(self.position)
                angle.normalize()
                return angle
        return None

    def get_orientation_robust(self):
        """노이즈 강건 heading: 위치 이력 앞 절반 평균(centroid) → 뒤 절반 평균 방향.
        단일 두 점 대신 다수 샘플을 평균해 GPS 노이즈를 ~1/√N로 줄인다.
        초기 자이로 영점 캘리브레이션처럼 한 번의 측정이 전체 런 heading 기준으로
        고정되는 경우에 사용한다. 이동이 부족하면 None → 호출부가 폴백 처리."""
        history = self.__position_history
        if len(history) < 4:
            return None
        half = len(history) // 2
        start = self.__average_positions(history[:half])
        end = self.__average_positions(history[half:])
        if start.get_distance_to(end) < self.min_calibration_baseline:
            return None
        angle = start.get_angle_to(end)
        angle.normalize()
        return angle

    def get_max_baseline_distance(self):
        """현재 위치에서 history 내 가장 먼 과거 위치까지 거리(m). 임시 진단용:
        baseline 게이트(0.08m)를 왜 못 넘는지 — 끼임(≈0) vs 짧음(예 0.05) 구분."""
        if len(self.__position_history) < 2:
            return 0.0
        return max(self.position.get_distance_to(p) for p in self.__position_history[:-1])

    def __average_positions(self, positions):
        """Position2D 목록의 평균 좌표(centroid)를 반환합니다."""
        total = Position2D(0, 0)
        for p in positions:
            total = total + p
        return total / len(positions)
