import math

from robot.devices.gps import Gps
from robot.devices.gyroscope import Gyroscope
from data_structures.angle import Angle
from data_structures.vectors import Position2D

from flags import SHOW_DEBUG


class PoseManager:
    """
    로봇의 위치(Position)와 방향(Orientation)을 통합 관리하는 클래스입니다.

    GPS와 자이로스코프 두 센서를 상황에 따라 선택적으로 사용합니다.
    - 직진 중 (회전 없음, 고속): GPS가 더 정확 → GPS 사용
    - 회전 중 또는 늪지대: GPS 오차 큼 → 자이로스코프 사용
    이 선택은 robot_is_going_straight() 조건으로 자동 판단하거나 수동 지정할 수 있습니다.
    """
    GPS = 0        # GPS 센서 식별자
    GYROSCOPE = 1  # 자이로스코프 센서 식별자

    def __init__(self, gps: Gps, gyroscope: Gyroscope, position_offsets=Position2D(0, 0)) -> None:
        # 직진 중이라고 판단하는 최대 각속도 임계값
        self.maximum_angular_velocity_for_gps = Angle(1, Angle.DEGREES)
        # 직진으로 인정하는 좌우 바퀴 속도차 상한(rad/s). 이보다 크면 곡선으로 보고 GPS heading 불신.
        # 곡선 구간에서 GPS 위치델타(현, chord) 방향을 heading으로 오인 → 벽이 회전 오차로 찍힘.
        # 기존 1.0(=ratio 0.16)은 완만한 arc를 직진으로 통과시킴. 보수적으로 낮춤. ★시뮬에서 튜닝 필요.
        self.maximum_wheel_difference_for_straight = 0.5
        # 흔들림(shaky) 판단에 사용하는 각속도 변화량 임계값
        self.maximum_angular_velocity_change_for_shaky = Angle(1, Angle.DEGREES)
        # GPS heading과 자이로가 이보다 크게 어긋나면 동기화 거부(곡선 구간의 chord 오측정으로 간주).
        # 완만한 곡선이 직진 판정을 통과해 GPS가 잘못된 heading을 자이로에 박는 갑작스런 점프를 막는다.
        self.max_gps_gyro_sync_diff = Angle(30, Angle.DEGREES)

        self.gps = gps
        self.gyroscope = gyroscope

        self.orientation = Angle(0)
        self.previous_orientation = Angle(0)

        self.__position = Position2D(0, 0)
        self.__previous_position = Position2D(0, 0)

        self.orientation_sensor = self.GYROSCOPE          # 기본값: 자이로스코프 사용
        self.previous_orientation_sensor = self.GYROSCOPE
        self.automatically_decide_orientation_sensor = True  # True면 자동 선택, False면 수동 지정

        self.position_offsets = position_offsets  # 시작점 오프셋 보정값

        self.shaky_threshold = Angle(5, unit=Angle.DEGREES)  # 흔들림 판단 각도 임계값

        # GPS baseline 구간에서 허용하는 누적 회전 상한. 매-스텝 직진판정만으로는 완만한 곡선이
        # baseline을 가로질러(예: 0.9°/step×40 = 36°) GPS chord가 실제 heading과 어긋난다.
        # baseline 이후 자이로가 이만큼 돌면 baseline을 리셋해 GPS heading을 직선 구간으로 한정.
        self.max_curve_for_gps_baseline = Angle(8, Angle.DEGREES)
        self.__gyro_at_baseline = Angle(0)  # 마지막 baseline 리셋 시점의 자이로 방향

        # ★핵심 수정(원인 가): report_fixture의 짧은 접근 이동(수 프레임)에서 GPS chord가
        # 노이즈에 지배돼 5~22° 잘못된 heading을 자이로에 박는 것을 로그로 확인.
        # 진짜 긴 직진(복도)일 때만 GPS를 신뢰하도록, 연속 직진이 임계 프레임 이상일 때만 GPS 사용.
        # report_fixture 접근(~5프레임 이하)은 임계 미달 → 자이로 유지 → heading 오염 차단.
        self.consecutive_straight_frames = 0
        self.min_straight_frames_for_gps = 8  # ★시뮬 튜닝

        # GPS 백스톱 = 상보필터(부분보정) + 데드밴드. 완전덮어쓰기(set_orientation(gps)) 대신
        # 작은차(노이즈)는 무시하고, 진짜 드리프트(5~30°)만 게인만큼 부드럽게 끈다.
        # 회전 감속으로 자이로가 정확해진 상태에서 완전덮어쓰기는 노이즈를 주입하므로.
        self.gps_correction_gain = 0.05                          # 부분보정 게인(5%)
        self.gps_correction_deadband = Angle(5, Angle.DEGREES)   # 이하 차이는 노이즈로 무시
        # 상단 가드는 self.max_gps_gyro_sync_diff(30°) 재사용: 이상이면 chord 오측정으로 거부.

        # ── 임시 계측(진단용, 확정 후 제거): GPS 백스톱이 왜 침묵하는지 추적 ──
        # sustained-straight 게이트(8프레임)서 막히나, baseline(0.08m None)서 막히나 구분.
        self.__diag_frames = 0
        self.__diag_gps_selected = 0        # decide에서 sensor=GPS 된 횟수
        self.__diag_gps_no_baseline = 0     # sensor=GPS인데 get_orientation()=None(baseline 부족)
        self.__diag_gps_no_baseline_moving = 0  # 그중 로봇이 실제 이동 중이었던 횟수(끼임 아님)
        self.__diag_baseline_dist_max = 0.0     # 관측된 최대 baseline 도달거리(m)
        self.__diag_max_straight = 0        # 관측된 최대 연속 직진 프레임

    def update(self, average_wheel_velocity, wheel_velocity_difference):
        """
        매 타임스텝 호출: GPS와 자이로스코프를 갱신하고 현재 위치·방향을 계산합니다.
        average_wheel_velocity: 양 바퀴 평균 각속도 (직진 여부 판단용)
        wheel_velocity_difference: 좌우 바퀴 속도 차 (회전 여부 판단용)
        """
        self.gps.update()
        self.gyroscope.update()

        # GPS로 전역 위치 갱신
        self.__previous_position = self.__position
        self.__position = self.gps.get_position()

        # 조건에 따라 방향 센서 자동 선택
        if self.automatically_decide_orientation_sensor:
            self.decide_orientation_sensor(average_wheel_velocity, wheel_velocity_difference)

        self.previous_orientation = self.orientation
        self.calculate_orientation()

        # ── 임시 계측 요약(진단용, 확정 후 제거) ──
        self.__diag_frames += 1
        if self.__diag_frames % 200 == 0:
            print(f"[heading계측:pose_manager] frames={self.__diag_frames}, "
                  f"GPS선택={self.__diag_gps_selected}회, "
                  f"baseline부족(None)={self.__diag_gps_no_baseline}회(그중 이동중={self.__diag_gps_no_baseline_moving}), "
                  f"baseline최대도달={self.__diag_baseline_dist_max:.3f}m(임계 {self.gps.min_baseline_distance:.2f}), "
                  f"최대연속직진={self.__diag_max_straight}프레임(임계 {self.min_straight_frames_for_gps})")

    def decide_orientation_sensor(self, average_wheel_velocity, wheel_velocity_difference):
        """직진 중이면 GPS, 회전 중이면 자이로스코프를 방향 센서로 선택합니다."""
        # baseline 이후 누적 회전이 크면(완만한 곡선이 매-스텝 직진판정을 통과한 경우)
        # baseline을 리셋해 GPS chord가 곡선 구간을 가로지르지 않게 한다.
        gyro_now = self.gyroscope.get_orientation()
        if gyro_now.get_absolute_distance_to(self.__gyro_at_baseline) > self.max_curve_for_gps_baseline:
            self.gps.reset_orientation_baseline()
            self.__gyro_at_baseline = gyro_now

        if self.robot_is_going_straight(average_wheel_velocity, wheel_velocity_difference):
            # 직진 누적. 충분히 길게(노이즈가 baseline에 평균화될 만큼) 직진했을 때만 GPS 신뢰.
            # 임계 미달이면 자이로 유지하되 baseline은 리셋하지 않아 계속 누적 → 임계 도달 시 baseline이 길다.
            self.consecutive_straight_frames += 1
            if self.consecutive_straight_frames > self.__diag_max_straight:
                self.__diag_max_straight = self.consecutive_straight_frames
            if self.consecutive_straight_frames >= self.min_straight_frames_for_gps:
                self.orientation_sensor = self.GPS
                self.__diag_gps_selected += 1
            else:
                self.orientation_sensor = self.GYROSCOPE
        else:
            self.consecutive_straight_frames = 0
            self.orientation_sensor = self.GYROSCOPE
            # 직진이 아니면 GPS 방향 기준선을 초기화하여, 기준선이 회전 구간을
            # 가로질러 잘못된 방향을 계산하는 것을 방지합니다.
            self.gps.reset_orientation_baseline()
            self.__gyro_at_baseline = gyro_now

    def robot_is_going_straight(self, average_wheel_velocity, wheel_velocity_difference) -> bool:
        """로봇이 직진 중인지 판단: 각속도 작고, 속도 충분하고, 좌우 속도 차 작으면 직진"""
        return self.gyroscope.get_angular_velocity() < self.maximum_angular_velocity_for_gps and \
               average_wheel_velocity >= 1 and \
               abs(wheel_velocity_difference) < self.maximum_wheel_difference_for_straight

    def calculate_orientation(self):
        """선택된 센서로 현재 방향 각도를 계산합니다. GPS 사용 시 자이로도 동기화합니다."""
        gps_orientation = self.gps.get_orientation()

        if self.orientation_sensor == self.GYROSCOPE or gps_orientation is None:
            # 계측: 게이트는 통과(sensor=GPS)했는데 baseline 부족(None)으로 GPS가 죽는 경우 집계.
            if self.orientation_sensor == self.GPS and gps_orientation is None:
                self.__diag_gps_no_baseline += 1
                d = self.gps.get_max_baseline_distance()
                if d > self.__diag_baseline_dist_max:
                    self.__diag_baseline_dist_max = d
                if self.__position.get_distance_to(self.__previous_position) > 0.0005:
                    self.__diag_gps_no_baseline_moving += 1
            self.orientation = self.gyroscope.get_orientation()
            if SHOW_DEBUG: print(f"[방향센서:pose_manager.calculate_orientation] 자이로스코프 사용: 방향={self.orientation.degrees:.1f}°, 각속도={self.gyroscope.get_angular_velocity().degrees:.2f}°/step")
        else:
            gyro_orientation = self.gyroscope.get_orientation()
            # 최단 부호차(–π~π): GPS heading이 자이로보다 어느 쪽으로 얼마나 어긋났나.
            gyro_rad = gyro_orientation.radians
            gps_rad = gps_orientation.radians
            delta = math.atan2(math.sin(gps_rad - gyro_rad), math.cos(gps_rad - gyro_rad))
            diff_deg = abs(math.degrees(delta))

            if diff_deg < self.gps_correction_deadband.degrees:
                # 노이즈 수준 차이 → 자이로 신뢰(보정 안 함). 정확한 자이로에 GPS 노이즈 안 박음.
                self.orientation = gyro_orientation
            elif diff_deg >= self.max_gps_gyro_sync_diff.degrees:
                # 거대 불일치 → GPS chord 오측정 의심 → 거부, 자이로 유지.
                self.orientation = gyro_orientation
                if SHOW_DEBUG: print(f"[방향센서:pose_manager.calculate_orientation] ⚠ GPS heading({gps_orientation.degrees:.1f}°)이 자이로({gyro_orientation.degrees:.1f}°)와 {diff_deg:.1f}° 차이 → 곡선 chord 의심, 동기화 거부")
            else:
                # 진짜 드리프트 의심(5~30°) → 게인만큼 부분 보정(상보필터). 노이즈는 여러 프레임에 평균돼 사라짐.
                corrected = Angle(gyro_rad + self.gps_correction_gain * delta)
                corrected.normalize()
                self.orientation = corrected
                self.gyroscope.set_orientation(self.orientation)
                # [진단] 부분보정 발생 — 깨끗한 런이면 긴직진서만 드물게 떠야 정상.
                # report_fixture 등에서 자주 뜨면 8프레임 게이트 실패(원인 가 재발).
                print(f"[heading진단:pose_manager] GPS부분보정 {self.gps_correction_gain * diff_deg:.2f}° "
                      f"(자이로 {gyro_orientation.degrees:.1f}° vs GPS {gps_orientation.degrees:.1f}°, 차이 {diff_deg:.1f}°), "
                      f"위치=({self.__position.x:.3f},{self.__position.y:.3f})m")
                if SHOW_DEBUG: print(f"[방향센서:pose_manager.calculate_orientation] GPS 부분보정 사용: 방향={self.orientation.degrees:.1f}°")

    @property
    def position(self):
        """오프셋이 적용된 현재 위치를 반환합니다."""
        return self.__position + self.position_offsets

    @property
    def raw_position(self):
        """오프셋이 적용되지 않은 GPS 원시 위치를 반환합니다 (서버 전송용)."""
        return self.__position

    @property
    def previous_position(self):
        """오프셋이 적용된 이전 타임스텝의 위치를 반환합니다."""
        return self.__previous_position + self.position_offsets

    def is_shaky(self) -> bool:
        """
        로봇이 심하게 흔들리거나 급격히 회전 중인지 판단합니다.
        True이면 라이다 데이터 신뢰도가 낮으므로 맵 업데이트를 일부 생략합니다.
        - 방향 변화가 크거나
        - 각속도 방향이 바뀌었거나
        - 각속도 크기 변화가 크면 → 흔들리는 것으로 판단
        """
        high_orient_diff = self.orientation.get_absolute_distance_to(self.previous_orientation) > self.shaky_threshold
        changed_direction = self.gyroscope.angular_velocity * self.gyroscope.previous_angular_velocity < 0
        high_angular_velocity_difference = self.gyroscope.previous_angular_velocity.get_absolute_distance_to(
            self.gyroscope.angular_velocity) > self.maximum_angular_velocity_change_for_shaky

        return high_orient_diff or changed_direction or high_angular_velocity_difference
