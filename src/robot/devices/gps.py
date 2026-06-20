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
        self.__prev_position = Position2D()   # 이전 타임스텝의 위치 (방향 계산용)
        self.position = self.get_position()   # 현재 위치 초기화

    def update(self):
        """매 타임스텝 호출: GPS 값을 갱신합니다."""
        self.__prev_position = self.position
        self.position = self.get_position()

    def get_position(self):
        """GPS 센서에서 현재 전역 위치(x, y)를 읽어 반환합니다. (Webots의 z축 → y축으로 변환)"""
        vals = self.device.getValues()
        return Position2D(vals[0] * self.multiplier, vals[2] * self.multiplier)

    def get_orientation(self):
        """
        이전 위치에서 현재 위치로의 방향 벡터를 이용해 진행 각도를 반환합니다.
        로봇이 충분히 이동했고 직진 중일 때만 정확합니다.
        이동이 없거나 너무 짧으면 None 반환 → PoseManager가 자이로스코프로 대체합니다.
        """
        if self.__prev_position != self.position:
            accuracy = abs(self.position.get_distance_to(self.__prev_position))
            if accuracy > 0.001:  # 0.001m(1mm) 이상 이동했을 때만 방향 계산
                angle = self.__prev_position.get_angle_to(self.position)
                angle.normalize()
                return angle
        return None
