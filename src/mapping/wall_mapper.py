import numpy as np
import cv2 as cv
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid

from data_structures.vectors import Position2D

import skimage

class WallMapper:
    """
    라이다 포인트클라우드를 받아 픽셀 그리드에 벽/장애물을 등록하는 클래스입니다.

    처리 흐름:
    1. 범위 내 포인트(in_bounds) → 해당 픽셀의 detected_points 카운터 증가
       → 임계값(3회) 초과 시 walls 레이어에 True 등록
    2. 범위 초과 포인트(out_of_bounds) → seen_by_lidar 레이어에 빔 궤적 등록
    3. 카메라가 본 영역과 라이다 벽을 교차하여 walls_seen_by_camera 갱신
    4. 벽 주변에 로봇 크기만큼 통과 불가 영역(traversable) 생성
    5. 벽 근처에 낮은 선호도(navigation_preference) 부여하여 경로 탐색 시 벽 회피 유도
    """
    def __init__(self, compound_grid: CompoundExpandablePixelGrid, robot_diameter: float) -> None:
        self.grid = compound_grid

        compensation = 0
        # 로봇 직경을 픽셀 단위로 변환
        self.robot_diameter = int(robot_diameter * self.grid.resolution) + compensation * 2
        self.robot_radius = int(robot_diameter / 2 * self.grid.resolution) + compensation

        # 벽 확정 임계. 실측 라이다는 희소함(지나가며 잠깐씩, 원거리는 프레임당 1히트 →
        # 매 프레임 dp≤1 필터에 소거) — 6은 기아 상태로 내부 벽 대부분이 영영 확정 못 됨
        # (시뮬 스크린샷 확인). 5 = 오프라인 품질 하니스 그리드서치 최적값
        # (tests/tune_wall_knobs.py, avgF1 0.946 vs 이전 4/5/3 = 0.922). ★시뮬 튜닝
        self.to_boolean_threshold = 5  # 그리드서치 2026-06-06
        self.delete_threshold = 1      # 이 횟수 이하의 감지 포인트는 노이즈로 제거
        # detected_points 상한(폭주 방지). 벽이 더 안 맞으면 몇 프레임 내 해제되도록 작게 유지.
        self.max_detected_points = 10
        # 빈 공간 클리어링: 라이다 빔이 통과한(=비어있는) 셀은 매 프레임 이만큼 감소.
        # 0이 되면 잘못 찍힌/번진 벽을 해제(재맵핑). 실제 벽은 빔이 끝점에서 맞으므로(통과 아님) 감소 안 됨.
        # 0 = 클리어링 비활성 (선맵핑 신뢰). 후반부 포즈 드리프트가 누적되면 라이다 빔이
        # (어긋난 좌표계에서) 이미 잘 찍힌 벽 자리를 관통해 멀쩡한 벽을 통째로 지우는 게
        # 더 큰 해악으로 확인됨. Erebus 맵은 정적이므로 처음 확정된 벽이 가장 정확하다.
        # 오탐 방어는 확정 임계(to_boolean_threshold)가 담당. 드리프트로 넓어진 띠는
        # 정제 단계(thin_walls)가 수렴시킨다. ★시뮬 튜닝
        self.free_space_decrement = 0

        # 벽 정제 활성 여부 (0=비활성: walls_raw가 그대로 walls로 복사됨)
        self.wall_refine_enabled = 1
        # 정제 적용 반경(px): 라이다 사거리(0.48m)+여유. 시야 밖 벽은 건드리지 않음.
        self.thinning_window_radius = round(0.55 * self.grid.resolution)
        # 가로/세로 closing 커널 크기(px). 희소하게 확정된 벽 셀들을 벽 방향으로 이어붙임.
        # 커널 3px는 2px(12mm) 이하 틈만 메움 → 통로 안전 + 과대 closing이 드리프트 정밀도를
        # 깎는 것 방지 (그리드서치에서 커널 크기가 정밀도 지배 변수였음). ★시뮬 튜닝
        self.wall_close_kernel_px = 3  # 그리드서치 2026-06-06
        self.__close_kernel_h = np.ones((1, self.wall_close_kernel_px), np.uint8)
        self.__close_kernel_v = np.ones((self.wall_close_kernel_px, 1), np.uint8)
        # 이보다 작은 고립 부스러기(px 수)는 표시에서 제외 (증거 raw에는 남아서 더 확정되면 복귀)
        self.min_wall_fragment_px = 2  # 그리드서치 2026-06-06

        # 로봇 크기 원형 마스크 (navigation_preference 생성에 사용)
        self.robot_diameter_template = np.zeros((self.robot_diameter, self.robot_diameter), dtype=np.uint8)
        self.robot_diameter_template = cv.circle(self.robot_diameter_template,
                                                  (self.robot_radius, self.robot_radius),
                                                  self.robot_radius, 255, -1)
        self.robot_diameter_template = self.robot_diameter_template.astype(np.bool_)

        # BFS 경로탐색용 축소 마진 (통로 통과 가능하도록 반경 1px 축소)
        traversable_radius = max(self.robot_radius - 1, 1)
        traversable_diameter = traversable_radius * 2 + 1
        self.traversable_template = np.zeros((traversable_diameter, traversable_diameter), dtype=np.uint8)
        self.traversable_template = cv.circle(self.traversable_template,
                                               (traversable_radius, traversable_radius),
                                               traversable_radius, 255, -1)
        self.traversable_template = self.traversable_template.astype(np.bool_)

        # 경로 탐색 선호도 그라디언트 템플릿 (벽 근처일수록 높은 값 → 회피)
        self.preference_template = self.__generate_quadratic_circle_gradient(
            self.robot_radius, self.robot_radius * 1.7)

    def load_point_cloud(self, in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position):
        """
        라이다 데이터를 받아 seen_by_lidar 초기화 후 벽/여유 공간을 등록합니다.
        매 타임스텝마다 호출됩니다.
        """
        robot_position_as_array = np.array(robot_position, dtype=float)

        self.__reset_seen_by_lidar()  # 이전 프레임의 라이다 시야 초기화

        self.load_in_bounds_point_cloud(in_bounds_point_cloud, robot_position_as_array)
        self.load_out_of_bounds_point_cloud(out_of_bounds_point_cloud, robot_position_as_array)

        # 양쪽 빔(범위내+범위초과)을 모두 그린 뒤, 빔이 통과한 빈 공간으로 벽을 정리(재맵핑)
        self.clear_free_space()

        # 마지막으로 로봇 주변 벽 번짐을 얇게 깎음 (이후 occupied/최종맵이 얇은 벽을 사용)
        self.thin_walls(self.grid.coordinates_to_array_index(robot_position_as_array))

    def load_in_bounds_point_cloud(self, point_cloud, robot_position):
        """
        감지 범위 내 포인트들을 처리합니다:
        - 그리드 확장
        - 포인트 카운터 증가 및 벽 등록
        - 라이다 빔 궤적 기록
        - 노이즈 필터링
        - 통과 불가 영역 생성
        """
        for p in point_cloud:
            point = np.array(p, dtype=float) + robot_position

            point_grid_index = self.grid.coordinates_to_grid_index(point)
            self.grid.expand_to_grid_index(point_grid_index)

            robot_array_index = self.grid.coordinates_to_array_index(robot_position)
            point_array_index = self.grid.grid_index_to_array_index(point_grid_index)

            self.occupy_point(point_array_index)                                      # 벽 카운터 증가
            self.mark_point_as_seen_by_lidar(robot_array_index, point_array_index)   # 빔 궤적 기록

        self.filter_out_noise()            # 저빈도 포인트 제거
        self.generate_navigation_margins() # traversable + navigation_preference 계산

    def load_out_of_bounds_point_cloud(self, point_cloud, robot_position):
        """
        감지 범위 초과 방향의 포인트들을 처리합니다:
        - 열린 공간 방향 라이다 빔 궤적 기록
        - 카메라에서 본 벽 영역 계산 갱신
        """
        for p in point_cloud:
            point = np.array(p, dtype=float) + robot_position

            point_grid_index = self.grid.coordinates_to_grid_index(point)
            self.grid.expand_to_grid_index(point_grid_index)

            robot_array_index = self.grid.coordinates_to_array_index(robot_position)
            point_array_index = self.grid.grid_index_to_array_index(point_grid_index)

            self.mark_point_as_seen_by_lidar(robot_array_index, point_array_index)

        self.calculate_seen_walls()

    def calculate_seen_walls(self):
        """카메라 시야와 라이다 벽 감지를 교차하여 카메라로 본 벽/못 본 벽을 분류합니다."""
        self.grid.arrays["walls_seen_by_camera"] = (
            self.grid.arrays["seen_by_camera"] * self.grid.arrays["walls"])
        self.grid.arrays["walls_not_seen_by_camera"] = np.logical_xor(
            self.grid.arrays["walls"],
            self.grid.arrays["walls_seen_by_camera"])

    def generate_navigation_margins(self):
        """
        벽 주변으로 로봇 크기만큼의 통과 불가 영역(traversable)과
        경로 탐색 선호도 지도(navigation_preference)를 생성합니다.
        """
        occupied_as_int = self.grid.arrays["occupied"].astype(np.uint8)

        # traversable: 축소 마진으로 통로 통과 허용 (실제 회피는 navigation_preference가 담당)
        traversable_template_as_int = self.traversable_template.astype(np.uint8)
        self.grid.arrays["traversable"] = np.zeros_like(self.grid.arrays["traversable"])
        self.grid.arrays["traversable"] = cv.filter2D(occupied_as_int, -1, traversable_template_as_int)
        self.grid.arrays["traversable"] = self.grid.arrays["traversable"].astype(np.bool_)

        # 벽 근처 픽셀에 높은 선호도 값 부여 (경로 탐색 시 회피 유도)
        self.grid.arrays["navigation_preference"] = cv.filter2D(occupied_as_int, -1, self.preference_template)
        self.grid.arrays["navigation_preference"][self.grid.arrays["swamps"]] = 150  # 늪지대도 회피

    def filter_out_noise(self):
        """감지 횟수가 delete_threshold 이하인 포인트를 노이즈로 제거합니다."""
        self.grid.arrays["detected_points"] = (
            self.grid.arrays["detected_points"] *
            (self.grid.arrays["detected_points"] > self.delete_threshold))

    def __generate_quadratic_circle_gradient(self, min_radius, max_radius):
        """벽 근처일수록 값이 커지는 2차 원형 그라디언트 템플릿을 생성합니다."""
        min_radius = round(min_radius)
        max_radius = round(max_radius)
        template = np.zeros((max_radius * 2 + 1, max_radius * 2 + 1), dtype=np.float32)
        for i in range(max_radius, min_radius, -1):
            template = cv.circle(template, (max_radius, max_radius), i,
                                 max_radius ** 2 - i ** 2, -1)
        return template * 0.1

    def occupy_point(self, point_array_index):
        """
        해당 위치의 감지 카운터를 증가시키고(상한까지), 임계값 초과 시 벽으로 확정합니다.
        단, 로봇 '중심'이 지나간 곳(robot_center_traversed)은 벽으로 등록하지 않습니다.
        ※ 풋프린트 전체(traversed)를 기준으로 거부하면, 포즈 오차 1~2cm로 풋프린트
        가장자리가 1cm(1.7px) 두께 벽 라인을 덮는 순간 그 벽이 영구 등록 불가가 됨
        (실런 증상: 가까이 간 벽이 안 찍힘). 중심 궤적(~2cm)은 진짜 벽일 수 없는
        최소 영역만 거부한다.
        벽 확정 후에도 계속 증가시켜(빔이 계속 맞으면 보충) clear_free_space의 감소와 균형을 맞춘다.
        """
        i, j = point_array_index[0], point_array_index[1]
        dp = self.grid.arrays["detected_points"]
        if dp[i, j] < self.max_detected_points:
            dp[i, j] += 1

        # 절충 거부 규칙 (실런 2회로 캘리브):
        # - 중심 궤적: 무조건 거부 (그 자리에 벽 불가)
        # - 풋프린트(traversed): 기본 거부, 단 dp 상한 도달 셀은 허용 —
        #   무조건 거부면 포즈 오차로 근접 벽 영구 미등록, 중심만 거부면 통로 안쪽
        #   노이즈 끝점이 누적 등록(궤적 주변 얼룩 + 외벽 틈 → 웨이포인트 맵 밖 누수).
        #   상한 도달 = 지속 스캔된 강한 증거 = 진짜 벽. ★시뮬 튜닝
        if dp[i, j] > self.to_boolean_threshold:
            if self.grid.arrays["robot_center_traversed"][i, j]:
                return
            if (self.grid.arrays["traversed"][i, j]
                    and dp[i, j] < self.max_detected_points):
                return
            self.grid.arrays["walls_raw"][i, j] = True

    def thin_walls(self, robot_array_index):
        """walls_raw(증거)에서 표시·항법용 walls를 로봇 주변 윈도우에 새로 계산한다.

        핵심: walls는 walls_raw의 순수 함수 — 출력이 다음 프레임 입력에 섞이지 않는다
        (피드백 없음 → 프레임 누적 변형/드리프트 원천 차단).

        실측 라이다는 희소·단속적(벽을 지나가며 잠깐씩만 관측)이라 과한 기하 가공
        (축 직선 재투영, 스켈레톤화)은 실데이터에서 벽을 지우거나 가짜 줄무늬를
        만들었다(시뮬 확인). 보수적으로 두 가지만 한다:
        1) 가로/세로 closing — 희소 확정 셀을 벽 방향으로 이어붙임
        2) 고립 부스러기 제거 — 표시에서만 제외, 증거는 raw에 남음"""
        if self.wall_refine_enabled <= 0:
            return
        raw = self.grid.arrays["walls_raw"]
        walls = self.grid.arrays["walls"]
        r = self.thinning_window_radius
        r0 = max(robot_array_index[0] - r, 0)
        c0 = max(robot_array_index[1] - r, 0)
        r1 = min(robot_array_index[0] + r, walls.shape[0])
        c1 = min(robot_array_index[1] + r, walls.shape[1])
        window = raw[r0:r1, c0:c1]
        if not window.any():
            walls[r0:r1, c0:c1] = False
            return

        # 1) 가로/세로 closing: 같은 벽의 희소 확정 셀들을 이어붙여 연속된 벽으로
        window_int = window.astype(np.uint8)
        closed = (cv.morphologyEx(window_int, cv.MORPH_CLOSE, self.__close_kernel_h) |
                  cv.morphologyEx(window_int, cv.MORPH_CLOSE, self.__close_kernel_v))

        # 2) 고립 부스러기 제거 (증거가 더 쌓여 커지면 자연 복귀)
        out = closed.astype(np.bool_)
        n, labels, stats, _ = cv.connectedComponentsWithStats(closed)
        for i in range(1, n):
            if stats[i, cv.CC_STAT_AREA] < self.min_wall_fragment_px:
                out[labels == i] = False

        walls[r0:r1, c0:c1] = out

    def clear_free_space(self):
        """라이다 빔이 통과한(seen_by_lidar=비어있음) 셀의 detected_points를 감소시키고,
        0이 되면 그 자리의 벽을 해제한다(재맵핑). 흔들림으로 번진 벽/오탐 벽을 자가 수정.
        실제 벽은 빔의 끝점(통과 아님, seen_by_lidar 제외)이라 감소되지 않는다."""
        if self.free_space_decrement <= 0:
            return  # 클리어링 비활성
        free = self.grid.arrays["seen_by_lidar"]
        dp = self.grid.arrays["detected_points"]
        # 주의: dp는 uint16 — 0에서 빼면 65535로 언더플로되어 해당 셀이 노이즈 한 발에
        # 임계 무시하고 즉시 벽이 되는 버그가 있었음. dp>0인 셀만 골라 바닥 0으로 감소.
        dec = free & (dp > 0)
        dp[dec] = np.maximum(dp[dec].astype(np.int32) - self.free_space_decrement, 0).astype(dp.dtype)
        cleared = free & (dp == 0) & self.grid.arrays["walls_raw"]
        self.grid.arrays["walls_raw"][cleared] = False

    def mark_point_as_seen_by_lidar(self, robot_array_index, point_array_index):
        """로봇 위치에서 포인트까지의 선을 seen_by_lidar 레이어에 그립니다."""
        self.grid.arrays["seen_by_lidar"] = self.__draw_bool_line(
            self.grid.arrays["seen_by_lidar"], robot_array_index, point_array_index)

    def __draw_bool_line(self, array, point1, point2):
        """Bresenham 직선 알고리즘으로 두 점 사이의 직선 경로를 True로 표시합니다."""
        indexes = skimage.draw.line(point1[0], point1[1], point2[0], point2[1])
        array[indexes[0][:-2], indexes[1][:-2]] = True  # 끝 2픽셀 제외 (포인트 자체는 제외)
        return array

    def __reset_seen_by_lidar(self):
        """매 프레임 시작 시 라이다 시야 레이어를 초기화합니다."""
        self.grid.arrays["seen_by_lidar"] = np.zeros_like(self.grid.arrays["seen_by_lidar"])
