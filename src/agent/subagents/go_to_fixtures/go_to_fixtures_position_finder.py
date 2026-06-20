import numpy as np
import cv2 as cv
from data_structures.vectors import Position2D

from algorithms.np_bool_array.bfs import NavigatingLimitedBFSAlgorithm, NavigatingBFSAlgorithm

from agent.agent_interface import PositionFinderInterface
from mapping.mapper import Mapper


class PositionFinder(PositionFinderInterface):
    """
    조난자(victim)가 감지된 위치 주변(zone of influence)에서
    아직 방문하지 않은 가장 가까운 목표 위치를 BFS로 탐색하는 클래스입니다.

    동작 원리:
    1. victims 레이어에 마킹된 조난자 위치에 원형 커널(zone of influence) 적용
    2. fixture_distance_margin과 AND 연산 → 유효한 도달 가능 후보 영역 도출
    3. 이미 지나간(robot_center_traversed) 위치 제거
    4. BFS로 현재 로봇 위치에서 가장 가까운 후보 선택 (탐색 한도: 1000)
    """
    def __init__(self, mapper: Mapper) -> None:
        self.__mapper = mapper
        # 탐색 한도 1000으로 제한된 BFS (후보: fixture_distance_margin & 조난자 영역, 통과: 벽 아닌 곳)
        self.__next_position_finder = NavigatingLimitedBFSAlgorithm(lambda x: x, lambda x: not x, limit=1000)
        # 목표가 아직 도달 가능한지 확인하는 BFS (traversed 레이어에서 연결 여부 판별)
        self.__still_reachable_bfs = NavigatingLimitedBFSAlgorithm(lambda x: x, lambda x: not x, limit=500)
        self.__target = None
        self.__target_age = 0
        self.__blacklisted_grid_indices: list = []
        self.__target_is_fixture = False
        # 보고 못 한 채 최적 보고위치까지 도달해 포기한 토큰(victim/hazmat)의 grid index.
        # 그리드 확장에 안전하도록 array 좌표가 아닌 grid index로 보관.
        self.__abandoned_fixture_centers: list = []

        # 조난자 영향 반경: 로봇 반경 + 여유 3픽셀
        circle_radius = round(self.__mapper.robot_diameter / 2 * self.__mapper.pixel_grid.resolution) + 3
        self.__zone_radius = circle_radius

        # 원형 커널 생성 - 조난자 주변 원 영역을 채워 zone of influence 계산에 사용
        self.circle_kernel = np.zeros((circle_radius * 2, circle_radius * 2), dtype=np.uint8)
        self.circle_kernel = cv.circle(self.circle_kernel, (circle_radius, circle_radius), circle_radius, 1, -1)
        self.__safety_kernel = np.ones((5, 5), dtype=np.uint8)
        self.__traversable_dilated_cache = None
        self.__traversable_shape_cache = None


    def update(self, force_calculation=False) -> None:
        """
        다음 조건 중 하나라도 해당하면 목표 위치를 재계산합니다:
        - 목표가 없음
        - 목표까지 경로가 막혔음
        - 로봇이 이미 그 위치를 지나쳤음
        - 강제 재계산 요청
        """
        # 옵션 A — orbit 방지: 토큰의 BFS 최적 보고위치(타겟)까지 로봇 중심이 실제
        # 도달했는데도 아직 미보고면(보고됐으면 그 zone은 robot_detected_fixture_from
        # 으로 이미 제외돼 타겟이 안 됐을 것), 그 위치에선 보고 불가로 판단하고 토큰
        # 전체를 포기한다. 보고 가능한 토큰은 접근 도중 report_fixture가 self-clear
        # 하므로 여기 안 걸린다 → 진짜 보고가능 피해자는 잃지 않음.
        if (self.target_position_exists() and self.__target_is_fixture and
                self.__already_passed_through_grid_index(self.__target)):
            self.__abandon_fixture_at(self.__target)

        needs_recalc = (
            not self.target_position_exists() or
            not self.__is_grid_index_still_reachable(self.__target) or
            self.__already_passed_through_grid_index(self.__target) or
            force_calculation
        )

        if needs_recalc:
            self.__target_age = 0
        else:
            self.__target_age += 1
            # 한도 450프레임(~15초): 60(~2초)은 토큰까지의 통상 이동시간(10초+)보다 짧아
            # 도착 전에 전부 블랙리스트됨(실런: 맵핑만 하고 보고 안 함). ★시뮬 튜닝
            if self.__target_age >= 450:
                self.__blacklisted_grid_indices.append(self.__target)
                self.__target = None
                self.__target_age = 0
                needs_recalc = True

        if needs_recalc:
            self.__calculate_position()

    def get_target_position(self) -> Position2D:
        """현재 목표 위치의 실제 좌표(m)를 반환합니다."""
        if self.target_position_exists():
            return self.__mapper.pixel_grid.grid_index_to_coordinates(self.__target)

    def target_position_exists(self) -> bool:
        return self.__target is not None

    def __calculate_position(self):
        """
        조난자(토큰) 주변 영역을 최우선 후보로 BFS 탐색하고,
        조난자 후보가 없거나 도달 불가일 때만 체크포인트를 후보로 사용합니다.
        → 토큰 보고가 체크포인트(포인트) 방문보다 항상 우선합니다.
        """
        current_shape = self.__mapper.pixel_grid.arrays["traversable"].shape
        if self.__traversable_dilated_cache is None or current_shape != self.__traversable_shape_cache:
            self.__traversable_dilated_cache = cv.dilate(
                self.__mapper.pixel_grid.arrays["traversable"].astype(np.uint8),
                self.__safety_kernel
            ).astype(np.bool_)
            self.__traversable_shape_cache = current_shape

        robot_array_index = self.__mapper.pixel_grid.grid_index_to_array_index(self.__mapper.robot_grid_index)

        # 우선순위 1: 조난자 zone of influence / 우선순위 2: 체크포인트.
        # 제거 필터가 다름: 토큰은 '보고한 곳'(robot_detected_fixture_from)만 제외 —
        # 지나가기만 한(traversed) 곳을 제외하면 벽 따라 스치기만 해도 미보고 토큰이
        # 영구 스킵됨(실런 증상). 체크포인트는 '방문 = 완료'이므로 traversed 제외 유지.
        candidate_groups = (
            (self.__get_fixtures_zone_of_influence(),
             self.__mapper.pixel_grid.arrays["robot_detected_fixture_from"]),
            (self.__mapper.pixel_grid.arrays["checkpoints"].copy(),
             self.__mapper.pixel_grid.arrays["robot_center_traversed"]),
        )

        for group_index, (possible_targets_array, exclude_mask) in enumerate(candidate_groups):
            possible_targets_array[exclude_mask] = False
            possible_targets_array[self.__traversable_dilated_cache] = False

            # 포기한 토큰(group 0=fixtures)의 zone 전체 제외 — 옆 crescent 재선택 차단.
            if group_index == 0:
                self.__apply_abandoned_exclusion(possible_targets_array)

            for gi in self.__blacklisted_grid_indices:
                ai = self.__mapper.pixel_grid.grid_index_to_array_index(gi)
                r, c = int(ai[0]), int(ai[1])
                if 0 <= r < possible_targets_array.shape[0] and 0 <= c < possible_targets_array.shape[1]:
                    possible_targets_array[r, c] = False

            if not np.any(possible_targets_array):
                continue

            # BFS로 탐색 가능한 가장 가까운 후보 선택 (최대 1000 루프 제한)
            results = self.__next_position_finder.bfs(possible_targets_array, self.__mapper.pixel_grid.arrays["traversable"], robot_array_index)
            if len(results):
                self.__target = self.__mapper.pixel_grid.array_index_to_grid_index(results[0])
                self.__target_age = 0
                self.__target_is_fixture = (group_index == 0)
                return

        self.__target = None
        self.__target_age = 0
        self.__target_is_fixture = False


    def __is_grid_index_still_reachable(self, grid_index) -> bool:
        """목표 위치가 traversable 영역에 있지 않고, traversed 경로와 연결되어 있는지 확인합니다."""
        start_array_index = self.__mapper.pixel_grid.grid_index_to_array_index(grid_index)

        # 목표가 통과 불가 영역이면 재계산 필요
        if self.__mapper.pixel_grid.arrays["traversable"][start_array_index[0], start_array_index[1]]:
             return False

        # traversed(지나간 경로) 레이어에서 목표까지 BFS로 연결 여부 확인
        results = self.__still_reachable_bfs.bfs(self.__mapper.pixel_grid.arrays["traversed"], self.__mapper.pixel_grid.arrays["traversable"], start_array_index)

        return bool(len(results))

    def __already_passed_through_grid_index(self, grid_index):
        """로봇 중심이 이미 이 위치를 지나쳤으면 True 반환 (목표 재탐색 필요)."""
        array_index = self.__mapper.pixel_grid.grid_index_to_array_index(grid_index)

        return self.__mapper.pixel_grid.arrays["robot_center_traversed"][array_index[0], array_index[1]]


    def __get_fixtures_zone_of_influence(self) -> np.ndarray:
        """
        victims 레이어에 원형 커널을 컨볼루션하여 각 조난자 주변의 원형 영역을 생성하고,
        fixture_distance_margin 영역과 AND 연산하여 최종 후보 영역을 반환합니다.
        """
        # victims 배열에 원형 커널 필터를 적용하여 zone of influence 계산
        zones = cv.filter2D(self.__mapper.pixel_grid.arrays["victims"].astype(np.uint8), -1, self.circle_kernel).astype(np.bool_)

        # 벽 근처 마진 영역과만 교집합 → 로봇이 실제로 도달 가능한 위치로 한정
        return np.bitwise_and(zones, self.__mapper.pixel_grid.arrays["fixture_distance_margin"])

    def __abandon_fixture_at(self, target_grid_index) -> None:
        """타겟(최적 보고위치) 주변의 victim/hazmat 포인트를 '포기 토큰'으로 등록합니다.
        타겟은 zone of influence 안에 있으므로 zone 반경 내에 원인 토큰이 존재합니다."""
        ai = self.__mapper.pixel_grid.grid_index_to_array_index(target_grid_index)
        r, c = int(ai[0]), int(ai[1])
        rad = self.__zone_radius
        h, w = self.__mapper.pixel_grid.array_shape
        r0, r1 = max(r - rad, 0), min(r + rad + 1, h)
        c0, c1 = max(c - rad, 0), min(c + rad + 1, w)
        victims = self.__mapper.pixel_grid.arrays["victims"]
        hazmats = self.__mapper.pixel_grid.arrays["hazmats"]
        window = victims[r0:r1, c0:c1] | hazmats[r0:r1, c0:c1]
        for (wr, wc) in np.argwhere(window):
            gi = self.__mapper.pixel_grid.array_index_to_grid_index(np.array([r0 + int(wr), c0 + int(wc)]))
            # numpy 배열은 `in` 비교가 element-wise라 모호 — 해시 가능한 튜플로 보관.
            gi = (int(gi[0]), int(gi[1]))
            if gi not in self.__abandoned_fixture_centers:
                self.__abandoned_fixture_centers.append(gi)

    def __apply_abandoned_exclusion(self, possible_targets_array: np.ndarray) -> None:
        """포기한 토큰들의 zone of influence(반경 = zone_radius 원)를 후보에서 제거합니다."""
        rad = self.__zone_radius
        h, w = possible_targets_array.shape
        for gi in self.__abandoned_fixture_centers:
            ai = self.__mapper.pixel_grid.grid_index_to_array_index(np.array(gi))
            r, c = int(ai[0]), int(ai[1])
            r0, r1 = max(r - rad, 0), min(r + rad, h)
            c0, c1 = max(c - rad, 0), min(c + rad, w)
            if r0 >= r1 or c0 >= c1:
                continue
            kr0, kc0 = r0 - (r - rad), c0 - (c - rad)
            kernel = self.circle_kernel[kr0:kr0 + (r1 - r0), kc0:kc0 + (c1 - c0)].astype(np.bool_)
            possible_targets_array[r0:r1, c0:c1][kernel] = False
