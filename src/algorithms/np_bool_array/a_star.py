import numpy as np
import cv2 as cv
import math

# A* 알고리즘의 각 노드를 나타내는 클래스
# 기본 A* 구현 (리스트 기반, 느림) - efficient_a_star.py의 힙 기반 구현을 권장합니다
class aStarNode():
    def __init__(self, parent=None, position=None):
        self.parent = parent    # 경로 역추적을 위한 부모 노드
        self.position = position
        self.g = 0  # 시작점에서 이 노드까지의 실제 비용
        self.h = 0  # 이 노드에서 목표까지의 휴리스틱 추정 비용
        self.p = 0  # 선호도(preference) 패널티 (벽 근처 경로 회피용)
        self.f = 0  # f = g + h + p (총 비용)

    def __eq__(self, other):
        return self.position == other.position

    def __repr__(self):
        return str(self.position)

class aStarAlgorithm:
    """
    기본 A* 경로 탐색 알고리즘입니다.
    open list를 정렬된 리스트로 관리하므로 O(n²) 복잡도를 가집니다.
    큰 맵에서는 efficient_a_star.py의 힙 기반 구현을 사용하세요.

    특징:
    - 4방향 이동 (상하좌우, 대각선 없음)
    - navigation_preference 배열로 벽 근처 경로에 패널티 부여
    - preference_weight=50으로 선호도 영향이 큼
    - 디버그용 OpenCV imshow가 포함되어 있어 실행 시 창이 열림 (비활성화 권장)
    """
    def __init__(self):
        # 4방향 인접 이동 벡터 (대각선은 주석 처리됨)
        self.adjacents = [[0, 1], [0, -1], [-1, 0], [1, 0], ]#[1, 1], [1, -1], [-1, -1], [-1, 1]]
        self.preference_weight = 50  # 선호도 패널티 가중치

    def get_preference(self, preference_grid, position):
        """주어진 위치의 선호도 값을 반환합니다. 범위 밖이면 0을 반환합니다."""
        if preference_grid is None:
            return 0
        elif not (position[0] >= preference_grid.shape[0] or position[1] >= preference_grid.shape[1] or position[0] < 0 or position[1] < 0):
            return preference_grid[position[0], position[1]]
        else:
            return 0

    # 주어진 미로(grid)에서 start에서 end까지의 경로를 리스트로 반환합니다
    def a_star(self, grid: np.ndarray, start, end, preference_grid=None):
        debug_grid = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)

        # 시작 노드와 목표 노드 생성
        startNode = aStarNode(None, list(start))
        startNode.g = startNode.h = startNode.f = 0

        if grid[start[0], start[1]]:
            print(f"[A*:a_star.a_star] 경고: 시작점 {list(start)}이 통과 불가(traversable) 영역입니다")

        endNode = aStarNode(None, list(end))

        # 목표가 장애물 위에 있으면 경로 없음
        if grid[end[0], end[1]]:
            print(f"[A*:a_star.a_star] 경고: 목표점 {list(end)}이 통과 불가 영역 → 빈 경로 반환")
            return []

        endNode.g = endNode.h = endNode.f = 0
        # open list(탐색 후보)와 closed list(탐색 완료) 초기화
        openList = []
        closedList = []

        # 시작 노드를 open list에 추가
        openList.append(startNode)

        # 목표에 도달하거나 open list가 빌 때까지 반복
        while len(openList) > 0:
            # open list에서 f 값이 가장 낮은 노드 선택 (O(n) 선형 탐색 - 비효율적)
            currentNode = openList[0]
            currentIndex = 0
            for index, item in enumerate(openList):
                if item.f < currentNode.f:
                    currentNode = item
                    currentIndex = index
            # 선택된 노드를 open list에서 제거하고 closed list에 추가
            openList.pop(currentIndex)

            closedList.append(currentNode)
            # 목표 노드에 도달 시 경로 역추적
            if currentNode == endNode:
                path = []
                current = currentNode
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]  # 역순으로 된 경로를 뒤집어 반환

            # 인접 노드(자식 노드) 생성
            children = []
            for adj in self.adjacents:
                # 인접 위치 계산
                nodePosition = [currentNode.position[0] + adj[0], currentNode.position[1] + adj[1]]
                # 배열 범위 내이고 통과 가능한 위치만 처리
                if not (nodePosition[0] >= grid.shape[0] or nodePosition[1] >= grid.shape[1] or nodePosition[0] < 0 or nodePosition[1] < 0):
                    if grid[nodePosition[0], nodePosition[1]]:
                        continue
                # 새 자식 노드 생성
                newNode = aStarNode(currentNode, nodePosition)
                children.append(newNode)

            # 각 자식 노드의 g, h, p, f 값을 계산하고 open list에 추가
            for child in children:
                continueLoop = False
                # 이미 closed list에 있으면 건너뜀
                for closedChild in closedList:
                    if child == closedChild:
                        continueLoop = True
                        break
                # g: 시작점에서 이 노드까지의 실제 비용 (1씩 증가)
                child.g = currentNode.g + 1
                # h: 맨해튼 거리 제곱 (유클리드보다 계산 빠름)
                child.h =  ((child.position[0] - endNode.position[0]) ** 2) + (
                           (child.position[1] - endNode.position[1]) ** 2)

                # p: 선호도 패널티 (벽 근처 높은 값 → 경로가 벽에서 멀어짐)
                child.p = self.get_preference(preference_grid, child.position) * self.preference_weight

                child.f = child.g + child.h + child.p
                # open list에 같은 위치가 있고 비용이 더 낮으면 건너뜀
                for index, openNode in enumerate(openList):
                    if child == openNode:
                        if child.p + child.g > openNode.p + openNode.g:
                            continueLoop = True
                            break

                if continueLoop:
                    continue
                # open list에 자식 노드 추가
                openList.append(child)

            # 디버그: open list 노드를 파란 점으로 표시 (성능에 영향을 줄 수 있음)
            for o in openList:
                debug_grid[o.position[0], o.position[1]] = [0, 0, 255]

            cv.imshow("debug", debug_grid)

            cv.waitKey(1)

        return []
