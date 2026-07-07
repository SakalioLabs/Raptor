"""A* path planner for grid-based navigation."""
import heapq
from dataclasses import dataclass
import math


@dataclass(frozen=True, order=True)
class GridCell:
    row: int
    col: int


class GridMap:
    def __init__(self, rows: int, cols: int):
        self.rows = rows
        self.cols = cols
        self._obstacles: set[GridCell] = set()

    def set_obstacle(self, cell: GridCell):
        self._obstacles.add(cell)

    def clear_obstacle(self, cell: GridCell):
        self._obstacles.discard(cell)

    def is_blocked(self, cell: GridCell) -> bool:
        return cell in self._obstacles

    def in_bounds(self, cell: GridCell) -> bool:
        return 0 <= cell.row < self.rows and 0 <= cell.col < self.cols

    def neighbors(self, cell: GridCell) -> list[GridCell]:
        dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
        result = []
        for dr, dc in dirs:
            nb = GridCell(cell.row + dr, cell.col + dc)
            if self.in_bounds(nb) and not self.is_blocked(nb):
                if abs(dr) + abs(dc) == 2:
                    d1 = GridCell(cell.row + dr, cell.col)
                    d2 = GridCell(cell.row, cell.col + dc)
                    if self.is_blocked(d1) or self.is_blocked(d2):
                        continue
                result.append(nb)
        return result


class AStarPlanner:
    def plan(self, grid: GridMap, start: GridCell, goal: GridCell) -> list[GridCell] | None:
        open_set = [(0, start)]
        came_from: dict[GridCell, GridCell] = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        closed = set()
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self._reconstruct(came_from, current)
            if current in closed:
                continue
            closed.add(current)
            for nb in grid.neighbors(current):
                if nb in closed:
                    continue
                tent_g = g_score[current] + self._dist(current, nb)
                if tent_g < g_score.get(nb, float('inf')):
                    came_from[nb] = current
                    g_score[nb] = tent_g
                    f_score[nb] = tent_g + self._heuristic(nb, goal)
                    heapq.heappush(open_set, (f_score[nb], nb))
        return None

    def _heuristic(self, a: GridCell, b: GridCell) -> float:
        return math.sqrt((a.row-b.row)**2 + (a.col-b.col)**2)

    def _dist(self, a: GridCell, b: GridCell) -> float:
        return math.sqrt((a.row-b.row)**2 + (a.col-b.col)**2)

    def _reconstruct(self, came_from, current) -> list[GridCell]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
