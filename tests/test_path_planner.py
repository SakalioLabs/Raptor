import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.mission.path_planner import AStarPlanner, GridMap, GridCell

def test_straight_path():
    planner = AStarPlanner()
    grid = GridMap(10, 10)
    path = planner.plan(grid, GridCell(0, 0), GridCell(0, 4))
    assert path is not None
    assert path[0] == GridCell(0, 0)
    assert path[-1] == GridCell(0, 4)

def test_path_around_obstacle():
    planner = AStarPlanner()
    grid = GridMap(5, 5)
    for r in range(5):
        grid.set_obstacle(GridCell(r, 2))
    grid.clear_obstacle(GridCell(4, 2))
    path = planner.plan(grid, GridCell(2, 0), GridCell(2, 4))
    assert path is not None
    assert path[-1] == GridCell(2, 4)

def test_no_path():
    planner = AStarPlanner()
    grid = GridMap(3, 3)
    for r in range(3):
        grid.set_obstacle(GridCell(r, 1))
    path = planner.plan(grid, GridCell(1, 0), GridCell(1, 2))
    assert path is None
