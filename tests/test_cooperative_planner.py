import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.planning.cooperative_planner import CooperativePlanner, Agent, Point

def test_single_agent():
    cp = CooperativePlanner(grid_size=(10, 10))
    agents = [Agent("a1", Point(0, 0), Point(3, 0))]
    result = cp.plan(agents)
    assert "a1" in result
    assert result["a1"][-1] == Point(3, 0)

def test_two_agents_no_conflict():
    cp = CooperativePlanner(grid_size=(10, 10))
    agents = [
        Agent("a1", Point(0, 0), Point(0, 3)),
        Agent("a2", Point(5, 0), Point(5, 3)),
    ]
    result = cp.plan(agents)
    assert len(result) == 2

def test_two_agents_with_conflict():
    cp = CooperativePlanner(grid_size=(10, 10))
    agents = [
        Agent("a1", Point(0, 0), Point(3, 0)),
        Agent("a2", Point(5, 0), Point(0, 0)),
    ]
    result = cp.plan(agents)
    assert len(result) >= 1
