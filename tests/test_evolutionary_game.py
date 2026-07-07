import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.swarm.evolutionary_game import ReplicatorDynamics, PayoffMatrix, TaskAllocationEGT

def test_replicator_converges():
    matrix = PayoffMatrix(2, np.array([[1, 0], [0, 1]]))
    dynamics = ReplicatorDynamics(matrix)
    pop = np.array([0.3, 0.7])
    ess = dynamics.find_ess(pop)
    assert abs(ess[1] - 1.0) < 0.1

def test_replicator_normalization():
    matrix = PayoffMatrix(3, np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
    dynamics = ReplicatorDynamics(matrix)
    pop = np.array([0.25, 0.25, 0.5])
    for _ in range(100):
        pop = dynamics.step(pop)
    assert abs(np.sum(pop) - 1.0) < 1e-6

def test_ess_detection():
    matrix = PayoffMatrix(2, np.array([[1, 2], [3, 1]]))
    dynamics = ReplicatorDynamics(matrix, mutation_rate=0)
    pop = np.array([0.0, 1.0])
    assert dynamics.is_ess(1, pop) == True

def test_task_allocation():
    egt = TaskAllocationEGT(n_tasks=3, n_agents=6)
    urgency = np.array([0.5, 0.3, 0.2])
    alloc = egt.allocate(urgency)
    assert np.sum(alloc) == 6
    assert all(a >= 0 for a in alloc)

def test_single_task():
    egt = TaskAllocationEGT(n_tasks=1, n_agents=4)
    alloc = egt.allocate(np.array([1.0]))
    assert alloc[0] == 4
