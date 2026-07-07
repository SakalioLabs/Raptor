"""Evolutionary Game Theory for multi-UAV task allocation.

Implements replicator dynamics and evolutionary stable strategies (ESS)
for emergent role allocation in UAV swarms. Inspired by biological
swarm intelligence and evolutionary computation approaches.

Based on: Smith & Price, "The logic of animal conflict" (1973),
Cao et al., "An Overview of Recent Progress in the Study of
Distributed Multi-Agent Coordination" (2013),
and Zhejiang University research on network-centric UAV coordination.
"""
from dataclasses import dataclass, field
import numpy as np
from typing import Callable


@dataclass
class Strategy:
    name: str
    fitness: float = 0.0
    population_share: float = 0.0


@dataclass
class PayoffMatrix:
    n_strategies: int
    matrix: np.ndarray  # (n, n) payoff matrix

    def payoff(self, i: int, j: int) -> float:
        return float(self.matrix[i, j])


class ReplicatorDynamics:
    def __init__(self, payoff_matrix: PayoffMatrix, mutation_rate: float = 0.01):
        self.payoff = payoff_matrix
        self.mutation_rate = mutation_rate

    def step(self, population: np.ndarray, dt: float = 0.01) -> np.ndarray:
        n = len(population)
        fitness = self.payoff.matrix @ population
        avg_fitness = population @ fitness
        if avg_fitness < 1e-10:
            return population
        growth = population * (fitness - avg_fitness) / max(avg_fitness, 1e-6)
        new_pop = population + growth * dt
        if self.mutation_rate > 0:
            mutation = np.full(n, self.mutation_rate / n)
            new_pop = new_pop * (1 - self.mutation_rate) + mutation
        new_pop = np.maximum(new_pop, 0)
        total = np.sum(new_pop)
        if total > 0:
            new_pop /= total
        return new_pop

    def find_ess(self, initial_pop: np.ndarray, max_iter: int = 10000,
                 tol: float = 1e-8) -> np.ndarray:
        pop = initial_pop.copy()
        for _ in range(max_iter):
            new_pop = self.step(pop, dt=0.05)
            if np.max(np.abs(new_pop - pop)) < tol:
                return new_pop
            pop = new_pop
        return pop

    def is_ess(self, strategy_idx: int, population: np.ndarray) -> bool:
        n = len(population)
        test_pop = np.zeros(n)
        test_pop[strategy_idx] = 1.0
        invader_fitness = self.payoff.matrix[strategy_idx] @ population
        resident_fitness = population @ (self.payoff.matrix @ population)
        if invader_fitness > resident_fitness + 1e-8:
            return False
        if abs(invader_fitness - resident_fitness) < 1e-8:
            mutant_payoff = self.payoff.matrix[strategy_idx, strategy_idx]
            resident_payoff = population @ (self.payoff.matrix @ population)
            return mutant_payoff < resident_payoff or abs(mutant_payoff - resident_payoff) < 1e-8
        return True


class TaskAllocationEGT:
    def __init__(self, n_tasks: int, n_agents: int):
        self.n_tasks = n_tasks
        self.n_agents = n_agents
        self._build_payoff()

    def _build_payoff(self):
        n = self.n_tasks
        matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i == j:
                    matrix[i, j] = 1.0
                else:
                    matrix[i, j] = -0.5
        self.payoff = PayoffMatrix(n, matrix)
        self.dynamics = ReplicatorDynamics(self.payoff)

    def allocate(self, task_urgency: np.ndarray) -> np.ndarray:
        pop = task_urgency / max(np.sum(task_urgency), 1e-6)
        ess = self.dynamics.find_ess(pop)
        allocation = np.round(ess * self.n_agents).astype(int)
        diff = self.n_agents - np.sum(allocation)
        if diff > 0:
            allocation[np.argmax(ess)] += int(diff)
        elif diff < 0:
            allocation[np.argmax(allocation)] += int(diff)
        return allocation
