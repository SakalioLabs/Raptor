"""Intent Recognition and Predictive Decision-Making for UAV Swarms.

Implements a Bayesian intent inference system that predicts other agents'
goals and plans based on observed behavior, enabling proactive cooperative
decision-making in multi-agent scenarios.

Based on: Baker et al., "Bayesian Theory of Mind: Modeling Joint
Belief-Desire Attribution" (2011) and Zhao et al., "Intention Recognition
in Multi-Agent Systems" from Zhejiang University (2023).
"""
from dataclasses import dataclass, field
import numpy as np


@dataclass
class Intent:
    name: str
    prior: float = 0.0
    likelihood: float = 0.0
    posterior: float = 0.0


@dataclass
class BehaviorObservation:
    position: tuple
    velocity: tuple
    action_type: str
    timestamp: float


class IntentPredictor:
    def __init__(self, possible_intents: list, observation_noise: float = 0.1):
        self.intents = [Intent(name=name, prior=1.0/len(possible_intents), posterior=1.0/len(possible_intents))
                       for name in possible_intents]
        self.noise = observation_noise
        self._history: list[BehaviorObservation] = []

    def observe(self, obs: BehaviorObservation):
        self._history.append(obs)

    def compute_likelihood(self, intent_name: str, obs: BehaviorObservation,
                           goal_positions: dict) -> float:
        if intent_name not in goal_positions:
            return 0.5
        goal = np.array(goal_positions[intent_name])
        pos = np.array(obs.position[:2])
        vel = np.array(obs.velocity[:2])
        to_goal = goal - pos
        dist = np.linalg.norm(to_goal)
        if dist < 1e-6:
            return 0.9
        to_goal_norm = to_goal / dist
        vel_norm = vel / max(np.linalg.norm(vel), 1e-6)
        alignment = float(np.dot(to_goal_norm, vel_norm))
        likelihood = 0.5 + 0.4 * alignment
        return np.clip(likelihood, 0.01, 0.99)

    def update_beliefs(self, obs: BehaviorObservation,
                       goal_positions: dict):
        total = 0.0
        for intent in self.intents:
            intent.likelihood = self.compute_likelihood(
                intent.name, obs, goal_positions
            )
            intent.posterior = intent.prior * intent.likelihood
            total += intent.posterior
        if total > 0:
            for intent in self.intents:
                intent.posterior /= total
                intent.prior = intent.posterior

    def predict_intent(self) -> tuple:
        best = max(self.intents, key=lambda i: i.posterior)
        confidence = best.posterior
        return (best.name, confidence)

    def predict_trajectory(self, n_steps: int = 10,
                           goal_positions: dict = None) -> list:
        if not self._history or not goal_positions:
            return []
        intent_name, conf = self.predict_intent()
        if intent_name not in goal_positions:
            return []
        goal = np.array(goal_positions[intent_name][:2])
        last = self._history[-1]
        pos = np.array(last.position[:2])
        trajectory = [pos.copy()]
        speed = np.linalg.norm(last.velocity[:2]) if any(last.velocity) else 1.0
        for _ in range(n_steps):
            direction = goal - pos
            dist = np.linalg.norm(direction)
            if dist < 0.5:
                break
            step = direction / dist * min(speed, dist)
            pos = pos + step
            trajectory.append(pos.copy())
        return trajectory

    def reset(self):
        n = len(self.intents)
        for intent in self.intents:
            intent.prior = 1.0 / n
            intent.likelihood = 0.0
            intent.posterior = 0.0
        self._history.clear()
