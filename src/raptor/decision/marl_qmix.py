"""Multi-Agent Q-Learning with QMIX-style Value Decomposition.

Implements a simplified QMIX architecture for cooperative multi-agent
decision making. Each agent maintains a local Q-network, and a mixing
network combines them for team-level credit assignment.

Based on: Rashid et al., "QMIX: Monotonic Value Function Factorisation
for Deep Multi-Agent Reinforcement Learning" (2018).
"""
from dataclasses import dataclass, field
import numpy as np


@dataclass
class AgentState:
    agent_id: str
    observation: np.ndarray
    last_action: int = 0
    q_table: dict = field(default_factory=dict)


class QMixAgent:
    def __init__(self, agent_id: str, n_actions: int, n_features: int,
                 lr: float = 0.1, gamma: float = 0.95, epsilon: float = 0.1):
        self.state = AgentState(agent_id=agent_id, observation=np.zeros(n_features))
        self.n_actions = n_actions
        self.n_features = n_features
        self.lr = lr
        self.gamma = gamma
        self.epsilon = epsilon

    def get_q_values(self, obs_key: str) -> np.ndarray:
        if obs_key not in self.state.q_table:
            self.state.q_table[obs_key] = np.zeros(self.n_actions)
        return self.state.q_table[obs_key]

    def select_action(self, obs_key: str) -> int:
        if np.random.random() < self.epsilon:
            return np.random.randint(self.n_actions)
        q = self.get_q_values(obs_key)
        return int(np.argmax(q))

    def update(self, obs_key: str, action: int, reward: float,
               next_obs_key: str, done: bool):
        q = self.get_q_values(obs_key)
        next_q = self.get_q_values(next_obs_key)
        target = reward + (0 if done else self.gamma * np.max(next_q))
        q[action] += self.lr * (target - q[action])


class QMixNetwork:
    def __init__(self, n_agents: int, n_actions: int, state_dim: int):
        self.n_agents = n_agents
        self.n_actions = n_actions
        self.state_dim = state_dim
        self.mix_weights = np.random.randn(state_dim, n_agents) * 0.1
        self.mix_bias = 0.0

    def mix(self, agent_q_values: list, global_state: np.ndarray) -> float:
        q_selected = np.array(agent_q_values)
        w = np.abs(global_state @ self.mix_weights) + 0.01
        return float(w @ q_selected + self.mix_bias)

    def update_weights(self, agent_q_values: list, global_state: np.ndarray,
                       td_error: float, lr: float = 0.01):
        w = np.abs(global_state @ self.mix_weights) + 0.01
        q_selected = np.array(agent_q_values)
        grad = np.outer(global_state, q_selected) * np.sign(td_error)
        self.mix_weights += lr * grad * td_error
        self.mix_bias += lr * td_error


class QMixTrainer:
    def __init__(self, n_agents: int, n_actions: int, state_dim: int):
        self.agents = [
            QMixAgent(f"agent_{i}", n_actions, state_dim)
            for i in range(n_agents)
        ]
        self.mixer = QMixNetwork(n_agents, n_actions, state_dim)
        self.n_agents = n_agents

    def select_actions(self, observations: dict) -> dict:
        actions = {}
        for agent in self.agents:
            obs_key = observations.get(agent.state.agent_id, "default")
            actions[agent.state.agent_id] = agent.select_action(obs_key)
        return actions

    def train_step(self, observations: dict, actions: dict,
                   rewards: dict, next_observations: dict,
                   global_state: np.ndarray, done: bool):
        q_values = []
        next_q_values = []
        for agent in self.agents:
            aid = agent.state.agent_id
            obs_key = observations.get(aid, "default")
            next_obs_key = next_observations.get(aid, "default")
            q = agent.get_q_values(obs_key)[actions[aid]]
            nq = np.max(agent.get_q_values(next_obs_key))
            q_values.append(q)
            next_q_values.append(nq)
            agent.update(obs_key, actions[aid], rewards.get(aid, 0),
                        next_obs_key, done)
        q_total = self.mixer.mix(q_values, global_state)
        next_q_total = self.mixer.mix(next_q_values, global_state)
        team_reward = np.mean(list(rewards.values()))
        target = team_reward + (0 if done else 0.95 * next_q_total)
        td_error = target - q_total
        self.mixer.update_weights(q_values, global_state, td_error)
        return float(td_error)
