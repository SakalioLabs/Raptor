import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.decision.marl_qmix import QMixAgent, QMixNetwork, QMixTrainer

def test_agent_select_action():
    agent = QMixAgent("a1", n_actions=4, n_features=2, epsilon=0)
    obs_key = str(np.array([0.5, 0.5]))
    q = agent.get_q_values(obs_key)
    q[2] = 10.0
    agent.state.q_table[obs_key] = q
    action = agent.select_action(obs_key)
    assert action == 2

def test_agent_update():
    agent = QMixAgent("a1", n_actions=4, n_features=2)
    agent.update("s0", 0, 1.0, "s1", False)
    q = agent.get_q_values("s0")
    assert q[0] != 0

def test_mix_network():
    mixer = QMixNetwork(n_agents=3, n_actions=4, state_dim=5)
    q_vals = [0.5, 0.8, 0.3]
    state = np.ones(5)
    result = mixer.mix(q_vals, state)
    assert isinstance(result, float)

def test_trainer_step():
    trainer = QMixTrainer(n_agents=2, n_actions=3, state_dim=4)
    obs = {"agent_0": "s0", "agent_1": "s1"}
    actions = trainer.select_actions(obs)
    assert len(actions) == 2
    rewards = {"agent_0": 1.0, "agent_1": 0.5}
    state = np.ones(4)
    td = trainer.train_step(obs, actions, rewards, obs, state, False)
    assert isinstance(td, float)
