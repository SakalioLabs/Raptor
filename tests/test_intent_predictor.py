import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.decision.intent_predictor import IntentPredictor, BehaviorObservation

def test_initial_beliefs():
    pred = IntentPredictor(["approach", "retreat", "patrol"])
    intent, conf = pred.predict_intent()
    assert intent in ["approach", "retreat", "patrol"]
    assert abs(conf - 1.0/3) < 0.1

def test_update_beliefs_approach():
    goals = {"approach": (10, 0), "retreat": (-10, 0), "patrol": (0, 10)}
    pred = IntentPredictor(["approach", "retreat", "patrol"])
    obs = BehaviorObservation(position=(0, 0, 0), velocity=(5, 0, 0),
                              action_type="move", timestamp=0)
    pred.update_beliefs(obs, goals)
    intent, conf = pred.predict_intent()
    assert intent == "approach"
    assert conf > 0.4

def test_trajectory_prediction():
    goals = {"reach": (10, 0)}
    pred = IntentPredictor(["reach"])
    obs = BehaviorObservation(position=(0, 0, 0), velocity=(2, 0, 0),
                              action_type="move", timestamp=0)
    pred.observe(obs)
    pred.update_beliefs(obs, goals)
    traj = pred.predict_trajectory(n_steps=5, goal_positions=goals)
    assert len(traj) >= 2

def test_reset():
    pred = IntentPredictor(["a", "b"])
    obs = BehaviorObservation((0,0,0), (1,0,0), "m", 0)
    pred.update_beliefs(obs, {"a": (5,0), "b": (-5,0)})
    pred.reset()
    for intent in pred.intents:
        assert abs(intent.prior - 0.5) < 1e-6
    assert len(pred._history) == 0
