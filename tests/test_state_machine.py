import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.flight.state_machine import FlightStateMachine, FlightMode

def test_initial_mode():
    fsm = FlightStateMachine()
    assert fsm.mode == FlightMode.DISARMED

def test_valid_transition():
    fsm = FlightStateMachine()
    assert fsm.transition(FlightMode.STANDBY)
    assert fsm.mode == FlightMode.STANDBY

def test_invalid_transition():
    fsm = FlightStateMachine()
    assert not fsm.transition(FlightMode.AUTO_MISSION)
    assert fsm.mode == FlightMode.DISARMED

def test_emergency_from_any():
    fsm = FlightStateMachine()
    fsm.transition(FlightMode.STANDBY)
    fsm.transition(FlightMode.STABILIZE)
    assert fsm.transition(FlightMode.EMERGENCY_LAND)
    assert fsm.mode == FlightMode.EMERGENCY_LAND

def test_full_sequence():
    fsm = FlightStateMachine()
    fsm.transition(FlightMode.STANDBY)
    fsm.transition(FlightMode.STABILIZE)
    fsm.transition(FlightMode.ALTITUDE_HOLD)
    fsm.transition(FlightMode.POSITION_HOLD)
    assert fsm.mode == FlightMode.POSITION_HOLD
