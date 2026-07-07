"""Flight state machine managing mode transitions."""
from enum import Enum, auto


class FlightMode(Enum):
    DISARMED = auto()
    STANDBY = auto()
    MANUAL = auto()
    STABILIZE = auto()
    ALTITUDE_HOLD = auto()
    POSITION_HOLD = auto()
    AUTO_MISSION = auto()
    RETURN_HOME = auto()
    EMERGENCY_LAND = auto()


_TRANSITIONS = {
    FlightMode.DISARMED: {FlightMode.STANDBY},
    FlightMode.STANDBY: {FlightMode.DISARMED, FlightMode.MANUAL, FlightMode.STABILIZE},
    FlightMode.MANUAL: {FlightMode.STABILIZE, FlightMode.ALTITUDE_HOLD, FlightMode.EMERGENCY_LAND, FlightMode.DISARMED},
    FlightMode.STABILIZE: {FlightMode.ALTITUDE_HOLD, FlightMode.POSITION_HOLD, FlightMode.MANUAL, FlightMode.EMERGENCY_LAND, FlightMode.DISARMED},
    FlightMode.ALTITUDE_HOLD: {FlightMode.POSITION_HOLD, FlightMode.AUTO_MISSION, FlightMode.STABILIZE, FlightMode.EMERGENCY_LAND, FlightMode.DISARMED},
    FlightMode.POSITION_HOLD: {FlightMode.AUTO_MISSION, FlightMode.RETURN_HOME, FlightMode.ALTITUDE_HOLD, FlightMode.EMERGENCY_LAND, FlightMode.DISARMED},
    FlightMode.AUTO_MISSION: {FlightMode.POSITION_HOLD, FlightMode.RETURN_HOME, FlightMode.EMERGENCY_LAND},
    FlightMode.RETURN_HOME: {FlightMode.POSITION_HOLD, FlightMode.EMERGENCY_LAND, FlightMode.DISARMED},
    FlightMode.EMERGENCY_LAND: {FlightMode.DISARMED},
}


class FlightStateMachine:
    def __init__(self):
        self._mode = FlightMode.DISARMED

    @property
    def mode(self):
        return self._mode

    def transition(self, target: FlightMode) -> bool:
        allowed = _TRANSITIONS.get(self._mode, set())
        if target in allowed or target == FlightMode.EMERGENCY_LAND:
            self._mode = target
            return True
        return False
