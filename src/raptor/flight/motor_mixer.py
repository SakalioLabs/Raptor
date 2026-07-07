"""Motor mixer for hexacopter flight control."""
from dataclasses import dataclass


@dataclass
class MixerOutput:
    motor_1: float = 0.0
    motor_2: float = 0.0
    motor_3: float = 0.0
    motor_4: float = 0.0
    motor_5: float = 0.0
    motor_6: float = 0.0

    def as_list(self):
        return [self.motor_1, self.motor_2, self.motor_3,
                self.motor_4, self.motor_5, self.motor_6]


class HexacopterMixer:
    """Mixes throttle/roll/pitch/yaw commands to6 motors."""
    def __init__(self, thrust_factor: float = 1.0):
        self.thrust_factor = thrust_factor

    def mix(self, throttle: float, roll: float, pitch: float, yaw: float) -> MixerOutput:
        t = max(0.0, min(1.0, throttle)) * self.thrust_factor
        r = max(-1.0, min(1.0, roll))
        p = max(-1.0, min(1.0, pitch))
        y = max(-1.0, min(1.0, yaw))
        motors = [
            t + 0.5*r + 0.5*p - y,
            t + 0.5*r - 0.5*p + y,
            t - 0.5*p - y,
            t - 0.5*r - 0.5*p + y,
            t - 0.5*r + 0.5*p - y,
            t + 0.5*p + y,
        ]
        motors = [max(0.0, min(1.0, m)) for m in motors]
        return MixerOutput(*motors)
