"""PID controller for flight attitude stabilization."""
from dataclasses import dataclass, field


@dataclass
class PIDGains:
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0


@dataclass
class PIDController:
    gains: PIDGains
    output_min: float = -1.0
    output_max: float = 1.0
    integral: float = 0.0
    prev_error: float = 0.0
    _first_update: bool = field(default=True, repr=False)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self._first_update = True

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        self.integral += error * dt
        if self._first_update:
            derivative = 0.0
            self._first_update = False
        else:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error
        output = (self.gains.kp * error
                  + self.gains.ki * self.integral
                  + self.gains.kd * derivative)
        return max(self.output_min, min(self.output_max, output))
