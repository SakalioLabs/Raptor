"""Attitude estimation using quaternion math."""
import math
from dataclasses import dataclass


@dataclass
class Quaternion:
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def normalize(self):
        n = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if n < 1e-10:
            return Quaternion()
        return Quaternion(self.w/n, self.x/n, self.y/n, self.z/n)

    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def multiply(self, other: "Quaternion") -> "Quaternion":
        w = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
        x = self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y
        y = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
        z = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
        return Quaternion(w, x, y, z)

    def to_euler(self):
        sinr = 2.0 * (self.w * self.x + self.y * self.z)
        cosr = 1.0 - 2.0 * (self.x**2 + self.y**2)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (self.w * self.y - self.z * self.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        siny = 2.0 * (self.w * self.z + self.x * self.y)
        cosy = 1.0 - 2.0 * (self.y**2 + self.z**2)
        yaw = math.atan2(siny, cosy)
        return (roll, pitch, yaw)

    @staticmethod
    def from_euler(roll: float, pitch: float, yaw: float) -> "Quaternion":
        cr, sr = math.cos(roll/2), math.sin(roll/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        return Quaternion(
            cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
        )

    @staticmethod
    def from_axis_angle(axis: tuple, angle: float) -> "Quaternion":
        half = angle / 2.0
        s = math.sin(half)
        n = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
        if n < 1e-10:
            return Quaternion()
        return Quaternion(math.cos(half), axis[0]*s/n, axis[1]*s/n, axis[2]*s/n)
