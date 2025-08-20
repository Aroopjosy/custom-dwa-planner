import math
from typing import Tuple


def quat_to_yaw(x, y, z, w) -> float:
    """Convert quaternion to yaw (ROS 2 uses xyzw). Works without tf libs."""
    # Yaw extraction per standard quaternion->Euler conversion
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return yaw


def wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))