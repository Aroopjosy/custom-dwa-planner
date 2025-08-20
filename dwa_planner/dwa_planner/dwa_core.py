import math
import numpy as np
from .motion_model import motion, calc_dynamic_window

def simulate_trajectory(x, y, yaw, v, omega, dt, predict_time):
    traj, t = [], 0.0
    while t < predict_time:
        x, y, yaw = motion(x, y, yaw, v, omega, dt)
        traj.append((x, y, yaw))
        t += dt
    return traj

def trajectory_clearance(traj, obstacles, effective_radius):
    if not obstacles:
        return float('inf')
    r2 = effective_radius**2
    min_d2 = float('inf')

    for (x, y, _) in traj:
        for (ox, oy) in obstacles:
            dx, dy = ox - x, oy - y
            d2 = dx*dx + dy*dy
            if d2 < r2:
                return 0.0
            min_d2 = min(min_d2, d2)
    return math.sqrt(min_d2)

def heading_score(traj, goal):
    if goal is None or not traj:
        return 0.0
    gx, gy, gyaw = goal
    x, y, yaw = traj[-1]
    goal_dir = math.atan2(gy - y, gx - x)
    ang_err  = abs((goal_dir - yaw + math.pi) % (2*math.pi) - math.pi)
    return 1.0 - (ang_err / math.pi)

def velocity_score(v, v_min, v_max):
    return (v - v_min) / (v_max - v_min) if v_max > v_min else 0.0

# def clearance_score(clearance, max_clearance):
#     return min(clearance, max_clearance) / max_clearance
def clearance_score(clearance, max_clearance):
    if clearance <= 0.0:
        return 0.0
    # Exponential shaping: very low score near obstacles, saturates at 1
    return 1.0 - math.exp(- (clearance / max_clearance) ** 2)


def path_smoothness_score(v, w, prev_v, prev_w, v_max, w_max):
    dv = abs(v - prev_v) / max(v_max, 1e-6)
    dw = abs(w - prev_w) / max(w_max, 1e-6)
    return 1.0 - min(1.0, (dv + dw) / 2.0)

def stopping_feasible(v, a_max, dt_control, clearance, effective_radius):
    if a_max <= 1e-6:
        return True
    stop_dist = (v*v) / (2.0 * a_max)
    buffer = v * dt_control
    return clearance > (stop_dist + buffer + effective_radius)
