import math

def motion(x, y, yaw, v, omega, dt):
    """Simple differential-drive motion model."""
    x_new   = x + v * math.cos(yaw) * dt
    y_new   = y + v * math.sin(yaw) * dt
    yaw_new = yaw + omega * dt
    return x_new, y_new, (yaw_new + math.pi) % (2*math.pi) - math.pi

def calc_dynamic_window(v, omega, v_min, v_max, w_min, w_max,
                        a_max, alpha_max, dt_control):
    """Compute dynamic window limits for (v, w)."""
    v_min_reach = v - a_max * dt_control
    v_max_reach = v + a_max * dt_control
    w_min_reach = omega - alpha_max * dt_control
    w_max_reach = omega + alpha_max * dt_control

    return (max(v_min, v_min_reach), min(v_max, v_max_reach),
            max(w_min, w_min_reach), min(w_max, w_max_reach))
