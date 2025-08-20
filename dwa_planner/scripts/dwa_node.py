#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point



class DWAPlannerNode(Node):
  
    def __init__(self):
        super().__init__("dwa_planner")

        # --- Robot Limits ---
        self.v_max         = 0.3    # m/s
        self.v_min         = 0.0
        self.omega_max     = 2.0    # rad/s
        self.omega_min     = -2.0
        self.a_max         = 0.5    # linear acceleration
        self.alpha_max     = 2.0    # angular acceleration
        self.robot_radius  = 0.20   # robot base radius
        self.safety_margin = 0.05    # extra buffer around robot
        self.effective_radius = self.robot_radius + self.safety_margin

        # --- DWA Parameters ---
        self.dt_control    = 0.2    # control update interval (s)
        self.dt_sim        = 0.1    # simulation step (s)
        self.predict_time  = 3.0    # prediction horizon (s)
        self.v_samples     = 20     # samples for linear velocity
        self.w_samples     = 10     # samples for angular velocity

        # --- Weights for scoring function ---
        self.w_heading     = 0.5
        self.w_clearance   = 0.3
        self.w_velocity    = 0.1
        self.w_smooth      = 0.1
        self.max_clearance = 0.3
        self.goal_tolerance = 0.1

        # --- State Variables ---
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.v, self.omega = 0.0, 0.0
        self.prev_v, self.prev_w = 0.0, 0.0
        self.goal = None
        self.obstacles = []
        self.path_points = []  


        # --- ROS 2 Interfaces ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer    = self.create_timer(self.dt_control, self.control_loop)
        self.goal_marker_pub = self.create_publisher(Marker, "goal_marker", 10)
        self.best_trej_pub = self.create_publisher(Marker, "best_trej", 10)
        self.path_pub = self.create_publisher(Marker, "robot_path", 10)




        self.get_logger().info("✅ DWA Planner Node started...")

    def odom_callback(self, msg: Odometry):
        """Update robot pose and velocity from odometry."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # self.get_logger().info(f"odom yaw : {self.yaw}")
        self.v     = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

        self.path_points.append((self.x, self.y))
        self.publish_path_marker()

    def goal_callback(self, msg: PoseStamped):
        """Update navigation goal (x, y, yaw)."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        q  = msg.pose.orientation

        _, _, gyaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.goal = (gx, gy, gyaw)
        self.gq = q
        self.path_points.clear()
        self.get_logger().info(f"🎯 Goal set to: x={gx:.3f}, y={gy:.3f}, yaw={gyaw:.3f} rad")

        self.publish_goal_marker()


    def scan_callback(self, msg: LaserScan):
        """Convert LaserScan into obstacle point cloud in world frame."""
        self.obstacles.clear()
        angle = msg.angle_min
        for r in msg.ranges:
            if 0.05 < r < msg.range_max and math.isfinite(r):
                ox = self.x + r * math.cos(self.yaw + angle)
                oy = self.y + r * math.sin(self.yaw + angle)
                self.obstacles.append((ox, oy))
            angle += msg.angle_increment

    def publish_goal_marker(self):
        if self.goal is None:
            return

        gx, gy, gyaw = self.goal

        marker = Marker()
        # Use the frame that makes sense for your system ("map" or "odom")
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dwa_goal"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = gx
        marker.pose.position.y = gy
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(gyaw * 0.5)
        marker.pose.orientation.w = math.cos(gyaw * 0.5)

        # Arrow sizing (small, nav-goal style)
        marker.scale.x = 1.5   # shaft length
        marker.scale.y = 0.2  # shaft diameter
        marker.scale.z = 0.2  # head diameter

        # Color (green)
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Optional: keep it visible until overwritten
        # marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        self.goal_marker_pub.publish(marker)

        self.get_logger().debug(f"Published goal arrow at ({gx:.2f}, {gy:.2f}) yaw={gyaw:.2f} rad")


    def publish_path_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dwa_path"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # style
        marker.scale.x = 0.05  # line thickness
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0  
        marker.color.b = 1.0

        # convert path points to geometry_msgs/Point
        for (x, y) in self.path_points:
            p = Point()
            p.x, p.y, p.z = x, y, 0.0
            marker.points.append(p)

        self.path_pub.publish(marker)


    def publish_traj_marker(self, trajectory):
        if not trajectory:
            return

        marker = Marker()
        marker.header.frame_id = "map"   # or "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dwa_path"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Add all trajectory points
        for (x, y, yaw) in trajectory:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        # Path line width
        marker.scale.x = 0.02  # line thickness

        # Path color (blue)
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.best_trej_pub.publish(marker)
   
    def motion(self, x, y, yaw, v, omega, dt):
        """Simple differential-drive motion model."""
        x_new   = x + v * math.cos(yaw) * dt
        y_new   = y + v * math.sin(yaw) * dt
        yaw_new = yaw + omega * dt
        # Normalize yaw to [-pi, pi]
        yaw_new = (yaw_new + math.pi) % (2.0 * math.pi) - math.pi
        return x_new, y_new, yaw_new

    def calc_dynamic_window(self):
        """Compute dynamic window: feasible (v, w) considering limits & acceleration."""
        # Global limits
        v_min, v_max = self.v_min, self.v_max
        w_min, w_max = self.omega_min, self.omega_max

        # Limits achievable in next step
        v_min_reach = self.v - self.a_max * self.dt_control
        v_max_reach = self.v + self.a_max * self.dt_control
        w_min_reach = self.omega - self.alpha_max * self.dt_control
        w_max_reach = self.omega + self.alpha_max * self.dt_control

        return (max(v_min, v_min_reach), min(v_max, v_max_reach),
                max(w_min, w_min_reach), min(w_max, w_max_reach))

    def simulate_trajectory(self, v, omega):
        """Roll out a trajectory (list of states) for predict_time."""
        x, y, yaw = self.x, self.y, self.yaw
        traj, t = [], 0.0
        while t < self.predict_time:
            x, y, yaw = self.motion(x, y, yaw, v, omega, self.dt_sim)
            traj.append((x, y, yaw))
            t += self.dt_sim
        return traj

   
    def trajectory_clearance(self, traj):
        """Compute min distance from trajectory to obstacles."""
        if not self.obstacles:
            return float('inf')
        rad = self.effective_radius
        r2  = rad * rad
        min_d2 = float('inf')

        for (x, y, _yaw) in traj:
            for (ox, oy) in self.obstacles:
                dx, dy = ox - x, oy - y
                d2 = dx*dx + dy*dy
                if d2 < r2:
                    return 0.0   # collision
                min_d2 = min(min_d2, d2)

        return math.sqrt(min_d2)

    def heading_score(self, traj):
        """Score based on how well trajectory final heading aligns with goal."""
        if self.goal is None or not traj:
            return 0.0
        gx, gy, gyaw = self.goal
        x, y, yaw = traj[-1]
        goal_dir = math.atan2(gy - y, gx - x)
        ang_err  = abs((goal_dir - yaw + math.pi) % (2*math.pi) - math.pi)
        return 1.0 - (ang_err / math.pi)

    def velocity_score(self, v):
        """Normalize forward velocity between [0,1]."""
        if self.v_max <= self.v_min:
            return 0.0
        return (v - self.v_min) / (self.v_max - self.v_min)

    def clearance_score(self, clearance):
        """Normalize clearance, capped at max_clearance."""
        return min(clearance, self.max_clearance) / self.max_clearance

    def path_smoothness_score(self, v, w):
        """Score based on smoothness (change from previous command)."""
        dv = abs(v - self.prev_v) / max(self.v_max, 1e-6)
        dw = abs(w - self.prev_w) / max(self.omega_max, 1e-6)
        return 1.0 - min(1.0, (dv + dw) / 2.0)

    def stopping_feasible(self, v, clearance):
        """Check if robot can stop safely within available clearance."""
        if self.a_max <= 1e-6:
            return True
        stop_dist = (v*v) / (2.0 * self.a_max)
        buffer    = v * self.dt_control
        return clearance > (stop_dist + buffer + self.effective_radius)


    def control_loop(self):
        if self.goal is None:
            return

        # Stop if goal is reached
        gx, gy, gyaw = self.goal
        if math.hypot(gx - self.x, gy - self.y) < self.goal_tolerance:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("🏁 Goal Reached.")
            return

        # 1) Compute dynamic window
        v_lower, v_upper, w_lower, w_upper = self.calc_dynamic_window()
        vs = np.linspace(v_lower, v_upper, max(1, self.v_samples))
        ws = np.linspace(w_lower, w_upper, max(1, self.w_samples))

        # 2) Evaluate candidates
        best_score, best_v, best_w = -1e9, 0.0, 0.0
        for v in vs:
            for w in ws:
                traj = self.simulate_trajectory(v, w)
                clearance = self.trajectory_clearance(traj)
                if clearance == 0.0 or not self.stopping_feasible(v, clearance):
                    continue

                heading_score = self.heading_score(traj)
                clear_score = self.clearance_score(clearance)
                vel_score = self.velocity_score(v)
                smooth_score    = self.path_smoothness_score(v, w)

                score = (self.w_heading*heading_score +
                         self.w_clearance*clear_score +
                         self.w_velocity*vel_score +
                         self.w_smooth*smooth_score)
                
                self.get_logger().debug(f"v={v:.2f}, w={w:.2f},goal heading={heading_score:.2f}, velocity score={vel_score:.2f}, clearence={clear_score:.2f}, path smooth={smooth_score:.2f} total={score:.2f}")

                if score > best_score:
                    best_score, best_v, best_w = score, float(v), float(w)

        # 3) Select command
        cmd = Twist()
        if best_score <= -1e8:
            self.get_logger().warn("⚠️ No valid trajectory found. Rotating in place.")
            cmd.angular.z = 0.3
        else:
            cmd.linear.x  = best_v
            cmd.angular.z = best_w

        # Update memory for smoothness check
        self.prev_v, self.prev_w = best_v, best_w
        best_traj = self.simulate_trajectory(best_v, best_w)

        self.cmd_pub.publish(cmd)
        self.publish_traj_marker(best_traj)




def main(args=None):
    rclpy.init(args=args)
    node = DWAPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
