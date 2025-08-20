#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray

# === Local imports ===

from dwa_planner.motion_model import calc_dynamic_window
from dwa_planner.visualization import (make_goal_marker, 
                                        make_path_marker, 
                                        make_traj_marker,
                                        make_candidate_markers 
                                        )
from dwa_planner.dwa_core import ( clearance_score, 
                                  heading_score, 
                                  path_smoothness_score, 
                                  stopping_feasible, 
                                  trajectory_clearance,
                                   velocity_score,
                                    simulate_trajectory,
                                    )

class DWAPlannerNode(Node):

    def __init__(self):
        super().__init__("dwa_planner")

            # --- Robot Limits ---
        self.v_max         = 0.3
        self.v_min         = 0.05
        self.omega_max     = 2.0
        self.omega_min     = -2.0
        self.a_max         = 1.0
        self.alpha_max     = 3.0
        self.robot_radius  = 0.30
        self.safety_margin = 0.10
        self.effective_radius = self.robot_radius + self.safety_margin

        # --- DWA Parameters ---
        self.dt_control   = 0.2
        self.dt_sim       = 0.1
        self.predict_time = 2.2
        self.v_samples    = 15
        self.w_samples    = 36

        # --- Weights (all positive, features normalized 0..1) ---
        self.w_heading   = 4.0
        self.w_clearance = -5.0
        self.w_velocity  = 2.0
        self.w_smooth    = 3.0

        self.max_clearance  = 0.4
        self.goal_tolerance = 0.1
        self.yaw_tolerance  = 0.1


        # --- State ---
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.v, self.omega = 0.0, 0.0
        self.prev_v, self.prev_w = 0.0, 0.0
        self.goal = None
        self.obstacles = []
        self.path_points = []

        # --- ROS Interfaces ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub  = self.create_publisher(Marker, "/goal_marker", 10)
        self.path_pub  = self.create_publisher(Marker, "/robot_path_marker", 10)
        self.traj_pub  = self.create_publisher(Marker, "/best_path_marker", 10)
        self.candidate_pub = self.create_publisher(MarkerArray, "/candidate_trajs", 10)


        self.timer = self.create_timer(self.dt_control, self.control_loop)

        self.get_logger().info("✅ DWA Planner Node started...")


    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.v     = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

        self.path_points.append((self.x, self.y))
        marker =  make_path_marker(self.path_points, self.get_clock().now().to_msg(), frame_id="map")
        self.path_pub.publish(marker)

    def goal_callback(self, msg: PoseStamped):
        gx, gy = msg.pose.position.x, msg.pose.position.y
        q = msg.pose.orientation
        _, _, gyaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.goal = (gx, gy, gyaw)

        self.path_points.clear()
        self.get_logger().info(f"🎯 Goal set: x={gx:.2f}, y={gy:.2f}, yaw={gyaw:.2f}")

        marker =  make_goal_marker(self.goal, self.get_clock().now().to_msg(), frame_id="map")
        self.goal_pub.publish(marker)

    def scan_callback(self, msg: LaserScan):
        self.obstacles.clear()
        angle = msg.angle_min
        for r in msg.ranges:
            if 0.05 < r < msg.range_max and math.isfinite(r):
                ox = self.x + r * math.cos(self.yaw + angle)
                oy = self.y + r * math.sin(self.yaw + angle)
                self.obstacles.append((ox, oy))
            angle += msg.angle_increment


    def control_loop(self):
        if self.goal is None:
            return

        gx, gy, gyaw = self.goal
        if math.hypot(gx - self.x, gy - self.y) < self.goal_tolerance:
            self.cmd_pub.publish(Twist())
            self.prev_v, self.prev_w = 0.0, 0.0
            self.get_logger().info("🏁 Goal Reached.")
            return

        # 1) Dynamic Window
        v_low, v_high, w_low, w_high = calc_dynamic_window(
            self.v, self.omega,
            self.v_min, self.v_max,
            self.omega_min, self.omega_max,
            self.a_max, self.alpha_max,
            self.dt_control
        )
        vs = np.linspace(v_low, v_high, max(1, self.v_samples))
        ws = np.linspace(w_low, w_high, max(1, self.w_samples))

        # 2) Evaluate trajectories
        best_score, best_v, best_w, best_traj, candidates = -1e9, 0.0, 0.0, [], []
        for v in vs:
            for w in ws:
                traj = simulate_trajectory(
                    self.x, self.y, self.yaw, v, w, self.dt_sim, self.predict_time
                )
                clearance = trajectory_clearance(traj, self.obstacles, self.effective_radius)
                if clearance < 0.02 or not stopping_feasible(v, self.a_max, self.dt_control, clearance, self.effective_radius):
                    continue
                
                candidates.append(traj)
                score_heading = self.w_heading * heading_score(traj, self.goal)
                score_clearance = self.w_clearance * clearance_score(clearance, self.max_clearance)
                score_velocity = self.w_velocity * velocity_score(v, self.v_min, self.v_max)
                score_smooth   = self.w_smooth * path_smoothness_score(v, w, self.prev_v, self.prev_w, self.v_max, self.omega_max)

                score = score_heading + score_clearance + score_velocity + score_smooth

                self.get_logger().info(
    f"v={self.v:.2f}, w={self.omega:.2f} | "
    f"H={score_heading:.2f}, C={score_clearance:.2f}, V={score_velocity:.2f}, S={score_smooth:.2f} "
    f"=> Total={score:.2f}"
)



                if score > best_score:
                    best_score, best_v, best_w, best_traj = score, float(v), float(w), traj

        cmd = Twist()
        if best_score <= -1e8:
            self.get_logger().warn("⚠️ No valid trajectory, rotating in place.")
            cmd.angular.z = 0.3
        else:
            cmd.linear.x = best_v
            cmd.angular.z = best_w

            self.prev_v, self.prev_w = best_v, best_w

        self.cmd_pub.publish(cmd)

        best_traj = simulate_trajectory(self.x, self.y, self.yaw, best_v, best_w, self.dt_sim, self.predict_time)
        
        if candidates:
            candidate_markers = make_candidate_markers(
                candidates, self.get_clock().now().to_msg(), frame_id="map"
            )
            self.candidate_pub.publish(candidate_markers)

        if best_traj:
            best_marker = make_traj_marker(
                best_traj, self.get_clock().now().to_msg(), frame_id="map"
            )
            self.traj_pub.publish(best_marker)


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
