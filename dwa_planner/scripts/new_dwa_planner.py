#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from tf_transformations import euler_from_quaternion

# ==============================
# Dynamic Window Approach (DWA) Helper Functions
# ==============================

def motion(x, y, yaw, v, omega, dt):
    """
    Robot motion model (differential drive).
    Update x, y, yaw based on v (linear vel) and omega (angular vel).
    """
    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    yaw += omega * dt
    return x, y, yaw


def simulate_trajectory(x, y, yaw, v, omega, dt, predict_time):
    """
    Simulate forward in time to generate a trajectory for given (v, omega).
    """
    traj = []
    t = 0.0
    while t < predict_time:
        x, y, yaw = motion(x, y, yaw, v, omega, dt)
        traj.append((x, y, yaw))
        t += dt
    return traj


def calc_dynamic_window(v, omega, config):
    """
    Calculate dynamic window [v_min, v_max, omega_min, omega_max]
    considering velocity limits and acceleration limits.
    """
    # From robot specs
    Vs = [config['min_speed'], config['max_speed'],
          -config['max_yawrate'], config['max_yawrate']]

    # From motion model
    Vd = [v - config['max_accel'] * config['dt'],
          v + config['max_accel'] * config['dt'],
          omega - config['max_dyawrate'] * config['dt'],
          omega + config['max_dyawrate'] * config['dt']]

    # [v_min, v_max, w_min, w_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    return dw


# ==============================
# DWA Planner Node (ROS2 Python)
# ==============================
class DWAPlannerNode(Node):
    def __init__(self):
        super().__init__("dwa_planner")

        # === Parameters ===
        self.config = {
            'max_speed': 0.5,
            'min_speed': -0.1,
            'max_yawrate': 1.0,
            'max_accel': 0.2,
            'max_dyawrate': 2.0,
            'v_reso': 0.01,
            'yawrate_reso': 0.1,
            'dt': 0.1,
            'predict_time': 2.0
        }

        # === State Variables ===
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.v, self.omega = 0.0, 0.0
        self.goal = None

        # === ROS2 Publishers & Subscribers ===
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.traj_pub = self.create_publisher(MarkerArray, "/trajectories", 10)
        self.best_pub = self.create_publisher(Marker, "/best_trajectory", 10)

        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(PoseStamped, "/goal", self.goal_callback, 10)
        self.create_subscription(Path, "/path", self.path_callback, 10)

        # Timer for control loop
        self.create_timer(self.config['dt'], self.control_loop)

        self.obstacles = []

    # ==============================
    # Callbacks
    # ==============================
    def odom_callback(self, msg: Odometry):
        """Update robot state (pose + velocity)."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def scan_callback(self, msg: LaserScan):
        """Convert laser scan into obstacle points."""
        self.obstacles = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                ox = self.x + r * math.cos(self.yaw + angle)
                oy = self.y + r * math.sin(self.yaw + angle)
                self.obstacles.append((ox, oy))
            angle += msg.angle_increment

    def goal_callback(self, msg: PoseStamped):
        """Update goal position."""
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def path_callback(self, msg: Path):
        """Handle global path (not used fully in this demo)."""
        pass

    # ==============================
    # DWA Control Loop
    # ==============================
    def control_loop(self):
        if self.goal is None:
            return

        dw = calc_dynamic_window(self.v, self.omega, self.config)

        best_u = [0.0, 0.0]   # (v, omega)
        best_traj = []
        min_cost = float("inf")

        marker_array = MarkerArray()

        # Evaluate all candidate trajectories
        v = dw[0]
        while v <= dw[1]:
            omega = dw[2]
            while omega <= dw[3]:
                traj = simulate_trajectory(self.x, self.y, self.yaw, v, omega,
                                           self.config['dt'], self.config['predict_time'])

                # === Cost functions ===
                # Goal cost (distance to goal)
                gx, gy = self.goal
                last_x, last_y, _ = traj[-1]
                goal_cost = math.hypot(gx - last_x, gy - last_y)

                # Velocity cost (prefer forward motion)
                speed_cost = (self.config['max_speed'] - abs(v))

                # Obstacle cost
                ob_cost = 0.0
                for (ox, oy) in self.obstacles:
                    dists = [math.hypot(x - ox, y - oy) for (x, y, _) in traj]
                    ob_cost += 1.0 / (min(dists) + 1e-6)

                # Total cost
                cost = goal_cost + ob_cost + speed_cost

                # Keep best
                if cost < min_cost:
                    min_cost = cost
                    best_u = [v, omega]
                    best_traj = traj

                # Visualization of candidate trajectory
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "dwa_traj"
                marker.id = len(marker_array.markers)
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.02
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 1.0, 0.0, 0.3)

                marker.points = []
                from geometry_msgs.msg import Point
                for (px, py, _) in traj:
                    p = Point()
                    p.x, p.y, p.z = px, py, 0.0
                    marker.points.append(p)
                marker_array.markers.append(marker)

                omega += self.config['yawrate_reso']
            v += self.config['v_reso']

        # Publish all candidate trajectories
        self.traj_pub.publish(marker_array)

        # Publish best trajectory marker
        if best_traj:
            best_marker = Marker()
            best_marker.header.frame_id = "odom"
            best_marker.header.stamp = self.get_clock().now().to_msg()
            best_marker.ns = "dwa_best"
            best_marker.id = 999
            best_marker.type = Marker.LINE_STRIP
            best_marker.action = Marker.ADD
            best_marker.scale.x = 0.05
            best_marker.color.r, best_marker.color.g, best_marker.color.b, best_marker.color.a = (1.0, 0.0, 0.0, 1.0)

            best_marker.points = []
            from geometry_msgs.msg import Point
            for (px, py, _) in best_traj:
                p = Point()
                p.x, p.y, p.z = px, py, 0.0
                best_marker.points.append(p)

            self.best_pub.publish(best_marker)

        # Publish best command
        twist = Twist()
        twist.linear.x = best_u[0]
        twist.angular.z = best_u[1]
        self.cmd_pub.publish(twist)


# ==============================
# Main
# ==============================
def main(args=None):
    rclpy.init(args=args)
    node = DWAPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
