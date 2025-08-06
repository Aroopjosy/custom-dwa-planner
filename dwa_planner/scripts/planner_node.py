#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

import math
import numpy as np


class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        # Robot parameters
        self.max_vel = 0.3
        self.min_vel = -0.1
        self.max_rot = 2.84
        self.max_acc = 0.4
        self.max_drot = 2.84
        self.sim_time = 1.0
        self.sim_dt = 0.1
        self.robot_radius = 0.2  # meters

        # Cost function weights
        self.w_heading = 0.6
        self.w_dist = 1.0
        self.w_vel = 0.1

        # State
        self.odom = None
        self.scan = None
        self.goal = None

        # ROS interfaces
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.odom = msg

    def scan_callback(self, msg):
        self.scan = msg

    def goal_callback(self, msg):
        self.goal = msg

    def control_loop(self):
        if not self.odom or not self.scan or not self.goal:
            return

        if min(self.scan.ranges) < 0.15:
            self.get_logger().warn("Emergency stop! Obstacle too close.")
            self.cmd_pub.publish(Twist())
            return

        # Extract state
        vx = self.odom.twist.twist.linear.x
        vtheta = self.odom.twist.twist.angular.z
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        q = self.odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Transform goal to robot frame
        gx = self.goal.pose.position.x - x
        gy = self.goal.pose.position.y - y
        x_r = math.cos(yaw) * gx + math.sin(yaw) * gy
        y_r = -math.sin(yaw) * gx + math.cos(yaw) * gy
        goal_dist = math.hypot(x_r, y_r)
        goal_angle = math.atan2(y_r, x_r)

        if goal_dist < 0.1:
            self.cmd_pub.publish(Twist())
            return

        # Dynamic window
        v_min_dyn = max(self.min_vel, vx - self.max_acc * self.sim_dt)
        v_max_dyn = min(self.max_vel, vx + self.max_acc * self.sim_dt)
        w_min_dyn = max(-self.max_rot, vtheta - self.max_drot * self.sim_dt)
        w_max_dyn = min(self.max_rot, vtheta + self.max_drot * self.sim_dt)

        v_samples = np.linspace(v_min_dyn, v_max_dyn, num=7)
        w_samples = np.linspace(w_min_dyn, w_max_dyn, num=15)

        best_score = -1e9
        best_v = 0.0
        best_w = 0.0

        for v in v_samples:
            for w in w_samples:
                traj_ok, score = self.evaluate_trajectory(v, w, goal_angle)
                if traj_ok and score > best_score:
                    best_score = score
                    best_v = v
                    best_w = w

        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        self.cmd_pub.publish(cmd)

    def evaluate_trajectory(self, v, w, goal_angle):
        x = 0.0
        y = 0.0
        yaw = 0.0
        min_dist = float('inf')
        t = 0.0

        while t < self.sim_time:
            x += v * math.cos(yaw) * self.sim_dt
            y += v * math.sin(yaw) * self.sim_dt
            yaw += w * self.sim_dt

            dist = math.hypot(x, y)
            angle = math.atan2(y, x)
            index = int((angle - self.scan.angle_min) / self.scan.angle_increment)

            if 0 <= index < len(self.scan.ranges):
                obstacle_dist = self.scan.ranges[index]
                if dist > obstacle_dist - self.robot_radius:
                    return (False, 0)
                min_dist = min(min_dist, obstacle_dist)
            else:
                return (False, 0)

            t += self.sim_dt

        heading_diff = abs(goal_angle - yaw)
        heading_score = math.pi - heading_diff
        dist_score = min_dist
        vel_score = v

        score = self.w_heading * heading_score + self.w_dist * dist_score + self.w_vel * vel_score
        return (True, score)


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
