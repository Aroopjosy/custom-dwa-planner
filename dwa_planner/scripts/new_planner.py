#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from math import cos, sin, atan2, hypot, pi
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

class DWAPlannerNode(Node):
    def __init__(self):
        super().__init__('dwa_planner')
        # Configuration parameters (could be declared as ROS params)
        self.max_vel = 0.2       # m/s
        self.min_vel = 0.0      # m/s (reverse, if needed)
        self.max_yawrate = 1.0   # rad/s
        self.max_accel = 0.2     # m/s^2
        self.max_dyawrate = 3.2  # rad/s^2
        self.vel_res = 0.01      # sampling resolution (m/s)
        self.yawrate_res = 0.1   # sampling resolution (rad/s)
        self.predict_time = 1.0  # seconds to simulate forward
        self.dt = 0.1            # time step for simulation
        self.heading_weight = 0.8
        self.dist_weight = 1.2
        self.vel_weight = 0.1
        self.robot_radius = 0.2  # for collision checking
        
        # ROS subscribers and publishers
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.current_odom = None
        self.current_scan = None
        self.goal = None
        
        # Prompt user for goal once topics are ready
        self.get_logger().info('Waiting for odom and scan. Then enter goal...')
        self.create_timer(0.1, self.wait_for_data)

    def odom_callback(self, msg):
        self.current_odom = msg

    def scan_callback(self, msg):
        self.current_scan = msg

    def wait_for_data(self):
        if self.current_odom and self.current_scan and self.goal is None:
            try:
                # Get goal in *robot's base frame* or odom frame
                gx = float(input('Enter goal X (meters): '))
                gy = float(input('Enter goal Y (meters): '))
                self.goal = (gx, gy)
                self.get_logger().info(f'Goal set to ({gx:.2f}, {gy:.2f}). Starting planner.')
                # Now start the main control loop
                self.create_timer(0.1, self.control_loop)
            except:
                self.get_logger().error('Invalid input for goal.')

    def control_loop(self):
        # Ensure we have latest data
        if not self.current_odom or not self.current_scan:
            return

        # Extract robot pose and velocities from odom
        odom = self.current_odom
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        ori = odom.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y  # ideally ~0 for diff-drive
        v = hypot(vx, vy)               # current forward speed
        omega = odom.twist.twist.angular.z

        # Convert LaserScan to list of obstacle points in robot frame
        angles = np.linspace(self.current_scan.angle_min,
                             self.current_scan.angle_max,
                             len(self.current_scan.ranges))
        ranges = np.array(self.current_scan.ranges)
        mask = np.isfinite(ranges)
        pts_x = ranges[mask] * np.cos(angles[mask])
        pts_y = ranges[mask] * np.sin(angles[mask])
        obstacles = np.vstack((pts_x, pts_y)).T  # Nx2 array

        # Build dynamic window of sampled velocities
        v_samples = np.arange(self.min_vel, self.max_vel, self.vel_res)
        w_samples = np.arange(-self.max_yawrate, self.max_yawrate, self.yawrate_res)

        best_score = -1e9
        best_v = 0.0
        best_w = 0.0

        # Evaluate all (v, w) candidates
        for v_cmd in v_samples:
            for w_cmd in w_samples:
                # Skip if outside acceleration limits
                if abs(v_cmd - v) > self.max_accel * self.dt: 
                    continue
                if abs(w_cmd - omega) > self.max_dyawrate * self.dt: 
                    continue
                score = self.evaluate_trajectory(v_cmd, w_cmd, yaw, obstacles)
                if score > best_score:
                    best_score = score
                    best_v = v_cmd
                    best_w = w_cmd

        # Publish the best velocity command
        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        self.pub_vel.publish(cmd)

    def evaluate_trajectory(self, v, w, yaw, obstacles):
        """Simulate the trajectory for (v,w) and compute the DWA score."""
        x = 0.0
        y = 0.0
        yaw_t = yaw
        min_dist = float('inf')

        steps = int(self.predict_time / self.dt)
        for i in range(steps):
            # Simulate motion for dt
            x += v * cos(yaw_t) * self.dt
            y += v * sin(yaw_t) * self.dt
            yaw_t += w * self.dt
            # Collision check (robot radius)
            for (ox, oy) in obstacles:
                if hypot(ox - x, oy - y) < self.robot_radius:
                    return -1e6  # collision -> very bad score
                dist = hypot(ox - x, oy - y)
                if dist < min_dist:
                    min_dist = dist

        # Compute heading to goal from end-of-trajectory
        gx, gy = self.goal
        # End-pose in map frame = (px + x, py + y, yaw + (yaw_t-yaw))
        dx = gx - (x + self.current_odom.pose.pose.position.x)
        dy = gy - (y + self.current_odom.pose.pose.position.y)
        goal_dist = hypot(dx, dy)
        angle_to_goal = atan2(dy, dx)
        heading_diff = abs(angle_to_goal - yaw_t)

        # Score components
        heading_score = (pi - heading_diff)        # larger if aligned
        clearance_score = min_dist                # larger if further from obstacles
        velocity_score  = v                       # encourage faster

        # Weighted sum (tunable)
        score = (self.heading_weight * heading_score +
                 self.dist_weight    * clearance_score +
                 self.vel_weight    * velocity_score)
        return score

def main(args=None):
    rclpy.init(args=args)
    node = DWAPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()