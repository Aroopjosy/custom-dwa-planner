#!/usr/bin/env python3
import rclpy
import math
import random
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        # Goal input
        self.goal_x = float(input("Enter goal X: "))
        self.goal_y = float(input("Enter goal Y: "))
        self.goal_reached = False

        # ROS data
        self.odom_data = None
        self.scan_data = None

        # Parameters
        self.max_speed = 0.15
        self.max_turn = 2.5
        self.step_time = 0.1
        self.num_paths = 30
        self.robot_radius = 0.105
        self.safety_margin = 0.05


        # Subscribers & Publishers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.path_history = []  # Store robot’s previous positions


        # Main timer loop
        self.create_timer(self.step_time, self.movement_loop)
        self.get_logger().info("DWA Planner node started.")

    def odom_callback(self, msg):
        self.odom_data = msg

    def scan_callback(self, msg):
        self.scan_data = msg

    def predict_path(self, speed, turn_rate):
        if self.odom_data is None:
            return []

        x = self.odom_data.pose.pose.position.x
        y = self.odom_data.pose.pose.position.y
        orientation = self.odom_data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        path = []
        for _ in range(100):
            yaw += turn_rate * self.step_time
            x += speed * math.cos(yaw) * self.step_time
            y += speed * math.sin(yaw) * self.step_time
            path.append((x, y))

        return path

    def check_for_collisions(self, path):
        if self.scan_data is None:
            return -float('inf')


        for x, y in path:
            distance = math.sqrt(x**2 + y**2)
            angle = math.atan2(y, x)
            index = int((angle + math.pi) / (2 * math.pi) * len(self.scan_data.ranges))
            index = max(0, min(len(self.scan_data.ranges) - 1, index))

            if distance < self.scan_data.ranges[index] - self.robot_radius + self.safety_margin:
                return -100000  # Collision penalty

        return 0  # No collision

    def choose_best_path(self, paths):
        if self.odom_data is None:
            return 0.0, 0.0

        cx = self.odom_data.pose.pose.position.x
        cy = self.odom_data.pose.pose.position.y
        orientation = self.odom_data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        dist_to_goal = math.hypot(self.goal_x - cx, self.goal_y - cy)
        if dist_to_goal < 0.05:
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info(f"Goal reached at ({self.goal_x:.2f}, {self.goal_y:.2f})!")
            return 0.0, 0.0

        best_score = float('-inf')
        best_cmd = (0.05, 0.0)

        for speed, turn, path in paths:
            if not path:
                continue

            gx, gy = self.goal_x, self.goal_y
            px, py = path[-1]

            dist_score = -math.hypot(px - gx, py - gy) * 5
            angle_diff = abs(math.atan2(gy - cy, gx - cx) - yaw)
            heading_score = -angle_diff * 2
            collision_score = self.check_for_collisions(path)
            smoothness_score = -0.1 * abs(turn)

            total = dist_score + heading_score + collision_score + smoothness_score
            if total > best_score:
                best_score = total
                best_cmd = (speed, turn)

        return best_cmd

    def generate_paths(self):
        for _ in range(self.num_paths):
            speed = random.uniform(0.0, self.max_speed)
            turn = random.uniform(-self.max_turn, self.max_turn)
            path = self.predict_path(speed, turn)
            yield (speed, turn, path)

    def movement_loop(self):
        if self.odom_data is None or self.scan_data is None or self.goal_reached:
            return

        paths = list(self.generate_paths())
        speed, turn = self.choose_best_path(paths)

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = turn
        self.cmd_pub.publish(twist)
        self.publish_markers()


    def publish_markers(self):
        # === Red Dot Marker (Goal Position) ===
        goal_marker = Marker()
        goal_marker.header.frame_id = "map"
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = "goal"
        goal_marker.id = 0
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = self.goal_x
        goal_marker.pose.position.y = self.goal_y
        goal_marker.pose.position.z = 0.0
        goal_marker.scale.x = 0.1
        goal_marker.scale.y = 0.1
        goal_marker.scale.z = 0.1
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0

        self.marker_pub.publish(goal_marker)

        # === Green Line Strip Marker (Path History) ===
        if self.odom_data:
            x = self.odom_data.pose.pose.position.x
            y = self.odom_data.pose.pose.position.y
            pt = Point(x=x, y=y, z=0.0)
            self.path_history.append(pt)
            if len(self.path_history) > 1000:
                self.path_history.pop(0)

            path_marker = Marker()
            path_marker.header.frame_id = "map"
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = "path"
            path_marker.id = 1
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.03
            path_marker.color.r = 0.0
            path_marker.color.g = 1.0
            path_marker.color.b = 0.0
            path_marker.color.a = 1.0
            path_marker.points = self.path_history
            self.marker_pub.publish(path_marker)



def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
