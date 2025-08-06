import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion


class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')
        self.get_logger().info("DWA Planner node started")

        # ROS interfaces
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # Timer to call main control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Goal location
        self.goal = Point(x=2.0, y=1.0, z=0.0)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0

        # Path history for visualization
        self.path_history = []

        # Laser scan data (optional use for collision checking)
        self.obstacle_points = []

        # Motion constraints
        self.max_linear_vel = 0.2
        self.max_angular_vel = 2.84
        self.max_linear_acc = 0.3
        self.max_angular_acc = 2.0
        self.dt = 0.1

    def odom_callback(self, msg):
        """ Update robot pose and velocity from odometry. """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def scan_callback(self, msg):
        """ Store obstacle points from laser scan. """
        self.obstacle_points = []
        for i, dist in enumerate(msg.ranges):
            if msg.range_min < dist < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                x = dist * math.cos(angle)
                y = dist * math.sin(angle)
                self.obstacle_points.append((x, y))  # relative to robot

    def control_loop(self):
        """ Main loop to drive robot toward the goal using DWA. """
        best_v, best_omega = self.dwa_planner()
        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_omega
        self.cmd_vel_pub.publish(cmd)
        self.publish_markers()

    def dwa_planner(self):
        """ Dynamic Window Approach: sample, simulate, score, and select best command. """
        best_score = -float('inf')
        best_v = 0.0
        best_omega = 0.0

        v_min = max(0.0, self.v - self.max_linear_acc * self.dt)
        v_max = min(self.max_linear_vel, self.v + self.max_linear_acc * self.dt)
        omega_min = self.omega - self.max_angular_acc * self.dt
        omega_max = self.omega + self.max_angular_acc * self.dt

        for v in self.frange(v_min, v_max, 0.02):
            for omega in self.frange(omega_min, omega_max, 0.1):
                traj, collision, min_dist = self.simulate_trajectory(v, omega)
                if collision:
                    continue

                final_x, final_y, final_theta = traj[-1]
                goal_dist = math.hypot(self.goal.x - final_x, self.goal.y - final_y)
                heading_score = -goal_dist
                clearance_score = min_dist
                velocity_score = v

                score = 1.0 * heading_score + 0.8 * clearance_score + 0.1 * velocity_score

                if score > best_score:
                    best_score = score
                    best_v = v
                    best_omega = omega

        return best_v, best_omega

    def simulate_trajectory(self, v, omega, sim_time=2.0):
        """ Simulate trajectory and check for collisions. """
        x, y, theta = 0.0, 0.0, 0.0
        traj = []
        min_dist = float('inf')

        for _ in range(int(sim_time / self.dt)):
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += omega * self.dt
            traj.append((x + self.x, y + self.y, theta + self.theta))

            for ox, oy in self.obstacle_points:
                dist = math.hypot(ox - x, oy - y)
                if dist < 0.2:
                    return traj, True, min_dist
                min_dist = min(min_dist, dist)

        return traj, False, min_dist

    def frange(self, start, stop, step):
        while start <= stop:
            yield round(start, 4)
            start += step

    def publish_markers(self):
        """ Publish red dot at goal and path line behind robot. """
        pt = Point(x=self.x, y=self.y, z=0.0)
        self.path_history.append(pt)
        if len(self.path_history) > 1000:
            self.path_history.pop(0)

        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "robot_path"
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

        goal_marker = Marker()
        goal_marker.header.frame_id = "map"
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = "goal_marker"
        goal_marker.id = 2
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position = self.goal
        goal_marker.scale.x = 0.1
        goal_marker.scale.y = 0.1
        goal_marker.scale.z = 0.1
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        self.marker_pub.publish(goal_marker)


def main():
    rclpy.init()
    node = DWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
