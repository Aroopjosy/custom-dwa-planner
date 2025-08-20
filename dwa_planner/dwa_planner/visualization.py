import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def make_goal_marker(goal, stamp, frame_id="odom"):
    gx, gy, gyaw = goal
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "dwa_goal"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = gx
    marker.pose.position.y = gy
    marker.pose.orientation.z = math.sin(gyaw * 0.5)
    marker.pose.orientation.w = math.cos(gyaw * 0.5)
    marker.scale.x, marker.scale.y, marker.scale.z = 1.0, 0.15, 0.2
    marker.color.a, marker.color.g = 1.0, 1.0
    return marker

def make_path_marker(path_points, stamp, frame_id="odom"):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "dwa_path"
    marker.id = 1
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.05
    marker.color.a, marker.color.b = 1.0, 1.0
    for (x, y) in path_points:
        marker.points.append(Point(x=x, y=y, z=0.0))
    return marker

def make_traj_marker(traj, stamp, frame_id="odom"):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "dwa_best_traj"
    marker.id = 2
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.02
    marker.color.a, marker.color.r = 1.0, 1.0
    for (x, y, _) in traj:
        marker.points.append(Point(x=x, y=y, z=0.0))
    return marker


def make_candidate_markers(trajs, stamp, frame_id="map"):
    marker_array = MarkerArray()
    for i, traj in enumerate(trajs):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "dwa_candidates"
        marker.id = i
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01   # line thickness
        marker.color.a = 0.3    # transparent
        marker.color.g = 1.0    # green for all candidates

        for (x, y, _) in traj:
            marker.points.append(Point(x=x, y=y, z=0.0))

        marker_array.markers.append(marker)
    return marker_array