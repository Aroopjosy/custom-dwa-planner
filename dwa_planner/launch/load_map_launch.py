# load_map_and_rviz.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        # ),
        Node(
            package="dwa_planner",
            executable='dwa_planner_node.py',
            output="screen",
        )
    ])
