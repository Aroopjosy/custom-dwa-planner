from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Paths
    tb3_gazebo_pkg = get_package_share_directory("turtlebot3_gazebo")
    tb3_desc_pkg = get_package_share_directory("turtlebot3_description")
    dwa_pkg_dir = get_package_share_directory('dwa_planner')


    urdf_file = os.path.join(tb3_desc_pkg, "urdf", "turtlebot3_burger.urdf")
    rviz_config = os.path.join(dwa_pkg_dir, "rviz", "rviz_config.rviz")
    map_file = os.path.join(dwa_pkg_dir, "maps", "map.yaml")

    return LaunchDescription([
        # Argument: Map file
        DeclareLaunchArgument(
            "map",
            default_value=map_file,
            description="Full path to map yaml file"
        ),
        
         Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_broadcaster",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),

        # Lifecycle Manager (activates map_server)
        # Node(
        #     package="nav2_lifecycle_manager",
        #     executable="lifecycle_manager",
        #     name="lifecycle_manager_localization",
        #     output="screen",
        #     parameters=[{
        #         "use_sim_time": True,
        #         "autostart": True,
        #         "node_names": ["map_server"]
        #     }]
        # ),
        # === DWA Planner Node ===
        Node(
            package="dwa_planner",
            executable="dwa_planner_node.py",   # change if your executable name differs
            name="dwa_planner",
            output="screen"
        ),

        # === Map Server ===
        # Node(
        #     package="nav2_map_server",
        #     executable="map_server",
        #     name="map_server",
        #     output="screen",
        #     parameters=[{"yaml_filename": LaunchConfiguration("map")}]
        # ),

    
        # === RViz2 ===
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config]
        )
    ])
