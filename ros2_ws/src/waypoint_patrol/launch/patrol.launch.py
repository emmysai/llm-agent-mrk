from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("waypoint_patrol")
    waypoints_yaml = os.path.join(pkg_share, "config", "waypoints.yaml")

    return LaunchDescription([
        Node(
            package="waypoint_patrol",
            executable="patrol_node",
            name="waypoint_patrol",
            output="screen",
            parameters=[{"waypoints_yaml": waypoints_yaml}],
        )
    ])
