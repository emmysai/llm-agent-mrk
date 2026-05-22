from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("llm_agent")
    waypoints_yaml = os.path.join(pkg_share, "config", "waypoints.yaml")

    return LaunchDescription([
        Node(
            package="llm_agent",
            executable="llm_agent_node",
            name="llm_agent",
            output="screen",
            parameters=[{
                "waypoints_yaml": waypoints_yaml,
                "map_frame": "map",
                "base_frame": "base_footprint",
                "odom_topic": "/odom",
                "scan_topic": "/scan",
            }],
        )
    ])
