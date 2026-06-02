#!/usr/bin/env python3

import json
import math
#import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry


import tf2_ros
from geometry_msgs.msg import TransformStamped
import yaml


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LLMAgentNode(Node):
    def __init__(self):
        super().__init__("llm_agent")

        self.declare_parameter("waypoints_yaml", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("odom_topic", "/odom")

        yaml_path = self.get_parameter("waypoints_yaml").get_parameter_value().string_value
        if not yaml_path:
            pkg_dir = Path(__file__).resolve().parents[1]
            yaml_path = str(pkg_dir / "config" / "waypoints.yaml")

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.frame_id, self.waypoints = self._load_waypoints(yaml_path)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {yaml_path}")

        self.last_odom = None

        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Tool 1: Robot pose only (no parameters)
        self.create_service(Trigger, "/llm_tools/get_robot_pose", self._srv_get_robot_pose)
        # Tool 2: All waypoints (no parameters)
        self.create_service(Trigger, "/llm_tools/get_waypoints", self._srv_get_waypoints)

        self.get_logger().info("LLM Agent Node ready 2 services registered.")

    def _load_waypoints(self, yaml_path: str):
        data = yaml.safe_load(Path(yaml_path).read_text())
        return data.get("frame_id", "map"), data.get("waypoints", [])

    def _on_odom(self, msg: Odometry):
        self.last_odom = msg

    def _get_pose_map(self):
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            return {"frame": self.map_frame, "x": x, "y": y, "yaw": yaw, "source": "tf"}
        except Exception:
            pass

        if self.last_odom is None:
            return None

        p = self.last_odom.pose.pose.position
        q = self.last_odom.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        return {"frame": self.last_odom.header.frame_id or "odom", "x": p.x, "y": p.y, "yaw": yaw, "source": "odom"}


    def _srv_get_robot_pose(self, request, response):
        """Tool 1: Returns current robot pose (x, y, yaw). No parameters."""
        pose = self._get_pose_map()
        response.success = True
        response.message = json.dumps(
            {"pose": pose} if pose else {"pose": None, "error": "Pose not yet available"},
            ensure_ascii=False,
        )
        return response

    def _srv_get_waypoints(self, request, response):
        """Tool 2: Returns all patrol waypoints. No parameters."""
        response.success = True
        response.message = json.dumps(
            {
                "frame_id": self.frame_id,
                "waypoints": self.waypoints,
                "count": len(self.waypoints),
            },
            ensure_ascii=False,
        )
        return response


def main():
    rclpy.init()
    node = LLMAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
