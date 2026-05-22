#!/usr/bin/env python3
"""
LLM Agent Node – ROS2 service provider for LLM tools.

Exposed services:
  /llm_tools/get_robot_pose      – current pose only (no params)  [Tool 1]
  /llm_tools/get_waypoints       – all 4 patrol waypoints (no params)  [Tool 2]
  /llm_tools/get_robot_state     – full sensor state (pose + odom + laser + imu)
  /llm_tools/get_nearest_waypoint – nearest waypoint from current pose
"""
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu

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
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/imu")

        yaml_path = self.get_parameter("waypoints_yaml").get_parameter_value().string_value
        if not yaml_path:
            pkg_dir = Path(__file__).resolve().parents[1]
            yaml_path = str(pkg_dir / "config" / "waypoints.yaml")

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.frame_id, self.waypoints = self._load_waypoints(yaml_path)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {yaml_path}")

        self.last_odom = None
        self.last_scan = None
        self.last_imu = None

        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10)
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self._on_scan, 10)
        self.create_subscription(Imu, self.get_parameter("imu_topic").value, self._on_imu, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Tool 1: Robot pose only (no parameters)
        self.create_service(Trigger, "/llm_tools/get_robot_pose", self._srv_get_robot_pose)
        # Tool 2: All waypoints (no parameters)
        self.create_service(Trigger, "/llm_tools/get_waypoints", self._srv_get_waypoints)
        # Full sensor state
        self.create_service(Trigger, "/llm_tools/get_robot_state", self._srv_get_robot_state)
        # Pre-computed nearest waypoint
        self.create_service(Trigger, "/llm_tools/get_nearest_waypoint", self._srv_get_nearest_waypoint)

        self.get_logger().info("LLM Agent Node ready – 4 services registered.")

    def _load_waypoints(self, yaml_path: str):
        data = yaml.safe_load(Path(yaml_path).read_text())
        return data.get("frame_id", "map"), data.get("waypoints", [])

    def _on_odom(self, msg: Odometry):
        self.last_odom = msg

    def _on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def _on_imu(self, msg: Imu):
        self.last_imu = msg

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

    def _nearest_waypoint(self, pose):
        if pose is None or not self.waypoints:
            return None
        rx, ry = pose["x"], pose["y"]
        best = None
        for i, wp in enumerate(self.waypoints):
            dx = rx - float(wp["x"])
            dy = ry - float(wp["y"])
            d = math.sqrt(dx * dx + dy * dy)
            if best is None or d < best["distance_m"]:
                best = {"index": i, "name": wp.get("name", f"wp{i}"), "distance_m": d, "waypoint": wp}
        return best

    def _interpret(self, state):
        notes = []
        lm = None
        if state.get("laser") is not None:
            lm = state["laser"].get("min_m")
        if lm is not None:
            if lm < 0.35:
                notes.append("Obstacle very close (<0.35 m).")
            elif lm < 0.70:
                notes.append("Obstacle at medium distance (<0.70 m).")
            else:
                notes.append("No close obstacles detected.")

        sp = state.get("speed_mps")
        if sp is not None:
            notes.append("Robot is stationary." if sp < 0.02 else "Robot is moving.")

        return " ".join(notes) if notes else "Insufficient sensor data for interpretation."

    def _build_state_json(self):
        pose = self._get_pose_map()

        speed = None
        if self.last_odom is not None:
            v = self.last_odom.twist.twist.linear
            speed = math.sqrt(v.x * v.x + v.y * v.y)
        
        laser = None
        if self.last_scan is not None and self.last_scan.ranges:
            vals = [r for r in self.last_scan.ranges if math.isfinite(r)]
            if vals:
                laser = {
                    "min_m": float(min(vals)),
                    "mean_m": float(sum(vals) / len(vals)),
                    "max_m": float(max(vals)),
                    "count": int(len(vals)),
                }

        imu = None
        if self.last_imu is not None:
            imu = {
                "angular_velocity": {
                    "x": float(self.last_imu.angular_velocity.x),
                    "y": float(self.last_imu.angular_velocity.y),
                    "z": float(self.last_imu.angular_velocity.z),
                },
                "linear_acceleration": {
                    "x": float(self.last_imu.linear_acceleration.x),
                    "y": float(self.last_imu.linear_acceleration.y),
                    "z": float(self.last_imu.linear_acceleration.z),
                },
            }
        

        nearest = self._nearest_waypoint(pose)

        state = {
            "time_unix": time.time(),
            "pose": pose,
            "speed_mps": speed,
            "laser": laser,
            "imu": imu,
            "nearest_waypoint": None if nearest is None else {
                "index": nearest["index"],
                "name": nearest["name"],
                "distance_m": nearest["distance_m"],
            }
        }
        state["interpretation"] = self._interpret(state)
        return state

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

    def _srv_get_robot_state(self, request, response):
        response.success = True
        response.message = json.dumps(self._build_state_json(), ensure_ascii=False)
        return response

    def _srv_get_nearest_waypoint(self, request, response):
        pose = self._get_pose_map()
        nearest = self._nearest_waypoint(pose)
        response.success = True
        response.message = json.dumps({
            "pose": pose,
            "nearest_waypoint": None if nearest is None else {
                "index": nearest["index"],
                "name": nearest["name"],
                "distance_m": nearest["distance_m"],
                "waypoint": nearest["waypoint"],
            }
        }, ensure_ascii=False)
        return response


def main():
    rclpy.init()
    node = LLMAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
