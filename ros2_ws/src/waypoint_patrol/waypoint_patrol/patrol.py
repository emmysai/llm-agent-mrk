#!/usr/bin/env python3
import math
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


def yaw_to_quat(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)


class WaypointPatrol(Node):
    def __init__(self):
        super().__init__("waypoint_patrol")

        self.declare_parameter("waypoints_yaml", "")
        yaml_path = self.get_parameter("waypoints_yaml").get_parameter_value().string_value
        if not yaml_path:
            pkg_dir = Path(__file__).resolve().parents[1]
            yaml_path = str(pkg_dir / "config" / "waypoints.yaml")

        self.frame_id, self.waypoints = self._load_waypoints(yaml_path)
        if len(self.waypoints) < 1:
            raise RuntimeError("No waypoints loaded")

        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.idx = 0
        self.in_flight = False

        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {yaml_path}")

    def _load_waypoints(self, yaml_path: str):
        data = yaml.safe_load(Path(yaml_path).read_text())
        frame_id = data.get("frame_id", "map")
        wps = data.get("waypoints", [])
        return frame_id, wps

    def _make_goal(self, wp):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp["x"])
        goal.pose.pose.position.y = float(wp["y"])
        x, y, z, w = yaw_to_quat(float(wp.get("yaw", 0.0)))
        goal.pose.pose.orientation.x = x
        goal.pose.pose.orientation.y = y
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w
        return goal

    def _tick(self):
        if self.in_flight:
            return

        wp = self.waypoints[self.idx]
        name = wp.get("name", f"wp{self.idx}")
        self.in_flight = True

        self.get_logger().info(f"Navigate to {name}: x={wp['x']} y={wp['y']} yaw={wp.get('yaw',0.0)}")

        self.client.wait_for_server()
        goal = self._make_goal(wp)
        send_future = self.client.send_goal_async(goal)

        def goal_response_cb(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Goal rejected. Retrying...")
                self.in_flight = False
                return

            result_future = goal_handle.get_result_async()

            def result_cb(rfut):
                status = rfut.result().status
                # 4 == SUCCEEDED (action_msgs/GoalStatus)
                if status == 4:
                    self.get_logger().info("Reached ✓")
                    self.idx = (self.idx + 1) % len(self.waypoints)
                else:
                    self.get_logger().warn(f"Nav failed (status={status}). Retry same waypoint.")
                self.in_flight = False

            result_future.add_done_callback(result_cb)

        send_future.add_done_callback(goal_response_cb)


def main():
    rclpy.init()
    node = WaypointPatrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
