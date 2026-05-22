#!/usr/bin/env python3
"""
ROS2 LLM Chatbot – CLI interface with agentic tool calling.

Required tools (per assignment):
  1. get_robot_pose()                   – reads current robot pose (no params)
  2. get_waypoints()                    – reads all target waypoints (no params)
  3. calculate_distance(point_a, point_b) – Euclidean distance between two 2-D vectors

Additional tools:
  4. get_robot_state()                  – full sensor snapshot (pose + laser + imu + speed)

Features:
  - Autonomous tool invocation by the LLM agent
  - Token consumption counted per query (prompt + completion + total)
  - Tool call count tracked per query
  - Multilingual: responds in the same language as the user's question
  - Token and tool-call stats printed after every answer
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from pydantic import BaseModel, Field

from langchain_core.messages import HumanMessage, AIMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.tools import tool
from langchain_core.callbacks import BaseCallbackHandler

try:
    from langchain.agents import AgentExecutor
except ImportError:
    from langchain.agents.agent import AgentExecutor

from langchain.agents import create_tool_calling_agent
from langchain_google_genai import ChatGoogleGenerativeAI


# ------------------------------------------------------------------
# System prompt (multilingual)
# ------------------------------------------------------------------
SYSTEM_PROMPT = """You are a ROS2 robot assistant for a TurtleBot3 mobile robot navigating in a Gazebo simulation.

LANGUAGE RULE: Always respond in the SAME language the user uses for their question.

You have access to these tools:
1. get_robot_pose()                      – Returns current robot position (x, y, yaw) in the map frame. No parameters.
2. get_waypoints()                        – Returns all 4 patrol waypoints with names and coordinates. No parameters.
3. calculate_distance(point_a, point_b)  – Calculates Euclidean distance between two 2-D points. Each point is a dict with keys 'x' and 'y'.
4. get_robot_state()                      – Returns full sensor snapshot (pose, speed, laser scan, IMU).

TOOL USAGE STRATEGY:
- "Where is the nearest waypoint?" / "Welcher Punkt ist am nächsten?"
  → call get_robot_pose(), then get_waypoints(), then call calculate_distance() for EACH waypoint → report the minimum.
- "Which waypoint is farthest?" / "Welcher Punkt ist am weitesten entfernt?"
  → call get_robot_pose(), then get_waypoints(), then calculate_distance() for EACH waypoint → report the maximum.
- "How far are all waypoints?" / "Wie weit sind alle Punkte?"
  → call get_robot_pose(), then get_waypoints(), then calculate_distance() for EVERY waypoint → list all distances.
- "What is the robot status?" / "Wie ist der Zustand?"
  → call get_robot_state().
- General distance questions: always use get_robot_pose() + get_waypoints() + calculate_distance().

RESPONSE FORMAT:
- Answer concisely and directly.
- For distance queries include the waypoint names and distances in metres (rounded to 2 decimal places).
- Do not mention internal tool names or JSON in the final answer.
"""

FULL_PROMPT = ChatPromptTemplate.from_messages([
    ("system", SYSTEM_PROMPT),
    ("placeholder", "{chat_history}"),
    ("human", "{input}"),
    ("placeholder", "{agent_scratchpad}"),
])


# ------------------------------------------------------------------
# Callback handler: tracks token usage and tool calls per query
# ------------------------------------------------------------------
class UsageTracker(BaseCallbackHandler):
    """Accumulates token counts and tool-call counts across one agent invocation."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.tool_calls: int = 0
        self.input_tokens: int = 0
        self.output_tokens: int = 0
        self.total_tokens: int = 0

    def on_tool_start(self, serialized, input_str, **kwargs):
        self.tool_calls += 1

    def on_llm_end(self, response, **kwargs):
        # Extract Gemini usage_metadata from each generation
        for gens in response.generations:
            for gen in gens:
                info = getattr(gen, "generation_info", None) or {}
                meta = info.get("usage_metadata", {})
                if meta:
                    self.input_tokens  += meta.get("prompt_token_count", 0)
                    self.output_tokens += meta.get("candidates_token_count", 0)
                    self.total_tokens  += meta.get("total_token_count", 0)


# ------------------------------------------------------------------
# ROS2 client node
# ------------------------------------------------------------------
class RosToolClient(Node):
    def __init__(self):
        super().__init__("llm_chat_cli_agentic")
        self.pose_cli    = self.create_client(Trigger, "/llm_tools/get_robot_pose")
        self.wp_cli      = self.create_client(Trigger, "/llm_tools/get_waypoints")
        self.state_cli   = self.create_client(Trigger, "/llm_tools/get_robot_state")
        self.nearest_cli = self.create_client(Trigger, "/llm_tools/get_nearest_waypoint")

    def call_trigger_json(self, client, timeout: float = 3.0) -> dict:
        if not client.wait_for_service(timeout_sec=timeout):
            return {"error": f"Service {client.srv_name} not available"}
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None:
            return {"error": f"Timeout calling {client.srv_name}"}
        res = fut.result()
        if not res.success:
            return {"error": f"{client.srv_name} returned success=False"}
        try:
            return json.loads(res.message)
        except Exception:
            return {"error": "Invalid JSON", "raw": res.message}


# ------------------------------------------------------------------
# Tool input schema for calculate_distance
# ------------------------------------------------------------------
class DistanceInput(BaseModel):
    point_a: dict = Field(description="First point as a dict with keys 'x' and 'y' (floats).")
    point_b: dict = Field(description="Second point as a dict with keys 'x' and 'y' (floats).")


# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------
def main():
    if not os.getenv("GOOGLE_API_KEY"):
        print("ERROR: GOOGLE_API_KEY not set in environment.")
        return

    rclpy.init()
    node = RosToolClient()

    llm = ChatGoogleGenerativeAI(
        model=os.getenv("GEMINI_MODEL", "gemini-2.5-flash"),
        temperature=0.3,
    )

    # ---- Tool 1: get_robot_pose (no parameters) ----
    @tool("get_robot_pose")
    def get_robot_pose() -> str:
        """Returns the current robot pose (x, y, yaw) in the map frame as JSON. No parameters required."""
        data = node.call_trigger_json(node.pose_cli)
        return json.dumps(data, ensure_ascii=False)

    # ---- Tool 2: get_waypoints (no parameters) ----
    @tool("get_waypoints")
    def get_waypoints() -> str:
        """Returns all 4 patrol waypoints (name, x, y, yaw) as JSON. No parameters required."""
        data = node.call_trigger_json(node.wp_cli)
        return json.dumps(data, ensure_ascii=False)

    # ---- Tool 3: calculate_distance (two vector parameters) ----
    @tool("calculate_distance", args_schema=DistanceInput)
    def calculate_distance(point_a: dict, point_b: dict) -> str:
        """Calculates the Euclidean distance between two 2-D points.
        Each point must be a dict with float keys 'x' and 'y'.
        Example: point_a={'x': 0.0, 'y': 0.0}, point_b={'x': 2.0, 'y': 1.0}
        Returns JSON with 'distance_m', 'point_a', and 'point_b'.
        """
        ax = float(point_a.get("x", 0.0))
        ay = float(point_a.get("y", 0.0))
        bx = float(point_b.get("x", 0.0))
        by = float(point_b.get("y", 0.0))
        dist = math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)
        return json.dumps(
            {"distance_m": round(dist, 4), "point_a": point_a, "point_b": point_b},
            ensure_ascii=False,
        )

    # ---- Tool 4: get_robot_state (full sensor snapshot) ----
    @tool("get_robot_state")
    def get_robot_state() -> str:
        """Returns a full robot sensor snapshot as JSON (pose, speed, laser scan stats, IMU, interpretation)."""
        data = node.call_trigger_json(node.state_cli)
        return json.dumps(data, ensure_ascii=False)

    tools = [get_robot_pose, get_waypoints, calculate_distance, get_robot_state]

    tracker = UsageTracker()

    agent = create_tool_calling_agent(llm, tools, FULL_PROMPT)
    executor = AgentExecutor(
        agent=agent,
        tools=tools,
        verbose=True,   # prints tool calls to terminal – proof of agentic behavior
        handle_parsing_errors=True,
        callbacks=[tracker],
    )

    chat_history = []

    print("=" * 60)
    print("ROS2 LLM Chatbot  |  type 'exit' to quit")
    print("Ask in any language, e.g.:")
    print("  'Where is the nearest waypoint?'")
    print("  'Welcher Punkt ist am weitesten entfernt?'")
    print("  'How far are all waypoints from me?'")
    print("=" * 60)

    while True:
        try:
            user = input("\nYou> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if user.lower() in ("exit", "quit", "q"):
            break
        if not user:
            continue

        chat_history.append(HumanMessage(content=user))
        tracker.reset()

        result = executor.invoke(
            {"input": user, "chat_history": chat_history},
            config={"callbacks": [tracker]},
        )

        output = result["output"]
        print("\nAssistant:\n" + output)

        # --- Usage statistics ---
        stats = (
            f"\n[Stats] Tool calls: {tracker.tool_calls} | "
            f"Tokens: {tracker.total_tokens} "
            f"(prompt: {tracker.input_tokens}, completion: {tracker.output_tokens})"
        )
        print(stats)

        chat_history.append(AIMessage(content=output))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()