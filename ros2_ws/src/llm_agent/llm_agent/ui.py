#!/usr/bin/env python3

import json
import math
import os

import streamlit as st

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from pydantic import BaseModel, Field

from langchain_core.messages import HumanMessage, AIMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.tools import tool
from langchain_core.callbacks import BaseCallbackHandler

from langchain_classic.agents import AgentExecutor, create_tool_calling_agent


from langchain_google_genai import ChatGoogleGenerativeAI


# ------------------------------------------------------------------
# System prompt (multilingual)
# ------------------------------------------------------------------
SYSTEM_PROMPT = """You are a ROS2 robot assistant for a TurtleBot3 mobile robot navigating in a Gazebo simulation.

LANGUAGE RULE: Always respond in the SAME language the user uses for their question.

You have access to these tools:
1. get_robot_pose()                      - Returns current robot position (x, y, yaw) in the map frame. No parameters.
2. get_waypoints()                        - Returns all 4 patrol waypoints with names and coordinates. No parameters.
3. calculate_distance(point_a, point_b)  - Calculates Euclidean distance between two 2-D points. Each point is a dict with keys 'x' and 'y'.

TOOL USAGE STRATEGY:
- "Where is the nearest waypoint?" 
  -> call get_robot_pose(), then get_waypoints(), then calculate_distance() for EACH waypoint → report the minimum.
- "Which waypoint is farthest?" 
  -> call get_robot_pose(), then get_waypoints(), then calculate_distance() for EACH waypoint → report the maximum.
- "How far are all waypoints?"
  -> call get_robot_pose(), then get_waypoints(), then calculate_distance() for EVERY waypoint → list all distances.
- General distance questions: always use get_robot_pose() + get_waypoints() + calculate_distance().

RESPONSE FORMAT:
- Answer concisely and directly.
- For distance queries include waypoint names and distances in metres (rounded to 2 decimal places).
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
        for gens in response.generations:
            for gen in gens:
                msg = getattr(gen, "message", None)
                if msg:
                    meta = getattr(msg, "usage_metadata", None) or {}
                    if meta:
                        self.input_tokens  += meta.get("input_tokens", 0)
                        self.output_tokens += meta.get("output_tokens", 0)
                        self.total_tokens  += meta.get("total_tokens", 0)    
# ------------------------------------------------------------------
# Tool input schema for calculate_distance
# ------------------------------------------------------------------
class DistanceInput(BaseModel):
    point_a: dict = Field(description="First point as a dict with keys 'x' and 'y' (floats).")
    point_b: dict = Field(description="Second point as a dict with keys 'x' and 'y' (floats).")


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------
def model_name() -> str:
    return os.getenv("GEMINI_MODEL", "gemini-2.5-flash")


def call_trigger_json(node: Node, client, timeout: float = 3.0) -> dict:
    if not client.wait_for_service(timeout_sec=timeout):
        return {"error": f"Service {client.srv_name} not available"}
    fut = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if fut.result() is None:
        return {"error": f"Timeout calling {client.srv_name}"}
    res = fut.result()
    if not res.success:
        return {"error": f"{client.srv_name} returned success=False"}
    try:
        return json.loads(res.message)
    except Exception:
        return {"error": "Invalid JSON returned", "raw": res.message}


# ------------------------------------------------------------------
# Cached ROS resources (one per Streamlit process)
# ------------------------------------------------------------------
@st.cache_resource
def ros_node_and_clients():
    rclpy.init(args=None)
    node = Node("llm_streamlit_agentic_ui")
    pose_cli    = node.create_client(Trigger, "/llm_tools/get_robot_pose")
    wp_cli      = node.create_client(Trigger, "/llm_tools/get_waypoints")
    return node, pose_cli, wp_cli


@st.cache_resource
def build_agent_executor():
    if not os.getenv("GOOGLE_API_KEY"):
        raise RuntimeError("GOOGLE_API_KEY is not set.")

    node, pose_cli, wp_cli = ros_node_and_clients()
    llm = ChatGoogleGenerativeAI(model=model_name(), temperature=0.3)

    # ---- Tool 1: get_robot_pose (no parameters) ----
    @tool("get_robot_pose")
    def get_robot_pose() -> str:
        """Returns the current robot pose (x, y, yaw) in the map frame as JSON. No parameters required."""
        data = call_trigger_json(node, pose_cli)
        return json.dumps(data, ensure_ascii=False)

    # ---- Tool 2: get_waypoints (no parameters) ----
    @tool("get_waypoints")
    def get_waypoints() -> str:
        """Returns all 4 patrol waypoints (name, x, y, yaw) as JSON. No parameters required."""
        data = call_trigger_json(node, wp_cli)
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

    tools = [get_robot_pose, get_waypoints, calculate_distance]
    agent = create_tool_calling_agent(llm, tools, FULL_PROMPT)
    executor = AgentExecutor(
        agent=agent,
        tools=tools,
        verbose=False,
        handle_parsing_errors=True,
    )
    return executor


# ------------------------------------------------------------------
# Streamlit UI
# ------------------------------------------------------------------
def main():
    st.set_page_config(page_title="ROS2 LLM Chatbot", layout="centered")
    st.title("ROS2 LLM Chatbot")

    with st.sidebar:
        st.markdown("**Configuration**")
        st.markdown(f"- Model: `{model_name()}`")
        st.markdown("- Tools: `get_robot_pose` · `get_waypoints` · `calculate_distance` ")
        st.markdown("- Language: multilingual (responds in user's language)")
        st.divider()
        if st.button("Reset chat"):
            st.session_state.history = []
            st.session_state.lc_history = []
            st.rerun()

    executor = build_agent_executor()

    if "history" not in st.session_state:
        st.session_state.history = []      # display history
    if "lc_history" not in st.session_state:
        st.session_state.lc_history = []   # LangChain message objects

    # Render existing chat history
    for msg in st.session_state.history:
        with st.chat_message(msg["role"]):
            st.markdown(msg["text"])
            if msg["role"] == "assistant" and "stats" in msg:
                st.caption(msg["stats"])

    user_text = st.chat_input(
        "Ask anything, e.g. 'Where is the nearest waypoint?' / 'Welcher Punkt ist am weitesten?'"
    )

    if user_text:
        # Show user message
        st.session_state.history.append({"role": "user", "text": user_text})
        with st.chat_message("user"):
            st.markdown(user_text)

        st.session_state.lc_history.append(HumanMessage(content=user_text))

        # Agentic tool calling with usage tracking
        tracker = UsageTracker()

        with st.chat_message("assistant"):
            with st.spinner("Generating answer..."):
                result = executor.invoke(
                    {"input": user_text, "chat_history": st.session_state.lc_history},
                    config={"callbacks": [tracker]},
                )
              
                answer = result["output"]

            st.markdown(answer)


            stats = (
                f"Tool calls: **{tracker.tool_calls}** | "
                f"Tokens: **{tracker.total_tokens}** "
                f"(prompt: {tracker.input_tokens}, completion: {tracker.output_tokens})"
            )
            st.caption(stats)

        st.session_state.history.append({
            "role": "assistant",
            "text": answer,
            "stats": stats,
        })
        st.session_state.lc_history.append(AIMessage(content=answer))


if __name__ == "__main__":
    main()