"""Thread-safe asynchronous wrapper around Nav2 NavigateToPose."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavigationState(Enum):
    """High-level state of the current navigation request."""

    IDLE = auto()
    SENDING = auto()
    ACTIVE = auto()
    SUCCEEDED = auto()
    FAILED = auto()
    REJECTED = auto()
    CANCELED = auto()
    TIMED_OUT = auto()


@dataclass(frozen=True)
class NavigationResult:
    """Final navigation outcome."""

    state: NavigationState
    message: str
    attempts: int


class NavClient:
    """Asynchronous NavigateToPose client with retry and timeout support.

    This class deliberately uses the caller's ROS node rather than creating a
    second node/process. That reduces memory and DDS endpoints on small VMs.
    """

    _TERMINAL_STATES = {
        NavigationState.SUCCEEDED,
        NavigationState.FAILED,
        NavigationState.REJECTED,
        NavigationState.CANCELED,
        NavigationState.TIMED_OUT,
    }

    def __init__(
        self,
        node: Node,
        action_name: str = "navigate_to_pose",
        goal_timeout_sec: float = 180.0,
        max_retries: int = 2,
        feedback_log_period_sec: float = 2.0,
    ) -> None:
        self._node = node
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._goal_timeout_sec = float(goal_timeout_sec)
        self._max_retries = int(max_retries)
        self._feedback_log_period_sec = float(feedback_log_period_sec)

        self._lock = threading.RLock()
        self._state = NavigationState.IDLE
        self._result: Optional[NavigationResult] = None
        self._attempts = 0
        self._target: Optional[PoseStamped] = None
        self._target_label = ""
        self._goal_handle = None
        self._send_goal_future = None
        self._result_future = None
        self._cancel_future = None
        self._attempt_started_monotonic = 0.0
        self._last_feedback_log_monotonic = 0.0
        self._distance_remaining: Optional[float] = None
        self._timeout_cancel_requested = False

    def server_is_ready(self) -> bool:
        """Return True when the Nav2 action server is discoverable."""
        return self._client.server_is_ready()

    @property
    def state(self) -> NavigationState:
        with self._lock:
            return self._state

    @property
    def distance_remaining(self) -> Optional[float]:
        with self._lock:
            return self._distance_remaining

    @property
    def is_busy(self) -> bool:
        with self._lock:
            return self._state in {NavigationState.SENDING, NavigationState.ACTIVE}

    @property
    def is_done(self) -> bool:
        with self._lock:
            return self._state in self._TERMINAL_STATES

    @property
    def result(self) -> Optional[NavigationResult]:
        with self._lock:
            return self._result

    def navigate_to(
        self,
        x: float,
        y: float,
        yaw: float,
        label: str,
        frame_id: str = "map",
    ) -> None:
        """Start a new asynchronous navigation request.

        Raises:
            RuntimeError: if another goal is still active or Nav2 is not ready.
        """
        if not self.server_is_ready():
            raise RuntimeError("Nav2 NavigateToPose action server is not ready")

        with self._lock:
            if self._state in {NavigationState.SENDING, NavigationState.ACTIVE}:
                raise RuntimeError("A navigation goal is already active")

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = self._node.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
            pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

            self._target = pose
            self._target_label = str(label)
            self._attempts = 0
            self._result = None
            self._distance_remaining = None
            self._timeout_cancel_requested = False
            self._goal_handle = None

        self._send_attempt()

    def poll_timeout(self) -> None:
        """Cancel an active goal when its wall-clock timeout expires.

        Wall clock is intentional: if simulation time stalls, the dispatcher
        must still be able to detect the hung mission and fail safely.
        """
        with self._lock:
            if self._state != NavigationState.ACTIVE:
                return
            if self._timeout_cancel_requested or self._goal_handle is None:
                return
            elapsed = time.monotonic() - self._attempt_started_monotonic
            if elapsed <= self._goal_timeout_sec:
                return
            self._timeout_cancel_requested = True
            goal_handle = self._goal_handle
            attempt = self._attempts
            label = self._target_label

        self._node.get_logger().error(
            f"Navigation timeout after {self._goal_timeout_sec:.0f}s "
            f"for {label} (attempt {attempt}); canceling goal"
        )
        self._cancel_future = goal_handle.cancel_goal_async()
        self._cancel_future.add_done_callback(self._cancel_response_callback)

    def cancel(self) -> None:
        """Request cancellation of the active goal."""
        with self._lock:
            if self._state != NavigationState.ACTIVE or self._goal_handle is None:
                return
            self._timeout_cancel_requested = False
            goal_handle = self._goal_handle
        self._cancel_future = goal_handle.cancel_goal_async()
        self._cancel_future.add_done_callback(self._cancel_response_callback)

    def _send_attempt(self) -> None:
        with self._lock:
            if self._target is None:
                raise RuntimeError("No target has been configured")
            if not self._client.server_is_ready():
                self._finalize(
                    NavigationState.FAILED,
                    "NavigateToPose action server disappeared before retry",
                )
                return

            self._attempts += 1
            self._state = NavigationState.SENDING
            self._goal_handle = None
            self._timeout_cancel_requested = False
            self._distance_remaining = None
            self._attempt_started_monotonic = time.monotonic()
            self._last_feedback_log_monotonic = 0.0

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self._target
            attempt = self._attempts
            label = self._target_label

        self._node.get_logger().info(
            f"Sending Nav2 goal to {label} "
            f"(attempt {attempt}/{self._max_retries + 1})"
        )
        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS future exceptions vary
            self._handle_attempt_failure(
                NavigationState.FAILED,
                f"Exception while sending goal: {exc}",
            )
            return

        if not goal_handle.accepted:
            self._node.get_logger().error(
                f"Nav2 goal REJECTED for {self._target_label}"
            )
            self._handle_attempt_failure(
                NavigationState.REJECTED,
                "Nav2 action server rejected the goal",
            )
            return

        with self._lock:
            self._goal_handle = goal_handle
            self._state = NavigationState.ACTIVE
            self._attempt_started_monotonic = time.monotonic()

        self._node.get_logger().info(
            f"Nav2 goal accepted for {self._target_label}"
        )
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        distance = float(feedback.distance_remaining)
        now_mono = time.monotonic()

        should_log = False
        with self._lock:
            self._distance_remaining = distance
            if (
                now_mono - self._last_feedback_log_monotonic
                >= self._feedback_log_period_sec
            ):
                self._last_feedback_log_monotonic = now_mono
                should_log = True

        if should_log:
            self._node.get_logger().info(
                f"Navigating to {self._target_label}: "
                f"{distance:.2f} m remaining"
            )

    def _result_callback(self, future) -> None:
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as exc:  # noqa: BLE001
            self._handle_attempt_failure(
                NavigationState.FAILED,
                f"Exception while receiving navigation result: {exc}",
            )
            return

        with self._lock:
            timed_out = self._timeout_cancel_requested

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._finalize(
                NavigationState.SUCCEEDED,
                f"Reached {self._target_label}",
            )
            return

        if status == GoalStatus.STATUS_CANCELED:
            if timed_out:
                self._handle_attempt_failure(
                    NavigationState.TIMED_OUT,
                    f"Navigation timed out for {self._target_label}",
                )
            else:
                self._finalize(
                    NavigationState.CANCELED,
                    f"Navigation canceled for {self._target_label}",
                )
            return

        status_name = {
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        }.get(status, f"STATUS_{status}")
        self._handle_attempt_failure(
            NavigationState.FAILED,
            f"Navigation finished with {status_name} for {self._target_label}",
        )

    def _cancel_response_callback(self, future) -> None:
        try:
            response = future.result()
            count = len(response.goals_canceling)
            self._node.get_logger().warning(
                f"Cancel request acknowledged for {count} goal(s)"
            )
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f"Goal cancellation failed: {exc}")

    def _handle_attempt_failure(
        self,
        final_state: NavigationState,
        message: str,
    ) -> None:
        with self._lock:
            attempts = self._attempts
            can_retry = attempts <= self._max_retries

        if can_retry:
            self._node.get_logger().warning(
                f"{message}. Retrying ({attempts + 1}/{self._max_retries + 1})"
            )
            self._send_attempt()
            return

        self._finalize(final_state, message)

    def _finalize(self, state: NavigationState, message: str) -> None:
        with self._lock:
            self._state = state
            self._result = NavigationResult(state, message, self._attempts)
            self._goal_handle = None
            self._timeout_cancel_requested = False

        log = self._node.get_logger()
        if state == NavigationState.SUCCEEDED:
            log.info(message)
        elif state in {NavigationState.REJECTED, NavigationState.TIMED_OUT}:
            log.error(message)
        else:
            log.warning(message)
