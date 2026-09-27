#!/usr/bin/env python3

"""
Warehouse Dispatcher Node
ROS 2 Humble + Nav2

Autonomous mission:

HOME
 -> Station A
 -> Drop Zone 1
 -> HOME

HOME
 -> Station B
 -> Drop Zone 2
 -> HOME

HOME
 -> Station C
 -> Drop Zone 3
 -> HOME

Features:
- Waits for Nav2 before starting
- Waits for valid map -> base_footprint TF
- Sends NavigateToPose goals automatically
- No RViz goal clicks required
- Navigation feedback
- Goal rejection handling
- Retry support
- Navigation timeout
- 2-second simulated pickup
- 2-second simulated delivery
- CSV mission logging
- Graceful mission completion
"""

from __future__ import annotations

import csv
import math
import os
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum, auto
from typing import Optional

import rclpy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer
from tf2_ros import TransformListener


# ============================================================
# WAREHOUSE LOCATIONS
# ============================================================


@dataclass(frozen=True)
class Station:
    """Warehouse navigation position."""

    name: str
    x: float
    y: float
    yaw: float = 0.0


HOME = Station(
    name="Home Base",
    x=0.0,
    y=-3.5,
    yaw=math.pi / 2.0,
)


STATION_A = Station(
    name="Pickup Station A",
    x=-4.0,
    y=4.0,
    yaw=0.0,
)


STATION_B = Station(
    name="Pickup Station B",
    x=0.0,
    y=4.0,
    yaw=0.0,
)


STATION_C = Station(
    name="Pickup Station C",
    x=4.0,
    y=4.0,
    yaw=math.pi,
)


DROP_1 = Station(
    name="Drop Zone 1",
    x=-4.0,
    y=-4.0,
    yaw=0.0,
)


DROP_2 = Station(
    name="Drop Zone 2",
    x=0.0,
    y=-4.0,
    yaw=0.0,
)


DROP_3 = Station(
    name="Drop Zone 3",
    x=4.0,
    y=-4.0,
    yaw=math.pi,
)


# ============================================================
# ORDERS
# ============================================================


@dataclass(frozen=True)
class Order:
    """One warehouse order."""

    order_id: str
    pickup: Station
    drop: Station


ORDERS = (
    Order(
        order_id="ORDER-001",
        pickup=STATION_A,
        drop=DROP_1,
    ),
    Order(
        order_id="ORDER-002",
        pickup=STATION_B,
        drop=DROP_2,
    ),
    Order(
        order_id="ORDER-003",
        pickup=STATION_C,
        drop=DROP_3,
    ),
)


# ============================================================
# MISSION STATES
# ============================================================


class MissionState(Enum):

    WAIT_FOR_NAV2 = auto()

    START_ORDER = auto()

    SEND_PICKUP_GOAL = auto()
    WAIT_PICKUP_GOAL = auto()

    PICKUP_WAIT = auto()

    SEND_DROP_GOAL = auto()
    WAIT_DROP_GOAL = auto()

    DELIVERY_WAIT = auto()

    SEND_HOME_GOAL = auto()
    WAIT_HOME_GOAL = auto()

    ORDER_COMPLETE = auto()

    ALL_COMPLETE = auto()

    FAILED = auto()


# ============================================================
# MAIN DISPATCHER
# ============================================================


class WarehouseDispatcher(Node):
    """Autonomous warehouse mission controller."""

    def __init__(self) -> None:

        super().__init__("warehouse_dispatcher")

        # ----------------------------------------------------
        # Parameters
        # ----------------------------------------------------

        self.declare_parameter(
            "navigation_timeout",
            180.0,
        )

        self.declare_parameter(
            "max_goal_retries",
            2,
        )

        self.declare_parameter(
            "pickup_wait_seconds",
            2.0,
        )

        self.declare_parameter(
            "delivery_wait_seconds",
            2.0,
        )

        self.navigation_timeout = float(
            self.get_parameter(
                "navigation_timeout"
            ).value
        )

        self.max_goal_retries = int(
            self.get_parameter(
                "max_goal_retries"
            ).value
        )

        self.pickup_wait_seconds = float(
            self.get_parameter(
                "pickup_wait_seconds"
            ).value
        )

        self.delivery_wait_seconds = float(
            self.get_parameter(
                "delivery_wait_seconds"
            ).value
        )

        # ----------------------------------------------------
        # Nav2 Action Client
        # ----------------------------------------------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        # ----------------------------------------------------
        # TF
        # ----------------------------------------------------

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ----------------------------------------------------
        # State Machine
        # ----------------------------------------------------

        self.state = (
            MissionState.WAIT_FOR_NAV2
        )

        self.order_index = 0

        self.current_goal: Optional[
            Station
        ] = None

        self.goal_handle = None

        self.goal_active = False
        self.goal_done = False
        self.goal_success = False

        self.goal_retry_count = 0

        self.goal_start_time = 0.0

        self.feedback_last_log = 0.0

        self.wait_until = 0.0

        self.finished = False

        self.goal_lock = threading.Lock()

        # ----------------------------------------------------
        # CSV Logger
        # ----------------------------------------------------

        self.csv_file = None
        self.csv_writer = None

        self._create_csv_log()

        # ----------------------------------------------------
        # Control timer
        # ----------------------------------------------------

        self.control_timer = (
            self.create_timer(
                0.25,
                self._control_loop,
            )
        )

        self._log_info(
            "Dispatcher started. "
            "Waiting for Nav2 action server "
            "and map -> base_footprint TF."
        )

    # ========================================================
    # LOGGING
    # ========================================================

    def _create_csv_log(self) -> None:

        directory = os.path.expanduser(
            "~/.ros/arbotrix_dispatcher"
        )

        os.makedirs(
            directory,
            exist_ok=True,
        )

        timestamp = datetime.now(
            timezone.utc
        ).strftime(
            "%Y%m%dT%H%M%SZ"
        )

        path = os.path.join(
            directory,
            f"mission_{timestamp}.csv",
        )

        self.csv_file = open(
            path,
            "w",
            newline="",
            encoding="utf-8",
        )

        self.csv_writer = csv.writer(
            self.csv_file
        )

        self.csv_writer.writerow(
            [
                "utc_time",
                "simulation_time",
                "order",
                "state",
                "level",
                "message",
            ]
        )

        self.csv_file.flush()

        self.get_logger().info(
            f"Mission CSV log: {path}"
        )

    def _write_csv(
        self,
        level: str,
        message: str,
    ) -> None:

        if self.csv_writer is None:
            return

        try:

            utc_time = datetime.now(
                timezone.utc
            ).isoformat()

            sim_time = (
                self.get_clock()
                .now()
                .nanoseconds
                / 1e9
            )

            if (
                self.order_index
                < len(ORDERS)
            ):
                order_name = (
                    ORDERS[
                        self.order_index
                    ].order_id
                )
            else:
                order_name = "NONE"

            self.csv_writer.writerow(
                [
                    utc_time,
                    f"{sim_time:.3f}",
                    order_name,
                    self.state.name,
                    level,
                    message,
                ]
            )

            self.csv_file.flush()

        except Exception as exc:

            self.get_logger().error(
                f"CSV logging failed: {exc}"
            )

    # IMPORTANT:
    # Separate logging functions avoid the
    # ROS 2 Humble logger severity error.

    def _log_info(
        self,
        message: str,
    ) -> None:

        self.get_logger().info(
            message
        )

        self._write_csv(
            "INFO",
            message,
        )

    def _log_warning(
        self,
        message: str,
    ) -> None:

        self.get_logger().warning(
            message
        )

        self._write_csv(
            "WARNING",
            message,
        )

    def _log_error(
        self,
        message: str,
    ) -> None:

        self.get_logger().error(
            message
        )

        self._write_csv(
            "ERROR",
            message,
        )

    # ========================================================
    # TIME
    # ========================================================

    def _time_seconds(self) -> float:

        return (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

    # ========================================================
    # CURRENT ORDER
    # ========================================================

    def _current_order(
        self,
    ) -> Optional[Order]:

        if (
            self.order_index
            >= len(ORDERS)
        ):
            return None

        return ORDERS[
            self.order_index
        ]

    # ========================================================
    # NAV2 READINESS
    # ========================================================

    def _nav2_ready(self) -> bool:

        return (
            self.nav_client
            .wait_for_server(
                timeout_sec=0.0
            )
        )

    def _tf_ready(self) -> bool:

        try:

            return (
                self.tf_buffer
                .can_transform(
                    "map",
                    "base_footprint",
                    Time(),
                    timeout=Duration(
                        seconds=0.1
                    ),
                )
            )

        except Exception:
            return False

    # ========================================================
    # CREATE POSE
    # ========================================================

    def _create_pose(
        self,
        target: Station,
    ) -> PoseStamped:

        pose = PoseStamped()

        pose.header.frame_id = "map"

        pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        pose.pose.position.x = (
            target.x
        )

        pose.pose.position.y = (
            target.y
        )

        pose.pose.position.z = 0.0

        half_yaw = (
            target.yaw / 2.0
        )

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0

        pose.pose.orientation.z = (
            math.sin(
                half_yaw
            )
        )

        pose.pose.orientation.w = (
            math.cos(
                half_yaw
            )
        )

        return pose

    # ========================================================
    # SEND NAVIGATION GOAL
    # ========================================================

    def _send_goal(
        self,
        target: Station,
    ) -> None:

        if self.goal_active:
            return

        self.current_goal = target

        self.goal_done = False
        self.goal_success = False
        self.goal_active = True

        self.goal_start_time = (
            self._time_seconds()
        )

        goal_msg = (
            NavigateToPose.Goal()
        )

        goal_msg.pose = (
            self._create_pose(
                target
            )
        )

        self._log_info(
            f"Sending Nav2 goal -> "
            f"{target.name} "
            f"(x={target.x:.2f}, "
            f"y={target.y:.2f})"
        )

        try:

            future = (
                self.nav_client
                .send_goal_async(
                    goal_msg,
                    feedback_callback=(
                        self._feedback_callback
                    ),
                )
            )

            future.add_done_callback(
                self._goal_response_callback
            )

        except Exception as exc:

            self.goal_active = False
            self.goal_done = True
            self.goal_success = False

            self._log_error(
                f"Could not send goal: "
                f"{exc}"
            )

    # ========================================================
    # GOAL ACCEPT / REJECT
    # ========================================================

    def _goal_response_callback(
        self,
        future,
    ) -> None:

        try:

            goal_handle = (
                future.result()
            )

        except Exception as exc:

            self.goal_active = False
            self.goal_done = True
            self.goal_success = False

            self._log_error(
                f"Goal request exception: "
                f"{exc}"
            )

            return

        if not goal_handle.accepted:

            self.goal_active = False
            self.goal_done = True
            self.goal_success = False

            self._log_warning(
                "Nav2 goal REJECTED!"
            )

            return

        self.goal_handle = (
            goal_handle
        )

        self._log_info(
            "Nav2 goal accepted."
        )

        result_future = (
            goal_handle
            .get_result_async()
        )

        result_future.add_done_callback(
            self._goal_result_callback
        )

    # ========================================================
    # GOAL RESULT
    # ========================================================

    def _goal_result_callback(
        self,
        future,
    ) -> None:

        try:

            result_response = (
                future.result()
            )

            status = (
                result_response.status
            )

        except Exception as exc:

            self.goal_active = False
            self.goal_done = True
            self.goal_success = False

            self._log_error(
                f"Navigation result "
                f"exception: {exc}"
            )

            return

        success = (
            status
            == GoalStatus.STATUS_SUCCEEDED
        )

        self.goal_active = False
        self.goal_done = True
        self.goal_success = success

        if success:

            if (
                self.current_goal
                is not None
            ):

                self._log_info(
                    f"ARRIVED at "
                    f"{self.current_goal.name}."
                )

        else:

            self._log_warning(
                f"Navigation failed. "
                f"Status code = {status}"
            )

    # ========================================================
    # NAVIGATION FEEDBACK
    # ========================================================

    def _feedback_callback(
        self,
        feedback_msg,
    ) -> None:

        try:

            distance = float(
                feedback_msg
                .feedback
                .distance_remaining
            )

        except Exception:
            return

        now = self._time_seconds()

        if (
            now
            - self.feedback_last_log
            < 2.0
        ):
            return

        self.feedback_last_log = now

        self.get_logger().info(
            f"Distance remaining: "
            f"{distance:.2f} m"
        )

    # ========================================================
    # CANCEL GOAL
    # ========================================================

    def _cancel_goal(self) -> None:

        if self.goal_handle is None:
            return

        try:

            self.goal_handle \
                .cancel_goal_async()

            self._log_warning(
                "Navigation cancel requested."
            )

        except Exception as exc:

            self._log_error(
                f"Goal cancellation "
                f"failed: {exc}"
            )

    # ========================================================
    # RESET GOAL
    # ========================================================

    def _reset_goal(self) -> None:

        with self.goal_lock:

            self.goal_active = False
            self.goal_done = False
            self.goal_success = False

        self.goal_handle = None
        self.current_goal = None

        self.goal_start_time = 0.0

    # ========================================================
    # HANDLE NAVIGATION RESULT
    # ========================================================

    def _handle_goal_result(
        self,
        target: Station,
        retry_state: MissionState,
        success_state: MissionState,
    ) -> None:

        # ----------------------------------------------------
        # Timeout
        # ----------------------------------------------------

        if self.goal_active:

            elapsed = (
                self._time_seconds()
                - self.goal_start_time
            )

            if (
                elapsed
                > self.navigation_timeout
            ):

                self._log_warning(
                    f"Navigation timeout "
                    f"for {target.name}."
                )

                self._cancel_goal()

                self.goal_active = False
                self.goal_done = True
                self.goal_success = False

        # ----------------------------------------------------
        # Still navigating
        # ----------------------------------------------------

        if not self.goal_done:
            return

        # ----------------------------------------------------
        # Success
        # ----------------------------------------------------

        if self.goal_success:

            self.goal_retry_count = 0

            self._reset_goal()

            self.state = (
                success_state
            )

            return

        # ----------------------------------------------------
        # Failure -> retry
        # ----------------------------------------------------

        self.goal_retry_count += 1

        if (
            self.goal_retry_count
            <= self.max_goal_retries
        ):

            self._log_warning(
                f"Retrying {target.name}. "
                f"Attempt "
                f"{self.goal_retry_count}/"
                f"{self.max_goal_retries}"
            )

            self._reset_goal()

            self.state = (
                retry_state
            )

        else:

            self._log_error(
                f"Navigation to "
                f"{target.name} failed "
                f"after maximum retries."
            )

            self._reset_goal()

            self.state = (
                MissionState.FAILED
            )

    # ========================================================
    # CONTROL LOOP
    # ========================================================

    def _control_loop(
        self,
    ) -> None:

        try:

            if (
                self.state
                == MissionState.WAIT_FOR_NAV2
            ):

                self._state_wait_nav2()

            elif (
                self.state
                == MissionState.START_ORDER
            ):

                self._state_start_order()

            elif (
                self.state
                == MissionState.SEND_PICKUP_GOAL
            ):

                self._state_send_pickup()

            elif (
                self.state
                == MissionState.WAIT_PICKUP_GOAL
            ):

                self._state_wait_pickup()

            elif (
                self.state
                == MissionState.PICKUP_WAIT
            ):

                self._state_pickup_wait()

            elif (
                self.state
                == MissionState.SEND_DROP_GOAL
            ):

                self._state_send_drop()

            elif (
                self.state
                == MissionState.WAIT_DROP_GOAL
            ):

                self._state_wait_drop()

            elif (
                self.state
                == MissionState.DELIVERY_WAIT
            ):

                self._state_delivery_wait()

            elif (
                self.state
                == MissionState.SEND_HOME_GOAL
            ):

                self._state_send_home()

            elif (
                self.state
                == MissionState.WAIT_HOME_GOAL
            ):

                self._state_wait_home()

            elif (
                self.state
                == MissionState.ORDER_COMPLETE
            ):

                self._state_order_complete()

            elif (
                self.state
                == MissionState.ALL_COMPLETE
            ):

                self._state_all_complete()

            elif (
                self.state
                == MissionState.FAILED
            ):

                self._state_failed()

        except Exception as exc:

            self._log_error(
                f"Dispatcher exception: "
                f"{exc}"
            )

            self.state = (
                MissionState.FAILED
            )

    # ========================================================
    # WAIT FOR NAV2
    # ========================================================

    def _state_wait_nav2(
        self,
    ) -> None:

        if not self._nav2_ready():
            return

        if not self._tf_ready():
            return

        self._log_info(
            "Nav2 READY and TF chain "
            "map -> odom -> "
            "base_footprint is available."
        )

        self.state = (
            MissionState.START_ORDER
        )

    # ========================================================
    # START ORDER
    # ========================================================

    def _state_start_order(
        self,
    ) -> None:

        order = (
            self._current_order()
        )

        if order is None:

            self.state = (
                MissionState.ALL_COMPLETE
            )

            return

        self._log_info(
            "================================"
        )

        self._log_info(
            f"Starting {order.order_id}: "
            f"{order.pickup.name} -> "
            f"{order.drop.name} -> HOME"
        )

        self._log_info(
            "================================"
        )

        self.goal_retry_count = 0

        self.state = (
            MissionState.SEND_PICKUP_GOAL
        )

    # ========================================================
    # PICKUP NAVIGATION
    # ========================================================

    def _state_send_pickup(
        self,
    ) -> None:

        order = (
            self._current_order()
        )

        if order is None:

            self.state = (
                MissionState.FAILED
            )

            return

        self._log_info(
            f"Navigating to "
            f"{order.pickup.name}."
        )

        self._send_goal(
            order.pickup
        )

        self.state = (
            MissionState.WAIT_PICKUP_GOAL
        )

    def _state_wait_pickup(
        self,
    ) -> None:

        order = (
            self._current_order()
        )

        if order is None:

            self.state = (
                MissionState.FAILED
            )

            return

        old_state = self.state

        self._handle_goal_result(
            target=order.pickup,
            retry_state=(
                MissionState.SEND_PICKUP_GOAL
            ),
            success_state=(
                MissionState.PICKUP_WAIT
            ),
        )

        if (
            old_state
            != self.state
            and self.state
            == MissionState.PICKUP_WAIT
        ):

            self.wait_until = (
                self._time_seconds()
                + self.pickup_wait_seconds
            )

            self._log_info(
                f"Picking up package "
                f"({self.pickup_wait_seconds:.1f}s)."
            )

    # ========================================================
    # PICKUP WAIT
    # ========================================================

    def _state_pickup_wait(
        self,
    ) -> None:

        if (
            self._time_seconds()
            < self.wait_until
        ):
            return

        self._log_info(
            "PACKAGE PICKED UP."
        )

        self.goal_retry_count = 0

        self.state = (
            MissionState.SEND_DROP_GOAL
        )

    # ========================================================
    # DROP NAVIGATION
    # ========================================================

    def _state_send_drop(
        self,
    ) -> None:

        order = (
            self._current_order()
        )

        if order is None:

            self.state = (
                MissionState.FAILED
            )

            return

        self._log_info(
            f"Navigating to "
            f"{order.drop.name}."
        )

        self._send_goal(
            order.drop
        )

        self.state = (
            MissionState.WAIT_DROP_GOAL
        )

    def _state_wait_drop(
        self,
    ) -> None:

        order = (
            self._current_order()
        )

        if order is None:

            self.state = (
                MissionState.FAILED
            )

            return

        old_state = self.state

        self._handle_goal_result(
            target=order.drop,
            retry_state=(
                MissionState.SEND_DROP_GOAL
            ),
            success_state=(
                MissionState.DELIVERY_WAIT
            ),
        )

        if (
            old_state
            != self.state
            and self.state
            == MissionState.DELIVERY_WAIT
        ):

            self.wait_until = (
                self._time_seconds()
                + self.delivery_wait_seconds
            )

            self._log_info(
                f"Delivering package "
                f"({self.delivery_wait_seconds:.1f}s)."
            )

    # ========================================================
    # DELIVERY WAIT
    # ========================================================

    def _state_delivery_wait(
        self,
    ) -> None:

        if (
            self._time_seconds()
            < self.wait_until
        ):
            return

        self._log_info(
            "PACKAGE DELIVERED."
        )

        self.goal_retry_count = 0

        self.state = (
            MissionState.SEND_HOME_GOAL
        )

    # ========================================================
    # HOME NAVIGATION
    # ========================================================

    def _state_send_home(
        self,
    ) -> None:

        self._log_info(
            "Returning to Home Base."
        )

        self._send_goal(
            HOME
        )

        self.state = (
            MissionState.WAIT_HOME_GOAL
        )

    def _state_wait_home(
        self,
    ) -> None:

        self._handle_goal_result(
            target=HOME,
            retry_state=(
                MissionState.SEND_HOME_GOAL
            ),
            success_state=(
                MissionState.ORDER_COMPLETE
            ),
        )

    # ========================================================
    # ORDER COMPLETE
    # ========================================================

    def _state_order_complete(
        self,
    ) -> None:

        order = (
            self._current_order()
        )

        if order is not None:

            self._log_info(
                f"{order.order_id} COMPLETE."
            )

        self.order_index += 1

        if (
            self.order_index
            >= len(ORDERS)
        ):

            self.state = (
                MissionState.ALL_COMPLETE
            )

        else:

            self.state = (
                MissionState.START_ORDER
            )

    # ========================================================
    # ALL ORDERS COMPLETE
    # ========================================================

    def _state_all_complete(
        self,
    ) -> None:

        self._log_info(
            "================================"
        )

        self._log_info(
            "ALL ORDERS COMPLETE"
        )

        self._log_info(
            "Robot is at Home Base."
        )

        self._log_info(
            "================================"
        )

        self.finished = True

        self.control_timer.cancel()

    # ========================================================
    # FAILURE
    # ========================================================

    def _state_failed(
        self,
    ) -> None:

        self._log_error(
            "MISSION FAILED."
        )

        if self.goal_active:
            self._cancel_goal()

        self.finished = True

        self.control_timer.cancel()

    # ========================================================
    # CLEANUP
    # ========================================================

    def cleanup(
        self,
    ) -> None:

        if self.goal_active:
            self._cancel_goal()

        if self.csv_file is not None:

            try:

                self.csv_file.flush()
                self.csv_file.close()

            except Exception:
                pass

            self.csv_file = None
            self.csv_writer = None


# ============================================================
# MAIN
# ============================================================


def main(args=None) -> None:

    rclpy.init(
        args=args
    )

    node = (
        WarehouseDispatcher()
    )

    executor = (
        MultiThreadedExecutor(
            num_threads=2
        )
    )

    executor.add_node(
        node
    )

    try:

        while (
            rclpy.ok()
            and not node.finished
        ):

            executor.spin_once(
                timeout_sec=0.5
            )

    except KeyboardInterrupt:

        node.get_logger().info(
            "Dispatcher interrupted."
        )

    except Exception as exc:

        node.get_logger().error(
            f"Fatal dispatcher error: "
            f"{exc}"
        )

    finally:

        node.cleanup()

        executor.remove_node(
            node
        )

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":
    main()
