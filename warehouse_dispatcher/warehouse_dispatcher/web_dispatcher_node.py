#!/usr/bin/env python3

import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Float32


class WebWarehouseDispatcher(Node):

    PICKUPS = {
        "A": (-4.0, 4.0),
        "B": (0.0, 4.0),
        "C": (4.0, 4.0),
    }

    DROPS = {
        "1": (-4.0, -4.0),
        "2": (0.0, -4.0),
        "3": (4.0, -4.0),
    }

    def __init__(self):
        super().__init__("web_warehouse_dispatcher")

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose"
        )

        self.command_sub = self.create_subscription(
            String,
            "/warehouse_dispatcher/web/command",
            self.command_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/warehouse_dispatcher/web/status",
            10
        )

        self.distance_pub = self.create_publisher(
            Float32,
            "/warehouse_dispatcher/web/distance",
            10
        )

        self.queue_pub = self.create_publisher(
            String,
            "/warehouse_dispatcher/web/queue",
            10
        )

        self.history_pub = self.create_publisher(
            String,
            "/warehouse_dispatcher/web/history",
            10
        )

        self.pending_orders = []
        self.history = []

        self.current_order = None
        self.current_stage = "IDLE"
        self.robot_state = "IDLE"
        self.detail = "Robot ready"
        self.distance_remaining = -1.0

        self.goal_handle = None
        self.order_counter = 0

        self.create_timer(0.5, self.process_queue)
        self.create_timer(1.0, self.publish_all)

        self.get_logger().info("Web warehouse dispatcher ready.")

    # ------------------------------------------------------

    def command_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
        except Exception:
            self.get_logger().error("Invalid JSON command")
            return

        command_type = cmd.get("type")

        if command_type == "enqueue":
            pickup = str(cmd.get("pickup", ""))
            drop = str(cmd.get("drop", ""))

            if pickup not in self.PICKUPS or drop not in self.DROPS:
                self.get_logger().error("Invalid pickup/drop")
                return

            self.order_counter += 1

            order = {
                "id": self.order_counter,
                "pickup": pickup,
                "drop": drop
            }

            self.pending_orders.append(order)

            self.add_history(
                order["id"],
                "QUEUED",
                f"Station {pickup} -> Drop Zone {drop}"
            )

            self.get_logger().info(
                f"Order {order['id']} queued: "
                f"Station {pickup} -> Zone {drop}"
            )

            self.publish_all()

        elif command_type == "clear_queue":
            self.pending_orders.clear()
            self.publish_all()

        elif command_type == "cancel":
            if self.goal_handle is not None:
                self.get_logger().warning("Cancelling active navigation")
                self.goal_handle.cancel_goal_async()

    # ------------------------------------------------------

    def process_queue(self):
        if self.current_order is not None:
            return

        if not self.pending_orders:
            return

        if not self.action_client.server_is_ready():
            self.detail = "Waiting for Nav2..."
            return

        self.current_order = self.pending_orders.pop(0)

        self.add_history(
            self.current_order["id"],
            "STARTED",
            "Mission started"
        )

        self.navigate_to_pickup()

    # ------------------------------------------------------

    def navigate_to_pickup(self):
        if self.current_order is None:
            return

        station = self.current_order["pickup"]
        x, y = self.PICKUPS[station]

        self.current_stage = "PICKUP"
        self.send_nav_goal(
            x,
            y,
            f"Pickup Station {station}"
        )

    # ------------------------------------------------------

    def navigate_to_drop(self):
        if self.current_order is None:
            return

        zone = self.current_order["drop"]
        x, y = self.DROPS[zone]

        self.current_stage = "DROP"
        self.send_nav_goal(
            x,
            y,
            f"Drop Zone {zone}"
        )

    # ------------------------------------------------------

    def send_nav_goal(self, x, y, target_name):

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.fail_current("Nav2 action server unavailable")
            return

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.robot_state = "NAVIGATING"
        self.detail = f"Navigating to {target_name}"
        self.distance_remaining = -1.0

        self.get_logger().info(self.detail)

        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    # ------------------------------------------------------

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.fail_current("Nav2 goal rejected")
            return

        self.goal_handle = goal_handle

        self.get_logger().info("Nav2 goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # ------------------------------------------------------

    def feedback_callback(self, feedback_msg):

        distance = feedback_msg.feedback.distance_remaining

        self.distance_remaining = float(distance)

        msg = Float32()
        msg.data = self.distance_remaining

        self.distance_pub.publish(msg)

    # ------------------------------------------------------

    def result_callback(self, future):

        result = future.result()
        status = result.status

        self.goal_handle = None
        self.distance_remaining = 0.0

        if status == GoalStatus.STATUS_SUCCEEDED:

            self.robot_state = "ARRIVED"

            if self.current_stage == "PICKUP":

                station = self.current_order["pickup"]

                self.detail = f"Arrived at Pickup Station {station}"

                self.add_history(
                    self.current_order["id"],
                    "PICKUP",
                    f"Package picked up at Station {station}"
                )

                self.get_logger().info(self.detail)

                self.one_shot(2.0, self.navigate_to_drop)

            elif self.current_stage == "DROP":

                zone = self.current_order["drop"]

                self.detail = f"Delivered at Drop Zone {zone}"

                self.add_history(
                    self.current_order["id"],
                    "DELIVERED",
                    f"Package delivered to Zone {zone}"
                )

                self.get_logger().info(self.detail)

                self.one_shot(2.0, self.complete_current_order)

        elif status == GoalStatus.STATUS_CANCELED:

            self.add_history(
                self.current_order["id"] if self.current_order else 0,
                "CANCELLED",
                "Mission cancelled"
            )

            self.current_order = None
            self.robot_state = "IDLE"
            self.current_stage = "IDLE"
            self.detail = "Mission cancelled"

        else:
            self.fail_current(
                f"Navigation failed. Status={status}"
            )

        self.publish_all()

    # ------------------------------------------------------

    def complete_current_order(self):

        if self.current_order is not None:

            self.add_history(
                self.current_order["id"],
                "COMPLETE",
                "Mission completed"
            )

        self.current_order = None
        self.current_stage = "IDLE"
        self.robot_state = "IDLE"
        self.detail = "Ready for next order"
        self.distance_remaining = -1.0

        self.publish_all()

    # ------------------------------------------------------

    def fail_current(self, reason):

        order_id = (
            self.current_order["id"]
            if self.current_order
            else 0
        )

        self.add_history(
            order_id,
            "FAILED",
            reason
        )

        self.get_logger().error(reason)

        self.current_order = None
        self.current_stage = "IDLE"
        self.robot_state = "IDLE"
        self.detail = reason
        self.distance_remaining = -1.0

        self.publish_all()

    # ------------------------------------------------------

    def one_shot(self, seconds, callback):

        holder = {}

        def run_once():
            timer = holder["timer"]
            timer.cancel()
            callback()

        holder["timer"] = self.create_timer(
            seconds,
            run_once
        )

    # ------------------------------------------------------

    def add_history(self, order_id, event, text):

        self.history.insert(0, {
            "time": datetime.now().strftime("%H:%M:%S"),
            "order": order_id,
            "event": event,
            "text": text
        })

        self.history = self.history[:50]

    # ------------------------------------------------------

    def publish_all(self):

        status = {
            "state": self.robot_state,
            "stage": self.current_stage,
            "detail": self.detail,
            "current": self.current_order
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)

        distance_msg = Float32()
        distance_msg.data = float(self.distance_remaining)
        self.distance_pub.publish(distance_msg)

        queue_msg = String()
        queue_msg.data = json.dumps({
            "current": self.current_order,
            "pending": self.pending_orders
        })
        self.queue_pub.publish(queue_msg)

        history_msg = String()
        history_msg.data = json.dumps(self.history)
        self.history_pub.publish(history_msg)


def main(args=None):

    rclpy.init(args=args)

    node = WebWarehouseDispatcher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    