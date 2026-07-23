#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tf2_ros import Buffer, TransformListener, TransformException

from nav2_msgs.action import NavigateToPose
from ar_final_interfaces.action import ArFinal


class FullRunActionServer(Node):

    def __init__(self):
        super().__init__('fullrun_action_server')

        self.cb_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.cb_group
        )

        self._action_server = ActionServer(
            self,
            ArFinal,
            'fullrun',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group
        )

        self.declare_parameter('target_wait_timeout_sec', 300.0)
        self.declare_parameter('follow_rate_hz', 10.0)

        self._nav_goal_handle = None   # currently in-flight/accepted Nav2 goal
        self._nav_goal_pending = False  # True while waiting on send_goal_async

        self.get_logger().info("FullRun action server started")

    def lookup_xy(self, target_frame, source_frame='map'):
        try:
            t = self.tf_buffer.lookup_transform(
                source_frame, target_frame, rclpy.time.Time()
            )
            return t.transform.translation.x, t.transform.translation.y
        except TransformException:
            return None

    def wait_for_target(self, goal_handle, target_frame='target_pose', source_frame='map'):
        """Blocking wait, but via plain time.sleep — no nested spin_once.
        Runs on this callback's own executor thread; other threads keep
        servicing TF, nav client callbacks, etc."""
        timeout = self.get_parameter('target_wait_timeout_sec').value
        start = time.monotonic()

        self.get_logger().info(f"Waiting for TF '{target_frame}' to appear...")

        while True:
            xy = self.lookup_xy(target_frame, source_frame)
            if xy is not None:
                self.get_logger().info(f"Target TF found: {xy}")
                return xy

            if goal_handle.is_cancel_requested:
                return None

            if time.monotonic() - start > timeout:
                self.get_logger().warn(
                    f"Timed out after {timeout:.1f}s waiting for TF '{target_frame}'"
                )
                return None

            time.sleep(0.2)

    def _nav_goal_response_callback(self, future):
        handle = future.result()
        if handle.accepted:
            self._nav_goal_handle = handle
            self.get_logger().info("Nav goal accepted")
        else:
            self.get_logger().warn("Nav2 rejected goal")
        self._nav_goal_pending = False

    def send_nav_goal(self, x, y):
        """Fire-and-forget, like the original TFGoalPublisher -> NavGoalClient
        hand-off: no blocking wait on acceptance, just an async callback."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self._nav_goal_pending = True
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._nav_goal_response_callback)
        self.get_logger().info(f"Sent new nav goal: ({x:.2f}, {y:.2f})")

    def execute_callback(self, goal_handle):
        task = goal_handle.request.task
        self.get_logger().info(f"Received task: {task}")

        result = ArFinal.Result()
        feedback_msg = ArFinal.Feedback()

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            result.message = "Failed: Nav2 action server not available"
            return result

        target = self.wait_for_target(goal_handle)
        if target is None:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message = "Cancelled while waiting for target_pose TF"
            else:
                goal_handle.abort()
                result.message = "Failed: target_pose TF never appeared (timeout)"
            return result

        rate_hz = self.get_parameter('follow_rate_hz').value
        period = 1.0 / rate_hz
        last_goal = None

        self.get_logger().info("Entering follow loop")

        while True:
            if goal_handle.is_cancel_requested:
                if self._nav_goal_handle is not None:
                    self._nav_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                result.message = "Cancelled by client"
                return result

            xy = self.lookup_xy('target_pose')
            if xy is not None:
                x, y = xy
                new_goal = (round(x, 2), round(y, 2))

                if new_goal != last_goal and not self._nav_goal_pending:
                    last_goal = new_goal
                    self.send_nav_goal(x, y)

            robot_xy = self.lookup_xy('base_link')
            if robot_xy is not None and last_goal is not None:
                rx, ry = robot_xy
                feedback_msg.distance = math.hypot(last_goal[0] - rx, last_goal[1] - ry)
                goal_handle.publish_feedback(feedback_msg)

            time.sleep(period)


def main():
    rclpy.init()
    node = FullRunActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()