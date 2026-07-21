#!/usr/bin/env python3
import math

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

        self.get_logger().info("FullRun action server started")

    def lookup_xy(self, target_frame, source_frame='map'):
        try:
            t = self.tf_buffer.lookup_transform(
                source_frame, target_frame, rclpy.time.Time()
            )
            return t.transform.translation.x, t.transform.translation.y
        except TransformException as ex:
            self.get_logger().warn(f"TF error ({target_frame}): {ex}")
            return None

    def execute_callback(self, goal_handle):
        task = goal_handle.request.task
        self.get_logger().info(f"Received task: {task}")

        result = ArFinal.Result()
        feedback_msg = ArFinal.Feedback()

        target = self.lookup_xy('target_pose')
        if target is None:
            goal_handle.abort()
            result.message = "Failed: could not resolve target_pose TF"
            return result

        tx, ty = target

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            result.message = "Failed: Nav2 action server not available"
            return result

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = tx
        goal_msg.pose.pose.position.y = ty
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        send_future = self._nav_client.send_goal_async(goal_msg)

        while not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        nav_goal_handle = send_future.result()
        if not nav_goal_handle.accepted:
            goal_handle.abort()
            result.message = "Failed: Nav2 rejected the goal"
            return result

        result_future = nav_goal_handle.get_result_async()

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                nav_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                result.message = "Cancelled by client"
                return result

            robot_xy = self.lookup_xy('base_link')
            if robot_xy is not None:
                rx, ry = robot_xy
                feedback_msg.distance = math.hypot(tx - rx, ty - ry)
                goal_handle.publish_feedback(feedback_msg)

            rclpy.spin_once(self, timeout_sec=0.1)

        nav_result = result_future.result()

        goal_handle.succeed()
        result.message = f"Navigation finished with status {nav_result.status}"
        return result


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