#!/usr/bin/env python3
"""
tf_pauser.py — subscribes to /fullrun action feedback and pauses the
target TF publisher via /pause_tf when distance exceeds 2.5 m.

    ros2 run fullrun tf_pauser
"""
import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool
from ar_final_interfaces.action import ArFinal


class TfPauseNode(Node):

    def __init__(self):
        super().__init__('tf_pause_node')

        self.pause_distance = 2.5
        self.resume_distance = 1.0  # hysteresis: must get well back in range before resuming
        self.is_paused = False

        self.pause_client = self.create_client(SetBool, '/pause_tf')

        # Action feedback is published on '<action_name>/_action/feedback'
        # with message type ArFinal.Impl.FeedbackMessage — subscribing
        # directly means we don't need to be the goal sender.
        self.feedback_sub = self.create_subscription(
            ArFinal.Impl.FeedbackMessage,
            '/fullrun/_action/feedback',
            self.feedback_callback,
            10
        )

        self.get_logger().info("TfPauseNode started, watching /fullrun feedback")

    def feedback_callback(self, msg):
        distance = msg.feedback.distance

        if not self.is_paused and distance > self.pause_distance:
            self.get_logger().warn(f"Distance {distance:.2f} > {self.pause_distance} — pausing TF")
            self.call_pause(True)
        elif self.is_paused and distance < self.resume_distance:
            self.get_logger().info(f"Distance {distance:.2f} < {self.resume_distance} — resuming TF")
            self.call_pause(False)

    def call_pause(self, pause: bool):
        if not self.pause_client.service_is_ready():
            self.get_logger().warn("pause_tf service not available")
            return
        req = SetBool.Request()
        req.data = pause
        future = self.pause_client.call_async(req)
        future.add_done_callback(lambda f: self._pause_result(f, pause))

    def _pause_result(self, future, requested_state):
        try:
            resp = future.result()
            if resp.success:
                self.is_paused = requested_state
        except Exception as e:
            self.get_logger().error(f"pause_tf call failed: {e}")


def main():
    rclpy.init()
    node = TfPauseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()