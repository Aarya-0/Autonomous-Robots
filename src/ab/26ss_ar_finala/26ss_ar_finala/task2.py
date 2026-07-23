#!/usr/bin/env python3
"""
follow_target.py — core control loop, run manually for now:
    ros2 run 26ss_ar_final follow_target
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tf2_ros import Buffer, TransformListener, TransformException
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class FollowTargetNode(Node):
    def __init__(self):
        super().__init__('follow_target_node')

        # --- TF setup: this is how we "ask" where things are, on demand ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Publish the required /distance topic ---
        self.distance_pub = self.create_publisher(Float32, '/distance', 10)

        # --- Nav2 action client (persistent — we reuse this, not recreate it) ---
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # --- Service client for the TF publisher's pause/resume ---
        self.pause_client = self.create_client(SetBool, '/pause_tf')

        # --- State ---
        self.last_sent_goal = None      # last (x, y) we sent to Nav2
        self.is_paused = False          # are we currently holding the TF?
        self.goal_move_threshold = 0.3  # meters target must move before we resend a goal

        # Distance thresholds — mapped directly to your grading zones:
        #   green: 0.5-1.5   yellow: 1.5-2.0   abort: >3.0
        # We pause proactively BEFORE hitting abort, and only resume once
        # solidly back in the green zone (hysteresis prevents flapping).
        self.pause_distance = 2.2
        self.resume_distance = 1.0

        cb_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.2, self.control_loop, callback_group=cb_group)

        self.get_logger().info("FollowTargetNode started")

    def control_loop(self):
        # 1. Look up robot's own position (AMCL keeps map->base_link updated)
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            target_tf = self.tf_buffer.lookup_transform(
                'map', 'target_pose', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            return

        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y
        tx = target_tf.transform.translation.x
        ty = target_tf.transform.translation.y

        distance = math.hypot(tx - rx, ty - ry)

        # 2. Publish it — this is graded regardless of anything else working
        msg = Float32()
        msg.data = float(distance)
        self.distance_pub.publish(msg)

        # 3. Decide whether to pause/resume the TF
        self.update_pause_state(distance)

        # 4. Keep Nav2 chasing the target (only resend if it moved enough)
        self.maybe_send_goal(tx, ty)

    def update_pause_state(self, distance):
        if not self.is_paused and distance > self.pause_distance:
            self.get_logger().warn(f"Falling behind (d={distance:.2f}) — pausing TF")
            self.call_pause(True)
        elif self.is_paused and distance < self.resume_distance:
            self.get_logger().info(f"Caught up (d={distance:.2f}) — resuming TF")
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
                self.is_paused = requested_state  # only update state on confirmed success
        except Exception as e:
            self.get_logger().error(f"pause_tf call failed: {e}")

    def maybe_send_goal(self, tx, ty):
        if self.last_sent_goal is not None:
            dx = tx - self.last_sent_goal[0]
            dy = ty - self.last_sent_goal[1]
            if math.hypot(dx, dy) < self.goal_move_threshold:
                return  # target hasn't moved enough to justify replanning

        if not self.nav_client.server_is_ready():
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = tx
        goal.pose.pose.position.y = ty
        goal.pose.pose.orientation.w = 1.0  # facing straight; fine as target moves fast

        self.last_sent_goal = (tx, ty)
        self.get_logger().info(f"Sending new goal: ({tx:.2f}, {ty:.2f})")
        self.nav_client.send_goal_async(goal)


def main():
    rclpy.init()
    node = FollowTargetNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
