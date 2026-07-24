#!/usr/bin/env python3
# Rotates to the goal direction.
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty


def yaw_to_quat(yaw: float):
    """Return (z, w) for a yaw-only quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# ============================================================
# NODE 1: TF -> Goal Publisher
# ============================================================
class TFGoalPublisher(Node):

    def __init__(self):
        super().__init__('tf_goal_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, '/target_goal', 10)

        # --- snapshot trigger ---
        self.snapshot_pub = self.create_publisher(Empty, '/take_snapshot', 10)
        self.snapshot_threshold_x = -3.0
        self.snapshot_triggered = False
        self.robot_base_frame = 'base_footprint'  # matches AMCL's base_frame_id
        # ------------------------

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.last_goal = None
        self.last_raw_xy = None  # unrounded previous (x, y), used only for yaw calc

        self.get_logger().info("TF Goal Publisher started")

    def check_snapshot_trigger(self):
        if self.snapshot_triggered:
            return

        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                self.robot_base_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF error (snapshot check): {ex}")
            return

        rx = t.transform.translation.x

        self.get_logger().info(f"[DEBUG] current x={rx:.3f}, threshold={self.snapshot_threshold_x}")

        if rx < self.snapshot_threshold_x:
            self.snapshot_triggered = True
            self.get_logger().info(
                f"x={rx:.2f} < {self.snapshot_threshold_x} — triggering snapshot"
            )
            self.snapshot_pub.publish(Empty())

    def timer_callback(self):
        self.check_snapshot_trigger()

        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'target_pose',
                rclpy.time.Time()
            )

            x = t.transform.translation.x
            y = t.transform.translation.y

        except TransformException as ex:
            self.get_logger().warn(f"TF error: {ex}")
            return

        new_goal = (round(x, 2), round(y, 2))
        if new_goal == self.last_goal:
            return

        # Orientation = direction of travel (previous target position ->
        # this one), instead of a fixed yaw. Falls back to identity only
        # for the very first goal, when there's no previous point yet to
        # compute a direction from.
        if self.last_raw_xy is not None:
            dx = x - self.last_raw_xy[0]
            dy = y - self.last_raw_xy[1]
            if math.hypot(dx, dy) > 1e-6:  # avoid atan2(0, 0) on a near-zero move
                yaw = math.atan2(dy, dx)
                qz, qw = yaw_to_quat(yaw)
            else:
                qz, qw = 0.0, 1.0
        else:
            qz, qw = 0.0, 1.0

        self.last_goal = new_goal
        self.last_raw_xy = (x, y)

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.get_logger().info(f"Publishing goal: ({x:.2f}, {y:.2f})")
        self.goal_pub.publish(msg)


# ============================================================
# NODE 2: Nav2 Action Client
# ============================================================
class NavGoalClient(Node):

    def __init__(self):
        super().__init__('nav_goal_client')

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.sub = self.create_subscription(
            PoseStamped,
            '/target_goal',
            self.goal_callback,
            10
        )

        self.current_goal = None

        self.get_logger().info("Nav Goal Client started")

    def goal_callback(self, msg: PoseStamped):

        x = msg.pose.position.x
        y = msg.pose.position.y

        new_goal = (round(x, 2), round(y, 2))
        if new_goal == self.current_goal:
            return

        self.current_goal = new_goal

        self.get_logger().info(f"Received goal: ({x:.2f}, {y:.2f})")
        self.send_goal(msg)

    def send_goal(self, pose: PoseStamped):

        if not self._action_client.server_is_ready():
            self.get_logger().warn("Nav2 not ready")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info("Sending goal to Nav2")

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info("Navigation completed")

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Distance remaining: {fb.distance_remaining:.2f}"
        )


# ============================================================
# MAIN (RUN BOTH NODES)
# ============================================================
def main():
    rclpy.init()

    tf_node = TFGoalPublisher()
    nav_node = NavGoalClient()

    # Use a MultiThreadedExecutor so both nodes run properly
    executor = rclpy.executors.MultiThreadedExecutor()

    executor.add_node(tf_node)
    executor.add_node(nav_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    tf_node.destroy_node()
    nav_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()