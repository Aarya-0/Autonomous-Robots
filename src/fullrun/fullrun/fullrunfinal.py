#!/usr/bin/env python3
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from ar_final_interfaces.action import ArFinal

# Reuse the same hardcoded poses + math helpers as the standalone pose
# setter instead of duplicating them here. Requires the `tf_follower`
# package (which contains pose_setter.py) to be a build/exec dependency
# of whichever package this action server lives in.
from tf_follower.pose_setter import POSES, yaw_from_qz_qw, angle_diff


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

        # --- Initial-pose (AMCL localization) parameters ---
        self.declare_parameter('pose_frame_id', 'map')
        self.declare_parameter('pose_tolerance', 0.3)        # meters
        self.declare_parameter('pose_yaw_tolerance', 0.15)   # radians (~8.6 deg)
        self.declare_parameter('pose_timeout', 15.0)         # seconds
        self.declare_parameter('pose_retry_period', 1.5)     # seconds

        pose_pub_qos = QoSProfile(depth=1)
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', pose_pub_qos)

        pose_sub_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose_cb,
            pose_sub_qos, callback_group=self.cb_group)

        # Only one localization is "active" (being confirmed) at a time.
        # Guarded by _loc_lock so overlapping goals can't stomp on each
        # other's target/confirmation state.
        self._loc_lock = threading.Lock()
        self._active_loc = None  # dict: x, y, target_yaw set while waiting

        self._nav_goal_handle = None   # currently in-flight/accepted Nav2 goal
        self._nav_goal_pending = False  # True while waiting on send_goal_async

        self.get_logger().info("FullRun action server started")

    # ------------------------------------------------------------------
    # AMCL initial-pose handling
    # ------------------------------------------------------------------

    def _amcl_pose_cb(self, msg):
        loc = self._active_loc
        if loc is None:
            return

        dx = msg.pose.pose.position.x - loc['x']
        dy = msg.pose.pose.position.y - loc['y']
        dist = (dx * dx + dy * dy) ** 0.5

        got_yaw = yaw_from_qz_qw(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        dyaw = abs(angle_diff(got_yaw, loc['target_yaw']))

        if dist <= loc['tolerance'] and dyaw <= loc['yaw_tolerance']:
            self.get_logger().info(
                f'AMCL adopted "{loc["task_name"]}" pose '
                f'(offset {dist:.3f} m <= {loc["tolerance"]} m, '
                f'yaw off {math.degrees(dyaw):.1f} deg <= {math.degrees(loc["yaw_tolerance"]):.1f} deg).')
            loc['event'].set()

    def _make_initialpose_msg(self, pose, frame_id):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = pose['x']
        msg.pose.pose.position.y = pose['y']
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = pose['qz']
        msg.pose.pose.orientation.w = pose['qw']
        msg.pose.covariance = pose['cov']
        return msg

    def localize_at(self, task_name, goal_handle):
        """Publish /initialpose repeatedly for the named task's pose until
        AMCL confirms via /amcl_pose (position + yaw), goal is cancelled,
        or pose_timeout elapses. Returns True on confirmation, False
        otherwise. Blocks the calling thread with plain time.sleep, same
        pattern as wait_for_target below, so other callbacks (TF, nav
        client, other goals) keep being serviced on other executor threads.
        """
        if task_name not in POSES:
            valid = ", ".join(sorted(set(POSES.keys())))
            self.get_logger().error(f'Unknown task "{task_name}". Valid values: {valid}')
            return False

        pose = POSES[task_name]
        frame_id = self.get_parameter('pose_frame_id').value
        tolerance = self.get_parameter('pose_tolerance').value
        yaw_tolerance = self.get_parameter('pose_yaw_tolerance').value
        timeout = self.get_parameter('pose_timeout').value
        retry_period = self.get_parameter('pose_retry_period').value

        target_yaw = yaw_from_qz_qw(pose['qz'], pose['qw'])
        event = threading.Event()

        with self._loc_lock:
            self._active_loc = dict(
                task_name=task_name,
                x=pose['x'], y=pose['y'], target_yaw=target_yaw,
                tolerance=tolerance, yaw_tolerance=yaw_tolerance,
                event=event,
            )

        self.get_logger().info(
            f'Localizing at "{task_name}": x={pose["x"]:.3f}, y={pose["y"]:.3f}, '
            f'yaw={math.degrees(target_yaw):.1f} deg (frame="{frame_id}")')

        msg = self._make_initialpose_msg(pose, frame_id)
        start = time.monotonic()
        last_pub = 0.0
        confirmed = False

        try:
            while True:
                if event.is_set():
                    confirmed = True
                    break

                if goal_handle.is_cancel_requested:
                    break

                if time.monotonic() - start > timeout:
                    self.get_logger().error(
                        f'Timed out after {timeout}s without AMCL confirming "{task_name}". '
                        f'Check that AMCL is up and /amcl_pose is publishing.')
                    break

                now = time.monotonic()
                if now - last_pub >= retry_period:
                    msg.header.stamp = self.get_clock().now().to_msg()
                    self._pose_pub.publish(msg)
                    self.get_logger().info(
                        f'Published /initialpose for "{task_name}", waiting for AMCL confirmation...')
                    last_pub = now

                time.sleep(0.1)
        finally:
            with self._loc_lock:
                self._active_loc = None

        return confirmed

    # ------------------------------------------------------------------
    # Nav / TF following (unchanged from before)
    # ------------------------------------------------------------------

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

        if task not in POSES:
            valid = ", ".join(sorted(set(POSES.keys())))
            goal_handle.abort()
            result.message = f'Failed: unknown task "{task}". Valid values: {valid}'
            return result

        # Localize at the pose for this task before doing anything else.
        localized = self.localize_at(task, goal_handle)
        if not localized:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message = f'Cancelled while localizing for task "{task}"'
            else:
                goal_handle.abort()
                result.message = f'Failed: could not confirm AMCL localization for task "{task}"'
            return result

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