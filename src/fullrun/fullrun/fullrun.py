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
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.srv import SetEntityState
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ar_final_interfaces.action import ArFinal

# Reuse the same hardcoded AMCL poses + math helpers as the standalone pose
# setter instead of duplicating them here.
from tf_follower.pose_setter import POSES, yaw_from_qz_qw, angle_diff

# Ground-truth spawn poses in the Gazebo WORLD frame (not the map frame).
# These are where the robot gets teleported before AMCL is told where it is.
# yaw is in radians; converted to quaternion (z, w) at teleport time.
SPAWN_POSES = {
    "task1":   dict(x=1.0,  y=2.0,  yaw=3.11),
    "task2":   dict(x=-3.0, y=1.0,  yaw=-1.5),
    "task3":   dict(x=1.0,  y=-4.0, yaw=-1.5),
    "task4":   dict(x=-1.5, y=-5.0, yaw=1.5),
}
SPAWN_POSES["fullrun"] = SPAWN_POSES["task1"]


def quat_from_yaw(yaw):
    """Quaternion (z, w) for a rotation of `yaw` radians about Z only."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


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

        # --- AMCL localization parameters ---
        self.declare_parameter('pose_frame_id', 'map')
        self.declare_parameter('pose_tolerance', 0.3)        # meters
        self.declare_parameter('pose_yaw_tolerance', 0.15)   # radians (~8.6 deg)
        self.declare_parameter('pose_timeout', 15.0)         # seconds
        self.declare_parameter('pose_retry_period', 1.5)     # seconds

        # --- Gazebo teleport parameters ---
        # NOTE: this MUST match the entity/model name your launch file gives
        # the robot at spawn time (check the `-entity` arg or `name:` field
        # passed to the spawn call in sim_ar_ss26.launch.py). If this is
        # wrong, the service call succeeds but silently does nothing.
        self.declare_parameter('robot_entity_name', 'tiago')
        self.declare_parameter('teleport_timeout', 10.0)     # seconds
        self.declare_parameter('teleport_z', 0.0)             # meters

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

        self._teleport_client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state',
            callback_group=self.cb_group)

        # Only one localization is "active" (being confirmed) at a time.
        self._loc_lock = threading.Lock()
        self._active_loc = None

        self._nav_goal_handle = None
        self._nav_goal_pending = False

        self.get_logger().info("FullRun action server started")

    # ------------------------------------------------------------------
    # Gazebo teleport (ground-truth spawn pose)
    # ------------------------------------------------------------------

    def teleport_to(self, task_name, goal_handle):
        """Teleport the robot to the task's world-frame spawn pose via
        /gazebo/set_entity_state. Blocks (via polling, not spin) until the
        service responds, is cancelled, or teleport_timeout elapses."""
        if task_name not in SPAWN_POSES:
            valid = ", ".join(sorted(set(SPAWN_POSES.keys())))
            self.get_logger().error(f'Unknown task "{task_name}". Valid values: {valid}')
            return False

        spawn = SPAWN_POSES[task_name]
        entity_name = self.get_parameter('robot_entity_name').value
        timeout = self.get_parameter('teleport_timeout').value
        z = self.get_parameter('teleport_z').value

        if not self._teleport_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                '/gazebo/set_entity_state service not available — is Gazebo Classic '
                'running with gazebo_ros loaded?')
            return False

        qz, qw = quat_from_yaw(spawn['yaw'])
        req = SetEntityState.Request()
        req.state.name = entity_name
        req.state.reference_frame = 'world'
        req.state.pose.position.x = spawn['x']
        req.state.pose.position.y = spawn['y']
        req.state.pose.position.z = z
        req.state.pose.orientation.x = 0.0
        req.state.pose.orientation.y = 0.0
        req.state.pose.orientation.z = qz
        req.state.pose.orientation.w = qw
        # Zero velocity so the robot doesn't carry momentum from wherever
        # it was before the teleport.
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0

        self.get_logger().info(
            f'Teleporting "{entity_name}" to "{task_name}" spawn pose: '
            f'x={spawn["x"]:.3f}, y={spawn["y"]:.3f}, yaw={math.degrees(spawn["yaw"]):.1f} deg')

        future = self._teleport_client.call_async(req)
        start = time.monotonic()
        while not future.done():
            if goal_handle.is_cancel_requested:
                return False
            if time.monotonic() - start > timeout:
                self.get_logger().error(
                    f'Timed out after {timeout}s waiting for /gazebo/set_entity_state response.')
                return False
            time.sleep(0.05)

        resp = future.result()
        if resp is None or not getattr(resp, 'success', True):
            self.get_logger().error(
                f'Teleport call for "{task_name}" failed or returned success=False. '
                f'Double-check robot_entity_name ("{entity_name}") matches the spawned model.')
            return False

        self.get_logger().info(f'Teleport to "{task_name}" confirmed.')
        return True

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
    # Nav / TF following (unchanged)
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

        if task not in POSES or task not in SPAWN_POSES:
            valid = ", ".join(sorted(set(POSES.keys()) & set(SPAWN_POSES.keys())))
            goal_handle.abort()
            result.message = f'Failed: unknown task "{task}". Valid values: {valid}'
            return result

        # 1) Teleport the robot to this task's ground-truth spawn pose.
        teleported = self.teleport_to(task, goal_handle)
        if not teleported:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message = f'Cancelled while teleporting for task "{task}"'
            else:
                goal_handle.abort()
                result.message = f'Failed: could not teleport robot for task "{task}"'
            return result

        # 2) Tell AMCL where that pose is and wait for it to confirm.
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


# ============================================================
# HeadSweepNode (incorporated from second script, unchanged)
# ============================================================
# class HeadSweepNode(Node):
#     """
#     Periodically tilts the head down to scan the near-field ground
#     blind spot, then returns to neutral. Every few cycles it also
#     pans sideways to widen horizontal coverage (doorways, corners).

#     Joint order for TIAGo's head_controller is [head_1_joint (pan),
#     head_2_joint (tilt)]. Pan: + is left, - is right (radians).
#     Tilt: negative looks down, 0.0 is level, positive looks up.

#     Tuned to match a 3.0s costmap clear / 3.0s observation_persistence:
#     one full dip completes well within that window so a fresh ground
#     observation is always in the buffer when the clear runs.
#     """

#     NEUTRAL = (0.0, 0.0)
#     TILT_DOWN = (0.0, -0.5)     # look down at near-field ground
#     PAN_LEFT_DOWN = (0.4, -0.4)  # widen coverage to the left
#     PAN_RIGHT_DOWN = (-0.4, -0.4)  # widen coverage to the right

#     def __init__(self):
#         super().__init__('head_sweep_node')

#         self._action_client = ActionClient(
#             self,
#             FollowJointTrajectory,
#             '/head_controller/follow_joint_trajectory'
#         )

#         self.joint_names = ['head_1_joint', 'head_2_joint']

#         # One full sweep cycle: down -> neutral -> (occasionally) side pans -> neutral
#         self.sweep_period_sec = 2.5   # < 3.0s clear interval, leaves margin
#         self.segment_duration_sec = 0.8

#         self.cycle_count = 0
#         self.side_pan_every_n = 3  # do a sideways pan every Nth cycle

#         self.timer = self.create_timer(self.sweep_period_sec, self.sweep_callback)

#         self.get_logger().info("Head Sweep Node started")

#     def sweep_callback(self):
#         if not self._action_client.server_is_ready():
#             self.get_logger().warn("Head controller action server not ready")
#             return

#         self.cycle_count += 1
#         do_side_pan = (self.cycle_count % self.side_pan_every_n == 0)

#         if do_side_pan:
#             waypoints = [
#                 self.TILT_DOWN,
#                 self.PAN_LEFT_DOWN,
#                 self.PAN_RIGHT_DOWN,
#                 self.NEUTRAL,
#             ]
#             self.get_logger().info("Head sweep: down + side pan (left/right)")
#         else:
#             waypoints = [
#                 self.TILT_DOWN,
#                 self.NEUTRAL,
#             ]
#             self.get_logger().info("Head sweep: down-tilt only")

#         self.send_trajectory(waypoints)

#     def send_trajectory(self, waypoints):
#         goal_msg = FollowJointTrajectory.Goal()
#         goal_msg.trajectory.joint_names = self.joint_names

#         points = []
#         t = self.segment_duration_sec
#         for pan, tilt in waypoints:
#             pt = JointTrajectoryPoint()
#             pt.positions = [pan, tilt]
#             pt.velocities = [0.0, 0.0]
#             pt.time_from_start = Duration(seconds=t).to_msg()
#             points.append(pt)
#             t += self.segment_duration_sec

#         goal_msg.trajectory.points = points

#         future = self._action_client.send_goal_async(goal_msg)
#         future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().warn("Head sweep goal rejected")
#             return
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.result_callback)

#     def result_callback(self, future):
#         self.get_logger().info("Head sweep segment completed")


def main():
    rclpy.init()
    node = FullRunActionServer()
    # head_node = HeadSweepNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    # executor.add_node(head_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    # head_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()