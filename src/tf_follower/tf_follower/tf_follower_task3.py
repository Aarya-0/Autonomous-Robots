#!/usr/bin/env python3
"""
Follow a moving TF target using ONLY the local costmap for obstacle avoidance.

Design decisions (each one fixes a specific failure mode found during debugging):
  - AMCL is stopped before navigation starts by PAUSING its lifecycle
    manager (/lifecycle_manager_localization), so map->odom is never
    touched and can never destabilize / snap. Note: the manager is NOT
    itself a lifecycle node -- `ros2 lifecycle get/set` don't work on it.
    It must be paused via its manage_nodes service
    (nav2_msgs/srv/ManageLifecycleNodes, command=PAUSE). Calling
    change_state on AMCL directly without pausing its manager first does
    not stick -- the manager treats that as a fault and cycles AMCL right
    back to active, which is what caused map->odom to keep re-publishing.
  - Goals are sent via /follow_path (DWB), NOT /navigate_to_pose. This avoids
    bt_navigator, planner_server, and any dependency on the `map` frame or
    global costmap sizing.
  - All poses are expressed in `odom` frame, matching local_costmap's
    global_frame, so no map->odom transform is ever required.
  - PoseStamped headers use zero/unset stamps, so tf2 resolves against the
    latest available transform instead of a specific point in time. This
    avoids the wall-clock-vs-sim-clock extrapolation bug.
  - The path sent to FollowPath includes the robot's *current* pose as the
    first waypoint, with orientation computed from direction-of-travel
    (not identity/w=1.0), which is what let DWB actually translate instead
    of endlessly rotating to face an unrelated fixed heading.
  - Goal updates are distance-gated (0.5 m by default) instead of firing on
    every 1 Hz TF tick, so FollowPath isn't preempted every second.
  - IMPORTANT CAVEAT: the TF target (target_pose) this node tracks is
    published by an external, unmodifiable script under `map` as its
    parent frame. That means a `map -> odom` link is still required for
    `odom -> target_pose` lookups to resolve at all -- pausing AMCL alone
    breaks that link entirely (TF tree splits into two disconnected
    islands: odom/base_link on one side, map/target_pose on the other).
    The fix: capture map->odom ONCE while AMCL is still active/converged,
    then re-broadcast it as a STATIC transform before pausing AMCL. This
    keeps the tree connected with a fixed, known-good offset, without any
    of AMCL's live corrections (which is what was destabilizing/snapping
    map->odom in the first place).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from nav2_msgs.srv import ManageLifecycleNodes

from tf2_ros import Buffer, TransformListener, TransformException, StaticTransformBroadcaster

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


def yaw_to_quat(yaw: float):
    """Return (z, w) for a yaw-only quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class LifecycleManagerPauser(Node):
    """
    One-shot helper: calls <manager_name>/manage_nodes with PAUSE.

    IMPORTANT: Nav2's lifecycle_manager nodes are NOT lifecycle nodes
    themselves -- `ros2 lifecycle get/set` does not work on them (it will
    report "Node not found"). They expose a separate service,
    manage_nodes (nav2_msgs/srv/ManageLifecycleNodes), which is the only
    way to actually stop them from managing/reconciling the nodes they
    own. Calling change_state directly on AMCL without pausing its
    manager first will not stick -- the manager treats that as a fault
    and cycles AMCL right back through its bringup sequence, which is
    what was causing map->odom to keep re-publishing.

    PAUSE (command id 1) transitions all managed nodes (e.g. amcl,
    map_server) to inactive AND stops the manager from reconciling them
    back to active.
    """

    PAUSE = 1

    def __init__(self, manager_name: str):
        super().__init__(f'{manager_name.strip("/").replace("/", "_")}_pauser')
        self.manager_name = manager_name
        self.client = self.create_client(
            ManageLifecycleNodes, f'{manager_name}/manage_nodes'
        )

    def pause(self, timeout_sec: float = 5.0) -> bool:
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f"Could not reach {self.manager_name}/manage_nodes — manager not "
                f"running under this name? Skipping."
            )
            return False

        req = ManageLifecycleNodes.Request()
        req.command = self.PAUSE

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f"{self.manager_name} paused successfully.")
            return True

        self.get_logger().warn(
            f"{self.manager_name} pause call did not report success."
        )
        return False


class MapOdomFreezer(Node):
    """
    One-shot helper: captures the current map->odom transform (while AMCL
    is still active and hopefully converged) and re-broadcasts it as a
    STATIC transform. This keeps `map` and `odom` connected in the TF
    tree -- required for looking up any `map`-frame-published topic (like
    target_pose from the fixed tf_publisher.py script) in `odom` -- but
    freezes it at a single known value instead of letting AMCL keep
    correcting it live, which is what caused the snapping/rotation issues.

    Call capture_and_freeze() BEFORE pausing AMCL's lifecycle manager.
    Keep a reference to the returned node alive for the lifetime of the
    program (destroying it stops the static broadcast).
    """

    def __init__(self):
        super().__init__('map_odom_freezer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

    def capture_and_freeze(self, timeout_sec: float = 10.0) -> bool:
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)

        while self.get_clock().now().nanoseconds < deadline:
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', 'odom', rclpy.time.Time()
                )
                self.static_broadcaster.sendTransform(t)
                self.get_logger().info(
                    f"Froze map->odom at translation="
                    f"({t.transform.translation.x:.3f}, "
                    f"{t.transform.translation.y:.3f}), "
                    f"rotation.z={t.transform.rotation.z:.3f}, "
                    f"rotation.w={t.transform.rotation.w:.3f}"
                )
                return True
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().error(
            "Could not capture map->odom before timeout -- AMCL may not be "
            "publishing yet, or may not be converged. target_pose lookups "
            "in odom will fail until this succeeds."
        )
        return False


class TFFollowerLocalOnly(Node):
    """
    Watches a TF frame (default: target_pose, in odom) and drives the robot
    toward it using FollowPath / DWB obstacle avoidance only.
    """

    def __init__(self,
                 target_frame: str = 'target_pose',
                 reference_frame: str = 'odom',
                 robot_frame: str = 'base_link',
                 controller_id: str = 'FollowPath',
                 min_regoal_distance: float = 0.1,
                 update_period_sec: float = 0.3):
        super().__init__('tf_follower_local_only')

        self.target_frame = target_frame
        self.reference_frame = reference_frame
        self.robot_frame = robot_frame
        self.controller_id = controller_id
        self.min_regoal_distance = min_regoal_distance

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._action_client = ActionClient(self, FollowPath, '/follow_path')

        self.last_goal_xy = None
        self.goal_in_flight = False
        self.current_goal_handle = None
        self.cancel_in_flight = False
        self._goal_epoch = 0  # incremented on every new send; guards stale callbacks

        self.timer = self.create_timer(update_period_sec, self.timer_callback)

        self.get_logger().info(
            f"TFFollowerLocalOnly started. Tracking '{target_frame}' in "
            f"'{reference_frame}' frame, sending FollowPath goals."
        )

    # -----------------------------------------------------------------
    def get_robot_pose(self):
        """Return (x, y) of the robot in reference_frame, or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.robot_frame,
                Time()  # latest available
            )
            return t.transform.translation.x, t.transform.translation.y
        except TransformException as ex:
            self.get_logger().warn(f"Robot pose TF lookup failed: {ex}")
            return None

    def get_target_pose(self):
        """Return (x, y) of the TF target in reference_frame, or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.target_frame,
                Time()  # latest available
            )
            return t.transform.translation.x, t.transform.translation.y
        except TransformException as ex:
            self.get_logger().warn(f"Target TF lookup failed: {ex}")
            return None

    # -----------------------------------------------------------------
    def timer_callback(self):
        if self.cancel_in_flight:
            # A cancel request is already being processed; wait for it to
            # land before sending another goal, to avoid overlapping
            # cancel/send races.
            return

        robot_xy = self.get_robot_pose()
        target_xy = self.get_target_pose()
        if robot_xy is None or target_xy is None:
            return

        if self.last_goal_xy is not None:
            dx = target_xy[0] - self.last_goal_xy[0]
            dy = target_xy[1] - self.last_goal_xy[1]
            if math.hypot(dx, dy) < self.min_regoal_distance:
                return  # target hasn't moved enough to bother re-goaling

        self.last_goal_xy = target_xy

        if self.goal_in_flight and self.current_goal_handle is not None:
            # Preempt: cancel the stale goal, then send the fresh one once
            # the cancel is acknowledged. Without this, a goal that's
            # stuck (e.g. rotating at a corner) never gets updated, so by
            # the time it eventually resolves the target has moved far
            # away and the next path requires a drastic turn.
            self.cancel_in_flight = True
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda fut, r=robot_xy, g=target_xy: self._on_cancel_done(fut, r, g)
            )
        else:
            self.send_path(robot_xy, target_xy)

    def _on_cancel_done(self, future, robot_xy, target_xy):
        self.cancel_in_flight = False
        self.goal_in_flight = False
        self.current_goal_handle = None
        # Re-fetch the freshest robot pose before sending, since some time
        # has passed while the cancel was processed.
        fresh_robot_xy = self.get_robot_pose() or robot_xy
        self.send_path(fresh_robot_xy, target_xy)

    # -----------------------------------------------------------------
    def build_path(self, start_xy, goal_xy) -> Path:
        yaw = math.atan2(goal_xy[1] - start_xy[1], goal_xy[0] - start_xy[0])
        qz, qw = yaw_to_quat(yaw)

        path = Path()
        path.header.frame_id = self.reference_frame
        # Leave stamp at zero (default) -> tf2 uses latest available transform,
        # avoiding wall-clock vs sim-clock extrapolation errors.

        for (x, y) in (start_xy, goal_xy):
            pose = PoseStamped()
            pose.header.frame_id = self.reference_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

        return path

    def send_path(self, start_xy, goal_xy):
        if not self._action_client.server_is_ready():
            self.get_logger().warn("FollowPath action server not ready.")
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = self.build_path(start_xy, goal_xy)
        goal_msg.controller_id = self.controller_id

        self.get_logger().info(
            f"Sending FollowPath goal: ({goal_xy[0]:.2f}, {goal_xy[1]:.2f})"
        )

        self._goal_epoch += 1
        my_epoch = self._goal_epoch

        self.goal_in_flight = True
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(
            lambda fut, e=my_epoch: self.goal_response_callback(fut, e)
        )

    def goal_response_callback(self, future, epoch):
        if epoch != self._goal_epoch:
            return  # a newer goal has already superseded this one
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("FollowPath goal rejected.")
            self.goal_in_flight = False
            self.current_goal_handle = None
            return

        self.get_logger().info("FollowPath goal accepted.")
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, e=epoch: self.result_callback(fut, e)
        )

    def result_callback(self, future, epoch):
        if epoch != self._goal_epoch:
            return  # stale result from a goal we already preempted
        self.goal_in_flight = False
        self.current_goal_handle = None
        try:
            status = future.result().status
            self.get_logger().info(f"FollowPath finished with status: {status}")
        except Exception as ex:
            self.get_logger().warn(f"FollowPath result error: {ex}")

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        # FollowPath feedback fields: distance_to_goal, speed
        self.get_logger().info(
            f"distance_to_goal={fb.distance_to_goal:.2f} speed={fb.speed:.2f}"
        )


def main():
    rclpy.init()

    # Step 1: capture map->odom and freeze it as a STATIC transform, while
    # AMCL is still active. This MUST happen before pausing AMCL: the
    # target TF this node tracks (target_pose) is published under `map`
    # by a script we cannot modify, so odom->target_pose lookups require
    # a map->odom link to exist at all times, even after AMCL stops.
    freezer = MapOdomFreezer()
    froze_ok = freezer.capture_and_freeze(timeout_sec=10.0)
    if not froze_ok:
        freezer.get_logger().error(
            "Proceeding without a frozen map->odom transform -- "
            "target_pose lookups will likely fail."
        )
    # NOTE: freezer is intentionally NOT destroyed here. StaticTransform-
    # Broadcaster publishes on a transient-local/latched topic, but the
    # node itself must stay alive for the process's lifetime for that
    # publisher to keep serving late-joining subscribers reliably.

    # Step 2: PAUSE the localization lifecycle manager. This is the correct
    # mechanism -- the manager is not itself a lifecycle node, so
    # change_state does not work on it (returns "Node not found" under
    # `ros2 lifecycle get`). manage_nodes/PAUSE transitions AMCL (and any
    # other node it owns) to inactive AND stops the manager from
    # reconciling it back to active, which is what was causing map->odom
    # to keep re-publishing and the map to jump/rotate in RViz.
    manager = LifecycleManagerPauser('/lifecycle_manager_localization')
    manager.pause()
    manager.destroy_node()

    # Step 3: start following the TF target using local costmap only.
    follower = TFFollowerLocalOnly()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(freezer)
    executor.add_node(follower)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    freezer.destroy_node()
    follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()