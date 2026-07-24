#!/usr/bin/env python3
"""
Mission architecture:
  1. Go out of the mapped area, following target_pose via FollowPath /
     the local costmap only (AMCL paused, map->odom frozen -- see the
     design notes below, unchanged from the previous version).
  2. Continuously watch the LIDAR scan for a doorway pattern: an opening
     flanked by close wall returns on both sides. Detection is ARMED only
     after the robot has traveled min_arm_distance_m from the start, so
     it does not fire immediately on the SAME doorway while still
     leaving (that would prematurely resume AMCL right as we're heading
     out, defeating the point of pausing it).
  3. Once a gap is confirmed: STOP following the TF (cancel any in-flight
     FollowPath goal, pause the follower's timer), and drive straight
     through the gap using raw /cmd_vel control (DoorwayGapCrosser),
     ignoring target_pose entirely during the crossing.
  4. Once through: seed /initialpose with the current TF-estimated pose
     (still valid at this instant, since AMCL is still paused) and RESUME
     the lifecycle manager -- AMCL reconverges starting from a sane
     hypothesis instead of its stale pre-pause belief.
  5. Resume TF-following. From here on, odom->target_pose lookups use
     AMCL's live, freshly-reconverged map->odom instead of the frozen
     static one (tf2 automatically prefers dynamic /tf data over the
     static fallback once it exists again -- no manual cleanup needed).

Design decisions carried over from the previous version (each fixes a
specific failure mode found during debugging):
  - AMCL is stopped before navigation starts by PAUSING its lifecycle
    manager (/lifecycle_manager_localization), so map->odom is never
    touched and can never destabilize / snap. The manager is NOT itself
    a lifecycle node -- `ros2 lifecycle get/set` don't work on it. It
    must be paused/resumed via its manage_nodes service
    (nav2_msgs/srv/ManageLifecycleNodes).
  - Goals are sent via /follow_path (DWB), NOT /navigate_to_pose. This
    avoids bt_navigator, planner_server, and any dependency on the `map`
    frame or global costmap sizing.
  - All FollowPath poses are expressed in `odom` frame, matching
    local_costmap's global_frame, so no map->odom transform is required
    for ordinary navigation.
  - PoseStamped headers use zero/unset stamps, so tf2 resolves against
    the latest available transform instead of a specific point in time,
    avoiding the wall-clock-vs-sim-clock extrapolation bug.
  - FollowPath goals include the robot's *current* pose as the first
    waypoint, with orientation computed from direction-of-travel (not
    identity/w=1.0), which is what let DWB actually translate instead of
    endlessly rotating to face an unrelated fixed heading.
  - Goal updates are distance-gated and PREEMPT (cancel + resend) an
    in-flight goal rather than waiting for it, so a stuck goal doesn't
    freeze the path against a target that has since moved on.
  - IMPORTANT CAVEAT: target_pose is published by an external,
    unmodifiable script under `map` as its parent frame. That means a
    `map -> odom` link is required for `odom -> target_pose` lookups to
    resolve at all -- pausing AMCL alone breaks that link entirely. Fix:
    capture map->odom ONCE while AMCL is active/converged and
    re-broadcast it as a STATIC transform before pausing.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from nav2_msgs.srv import ManageLifecycleNodes

from tf2_ros import Buffer, TransformListener, TransformException, StaticTransformBroadcaster

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import FollowPath
from sensor_msgs.msg import LaserScan


def yaw_to_quat(yaw: float):
    """Return (z, w) for a yaw-only quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class LifecycleManagerCommander(Node):
    """
    One-shot helper: calls <manager_name>/manage_nodes with PAUSE/RESUME.

    IMPORTANT: Nav2's lifecycle_manager nodes are NOT lifecycle nodes
    themselves -- `ros2 lifecycle get/set` does not work on them. They
    expose a separate service, manage_nodes
    (nav2_msgs/srv/ManageLifecycleNodes), which is the only way to
    actually stop/start them managing the nodes they own. Calling
    change_state directly on AMCL without going through its manager does
    not stick -- the manager treats that as a fault and cycles AMCL
    right back to its previous state.
    """

    PAUSE = 1
    RESUME = 2

    def __init__(self, manager_name: str):
        super().__init__(f'{manager_name.strip("/").replace("/", "_")}_commander')
        self.manager_name = manager_name
        self.client = self.create_client(
            ManageLifecycleNodes, f'{manager_name}/manage_nodes'
        )

    def _send_command(self, command: int, label: str, timeout_sec: float = 5.0) -> bool:
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f"Could not reach {self.manager_name}/manage_nodes — manager not "
                f"running under this name? Skipping {label}."
            )
            return False

        req = ManageLifecycleNodes.Request()
        req.command = command

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f"{self.manager_name} {label} succeeded.")
            return True

        self.get_logger().warn(
            f"{self.manager_name} {label} call did not report success."
        )
        return False

    def pause(self, timeout_sec: float = 5.0) -> bool:
        return self._send_command(self.PAUSE, "pause", timeout_sec)

    def resume(self, timeout_sec: float = 5.0) -> bool:
        return self._send_command(self.RESUME, "resume", timeout_sec)


class MapOdomFreezer(Node):
    """
    One-shot helper: captures the current map->odom transform (while AMCL
    is still active and hopefully converged) and re-broadcasts it as a
    STATIC transform. This keeps `map` and `odom` connected in the TF
    tree -- required for looking up any `map`-frame-published topic (like
    target_pose from the fixed tf_publisher.py script) in `odom` -- but
    freezes it at a single known value instead of letting AMCL keep
    correcting it live.

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

    Can be paused externally (e.g. by MissionSupervisor while the
    DoorwayGapCrosser has taken over /cmd_vel) via pause()/resume(). While
    paused, timer_callback does nothing and any in-flight goal is
    canceled so it doesn't keep publishing through FollowPath's own
    controller_server output while raw cmd_vel control is active
    elsewhere.
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

        self.paused = False

        self.timer = self.create_timer(update_period_sec, self.timer_callback)

        self.get_logger().info(
            f"TFFollowerLocalOnly started. Tracking '{target_frame}' in "
            f"'{reference_frame}' frame, sending FollowPath goals."
        )

    # -----------------------------------------------------------------
    def pause(self):
        """Stop sending/following goals; cancel anything in flight."""
        if self.paused:
            return
        self.paused = True
        self.get_logger().info("TFFollowerLocalOnly paused.")
        if self.goal_in_flight and self.current_goal_handle is not None:
            self.cancel_in_flight = True
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_pause_cancel_done)

    def _on_pause_cancel_done(self, future):
        self.cancel_in_flight = False
        self.goal_in_flight = False
        self.current_goal_handle = None

    def resume(self):
        """Resume normal TF-following operation."""
        if not self.paused:
            return
        self.paused = False
        self.last_goal_xy = None  # force a fresh goal on the next tick
        self.get_logger().info("TFFollowerLocalOnly resumed.")

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
        if self.paused:
            return

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
        if self.paused:
            return  # got paused while the cancel was in flight; don't resend
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


class DoorwayGapCrosser(Node):
    """
    Detects a doorway-like gap in the LIDAR scan (an opening flanked by
    close wall returns on both sides) and drives the robot straight
    through it via raw /cmd_vel -- no map, AMCL, or Nav2 involved, so
    "we passed through" is a real physical event, not a position
    estimate.

    Detection only runs while `armed` is True (see MissionSupervisor --
    gated on distance traveled from the start, so this does not fire on
    the same doorway while still leaving the mapped area).

    State machine: SEARCH -> ALIGN -> CROSS -> DONE
      SEARCH: passively watch scans; no cmd_vel published (so it doesn't
              fight with FollowPath's own /cmd_vel output during normal
              TF-following operation). Requires several consecutive
              matching scans before committing, to avoid transient noise.
      ALIGN:  rotate in place until heading points at the gap center.
      CROSS:  drive straight through at a fixed speed, tracking distance
              via odometry. One safety check: if the forward-center beam
              suddenly reads closer than stop_distance, halt.
      DONE:   stop, report crossed via on_crossed (called exactly once).
    """

    def __init__(self,
                 scan_topic: str = '/scan_raw',
                 odom_topic: str = '/mobile_base_controller/odom',
                 cmd_vel_topic: str = '/cmd_vel',
                 forward_cone_deg: float = 120.0,
                 wall_range_max: float = 2.0,
                 gap_range_min: float = 3.0,
                 min_gap_width_deg: float = 8.0,
                 required_consecutive_detections: int = 10,
                 align_tolerance_rad: float = 0.05,
                 cross_speed: float = 0.3,
                 align_ang_gain: float = 1.0,
                 cross_ang_gain: float = 0.6,
                 crossing_distance_m: float = 1.8,
                 stop_distance_m: float = 0.35,
                 on_gap_confirmed=None,
                 on_crossed=None):
        super().__init__('doorway_gap_crosser')

        self.forward_cone_rad = math.radians(forward_cone_deg)
        self.wall_range_max = wall_range_max
        self.gap_range_min = gap_range_min
        self.min_gap_width_rad = math.radians(min_gap_width_deg)
        self.required_consecutive_detections = required_consecutive_detections
        self.align_tolerance_rad = align_tolerance_rad
        self.cross_speed = cross_speed
        self.align_ang_gain = align_ang_gain
        self.cross_ang_gain = cross_ang_gain
        self.crossing_distance_m = crossing_distance_m
        self.stop_distance_m = stop_distance_m
        self.on_gap_confirmed = on_gap_confirmed
        self.on_crossed = on_crossed

        self.armed = False
        self.state = 'SEARCH'
        self.consecutive_hits = 0
        self.gap_bearing = None
        self.latest_center_range = None

        self.odom_start_xy = None
        self.latest_odom_xy = None
        self.crossed = False

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, 10
        )

        self.get_logger().info(
            f"DoorwayGapCrosser created (disarmed). Will search within "
            f"+/-{forward_cone_deg:.0f} deg of forward once armed."
        )

    def arm(self):
        if not self.armed:
            self.armed = True
            self.get_logger().info("DoorwayGapCrosser armed -- now searching for a gap.")

    # -----------------------------------------------------------------
    def _odom_callback(self, msg: Odometry):
        self.latest_odom_xy = (
            msg.pose.pose.position.x, msg.pose.pose.position.y
        )

    def _distance_traveled(self) -> float:
        if self.odom_start_xy is None or self.latest_odom_xy is None:
            return 0.0
        dx = self.latest_odom_xy[0] - self.odom_start_xy[0]
        dy = self.latest_odom_xy[1] - self.odom_start_xy[1]
        return math.hypot(dx, dy)

    # -----------------------------------------------------------------
    def _find_gap(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return None

        def angle_at(i):
            return msg.angle_min + i * msg.angle_increment

        def classify(r):
            if r is None or math.isnan(r) or math.isinf(r) or r >= self.gap_range_min:
                return 'far'
            if r <= self.wall_range_max:
                return 'close'
            return 'mid'

        indices = [i for i in range(n) if abs(angle_at(i)) <= self.forward_cone_rad]
        if not indices:
            return None

        best_run = None
        run_start = None

        for idx in indices:
            cls = classify(msg.ranges[idx])
            if cls == 'far':
                if run_start is None:
                    run_start = idx
            else:
                if run_start is not None:
                    run_end = idx - 1
                    best_run = self._better_run(best_run, (run_start, run_end))
                    run_start = None
        if run_start is not None:
            run_end = indices[-1]
            best_run = self._better_run(best_run, (run_start, run_end))

        if best_run is None:
            return None

        start_i, end_i = best_run
        width_rad = (end_i - start_i) * msg.angle_increment
        if width_rad < self.min_gap_width_rad:
            return None

        before_i = start_i - 1
        after_i = end_i + 1
        has_wall_before = (
            before_i >= 0 and
            not math.isnan(msg.ranges[before_i]) and
            not math.isinf(msg.ranges[before_i]) and
            msg.ranges[before_i] <= self.wall_range_max
        )
        has_wall_after = (
            after_i < n and
            not math.isnan(msg.ranges[after_i]) and
            not math.isinf(msg.ranges[after_i]) and
            msg.ranges[after_i] <= self.wall_range_max
        )
        if not (has_wall_before and has_wall_after):
            return None

        center_i = (start_i + end_i) // 2
        return angle_at(center_i)

    def _better_run(self, current_best, candidate):
        if current_best is None:
            return candidate
        cur_width = current_best[1] - current_best[0]
        cand_width = candidate[1] - candidate[0]
        return candidate if cand_width > cur_width else current_best

    # -----------------------------------------------------------------
    def _scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if n > 0:
            center_idx = int((0.0 - msg.angle_min) / msg.angle_increment)
            center_idx = max(0, min(n - 1, center_idx))
            window = msg.ranges[max(0, center_idx - 3):center_idx + 4]
            valid = [r for r in window if not math.isnan(r) and not math.isinf(r)]
            self.latest_center_range = min(valid) if valid else None

        if not self.armed or self.state != 'SEARCH':
            return

        bearing = self._find_gap(msg)
        if bearing is None:
            self.consecutive_hits = 0
            self.gap_bearing = None
            return

        self.gap_bearing = bearing
        self.consecutive_hits += 1

        if self.consecutive_hits >= self.required_consecutive_detections:
            self.get_logger().info(
                f"Gap confirmed at bearing {math.degrees(bearing):.1f} deg. Aligning."
            )
            self.state = 'ALIGN'
            if self.on_gap_confirmed is not None:
                self.on_gap_confirmed()

    # -----------------------------------------------------------------
    def step(self):
        """Call periodically (e.g. from a timer) to run the control loop."""
        if self.state == 'SEARCH':
            return  # no cmd_vel published; avoids fighting FollowPath's output

        if self.state == 'ALIGN':
            if self.gap_bearing is None:
                self.state = 'SEARCH'
                self.consecutive_hits = 0
                return
            error = self.gap_bearing
            if abs(error) <= self.align_tolerance_rad:
                self.get_logger().info("Aligned with gap. Crossing.")
                self.odom_start_xy = self.latest_odom_xy
                self.state = 'CROSS'
                return
            ang = max(-1.0, min(1.0, self.align_ang_gain * error))
            self._publish_cmd(0.0, ang)
            return

        if self.state == 'CROSS':
            if (self.latest_center_range is not None and
                    self.latest_center_range < self.stop_distance_m):
                self.get_logger().warn(
                    f"Obstacle at {self.latest_center_range:.2f} m ahead during "
                    f"crossing -- halting."
                )
                self._publish_cmd(0.0, 0.0)
                return

            traveled = self._distance_traveled()
            if traveled >= self.crossing_distance_m:
                self._publish_cmd(0.0, 0.0)
                self.state = 'DONE'
                self.crossed = True
                self.get_logger().info(f"Crossing complete ({traveled:.2f} m traveled).")
                if self.on_crossed is not None:
                    self.on_crossed()
                return

            ang = max(-0.5, min(0.5, self.cross_ang_gain * (self.gap_bearing or 0.0)))
            self._publish_cmd(self.cross_speed, ang)
            return

        if self.state == 'DONE':
            self._publish_cmd(0.0, 0.0)
            return

    def _publish_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)


class MissionSupervisor(Node):
    """
    Orchestrates the full mission:
      - Tracks distance traveled since start (via the crosser's odom
        subscription) and arms gap detection once past min_arm_distance_m.
      - On gap confirmed: pauses TFFollowerLocalOnly (cancels any
        in-flight FollowPath goal) so raw /cmd_vel control from the
        crosser doesn't fight with FollowPath's own output.
      - Steps the crosser's control loop on a timer while it's active.
      - On crossed: reads the current TF-estimated map-frame pose (still
        valid -- AMCL is still paused at this point), publishes it to
        /initialpose, resumes the lifecycle manager, then resumes
        TFFollowerLocalOnly.
    """

    def __init__(self,
                 follower: TFFollowerLocalOnly,
                 crosser: DoorwayGapCrosser,
                 manager: LifecycleManagerCommander,
                 map_frame: str = 'map',
                 robot_frame: str = 'base_link',
                 min_arm_distance_m: float = 3.0,
                 step_period_sec: float = 0.1):
        super().__init__('mission_supervisor')

        self.follower = follower
        self.crosser = crosser
        self.manager = manager
        self.map_frame = map_frame
        self.robot_frame = robot_frame
        self.min_arm_distance_m = min_arm_distance_m

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        self.start_odom_xy = None
        self.armed = False
        self.crossing_active = False
        self.relocalized = False

        crosser.on_gap_confirmed = self._on_gap_confirmed
        crosser.on_crossed = self._on_crossed

        self.timer = self.create_timer(step_period_sec, self._tick)

        self.get_logger().info(
            f"MissionSupervisor started. Will arm gap detection after "
            f"{min_arm_distance_m:.1f} m traveled from start."
        )

    def _tick(self):
        # Arm gap detection once far enough from the start (avoids
        # triggering on the exit doorway while still leaving).
        if not self.armed:
            if self.crosser.latest_odom_xy is None:
                return
            if self.start_odom_xy is None:
                self.start_odom_xy = self.crosser.latest_odom_xy
                return
            dx = self.crosser.latest_odom_xy[0] - self.start_odom_xy[0]
            dy = self.crosser.latest_odom_xy[1] - self.start_odom_xy[1]
            if math.hypot(dx, dy) >= self.min_arm_distance_m:
                self.armed = True
                self.crosser.arm()
            return

        # Step the crosser's control loop while it's actively
        # aligning/crossing (SEARCH state is a passive no-op, cheap to
        # call regardless).
        self.crosser.step()

    def _on_gap_confirmed(self):
        if self.crossing_active:
            return
        self.crossing_active = True
        self.get_logger().info(
            "Gap confirmed -- pausing TF following, handing control to "
            "DoorwayGapCrosser."
        )
        self.follower.pause()

    def _on_crossed(self):
        self.get_logger().info(
            "Crossing complete -- relocalizing."
        )
        self._relocalize()
        self.follower.resume()
        self.crossing_active = False

    def _relocalize(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time()
            )
        except TransformException as ex:
            self.get_logger().error(
                f"Could not get {self.map_frame}->{self.robot_frame} for "
                f"relocalization seed: {ex}. Resuming AMCL without a seed."
            )
            self.manager.resume()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = t.transform.translation.x
        msg.pose.pose.position.y = t.transform.translation.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = t.transform.rotation.z
        msg.pose.pose.orientation.w = t.transform.rotation.w

        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.0685
        msg.pose.covariance = cov

        self.initialpose_pub.publish(msg)
        self.get_logger().info(
            f"Published /initialpose estimate: "
            f"({t.transform.translation.x:.2f}, {t.transform.translation.y:.2f})"
        )

        resumed = self.manager.resume()
        if resumed:
            self.get_logger().info(
                "AMCL resumed. It should now converge near the seeded pose."
            )
            self.relocalized = True
        else:
            self.get_logger().error(
                "Failed to resume lifecycle manager -- AMCL remains paused."
            )


def main():
    rclpy.init()

    # Step 1: capture map->odom and freeze it as a STATIC transform, while
    # AMCL is still active. This MUST happen before pausing AMCL.
    freezer = MapOdomFreezer()
    froze_ok = freezer.capture_and_freeze(timeout_sec=10.0)
    if not froze_ok:
        freezer.get_logger().error(
            "Proceeding without a frozen map->odom transform -- "
            "target_pose lookups will likely fail."
        )

    # Step 2: PAUSE the localization lifecycle manager.
    manager = LifecycleManagerCommander('/lifecycle_manager_localization')
    manager.pause()

    # Step 3: create the follower and the (disarmed) gap crosser.
    follower = TFFollowerLocalOnly()
    crosser = DoorwayGapCrosser()

    # Step 4: create the supervisor, which arms the crosser once far
    # enough from the start, and orchestrates the pause/cross/relocalize/
    # resume sequence.
    supervisor = MissionSupervisor(follower, crosser, manager)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(freezer)
    executor.add_node(follower)
    executor.add_node(crosser)
    executor.add_node(supervisor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    freezer.destroy_node()
    follower.destroy_node()
    crosser.destroy_node()
    supervisor.destroy_node()
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()