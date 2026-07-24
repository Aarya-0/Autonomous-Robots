#!/usr/bin/env python3
"""
Standalone out-of-map navigation script.

  1. Pauses Nav2's navigation lifecycle manager (controller_server,
     planner_server, bt_navigator, behavior_server) so nothing else can
     publish to /cmd_vel while this script drives directly.
  2. Runs a TF-follower + simplified DWA obstacle avoider, publishing
     /cmd_vel directly. No Nav2, no action server, no map/AMCL
     dependency -- everything here works in `robot_frame` off of live
     TF + /scan_raw.
  3. On shutdown (Ctrl+C), stops the robot and RESUMES the Nav2
     lifecycle manager, so this is genuinely "temporary" -- Nav2 is
     left running normally afterward.

Algorithm: simplified Dynamic Window Approach (DWA). Chosen over
potential fields (fail at narrow symmetric openings -- attractive +
repulsive vectors cancel right in a doorway), VFH (better doorway
handling but heavy histogram/threshold tuning), and pure pursuit (needs
a precomputed path, not a single moving TF point). DWA scores candidate
(v, w) trajectories directly against current sensor data, so there's no
vector-cancellation failure mode and no oscillation snap between two
fixed behaviors.

Design notes:
  - Uses self.get_clock().now() (ROS clock, sim-time-safe) throughout.
  - lookup_transform(robot_frame, target_frame) gives distance/bearing
    to the target directly, no manual odom subtraction.
  - Missing scan/TF data has a defined, safe behavior (stop), not
    undefined behavior.
  - No valid (non-colliding) trajectory -> recovery rotation toward
    whichever side has more clearance, rather than freezing (avoids
    deadlock).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav2_msgs.srv import ManageLifecycleNodes
from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class LifecycleManagerCommander(Node):
    """
    One-shot helper: calls <manager_name>/manage_nodes with PAUSE/RESUME.

    Nav2's lifecycle_manager nodes are NOT lifecycle nodes themselves --
    `ros2 lifecycle get/set` do not work on them. They expose a separate
    service, manage_nodes (nav2_msgs/srv/ManageLifecycleNodes), which is
    the only way to actually stop/start them managing the nodes they own.
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

        self.get_logger().warn(f"{self.manager_name} {label} call did not report success.")
        return False

    def pause(self, timeout_sec: float = 5.0) -> bool:
        return self._send_command(self.PAUSE, "pause", timeout_sec)

    def resume(self, timeout_sec: float = 5.0) -> bool:
        return self._send_command(self.RESUME, "resume", timeout_sec)


class OutOfMapFollower(Node):
    """
    Continuously follows `target_frame` (TF) using a simplified DWA
    obstacle avoider, publishing directly to /cmd_vel. Runs forever
    (stops/hovers when within distance_threshold, resumes driving if the
    target moves away again) -- no action server, no terminal state.
    """

    def __init__(self,
                 target_frame: str = 'target_pose',
                 robot_frame: str = 'base_link',
                 scan_topic: str = '/scan_raw',
                 odom_topic: str = '/mobile_base_controller/odom',
                 cmd_vel_topic: str = '/cmd_vel',
                 control_period_sec: float = 0.05,
                 max_lin_vel: float = 0.4,
                 min_lin_vel: float = 0.0,
                 max_ang_vel: float = 1.0,
                 max_lin_accel: float = 0.5,
                 max_ang_accel: float = 2.0,
                 v_samples: int = 7,
                 w_samples: int = 15,
                 sim_time_sec: float = 1.5,
                 sim_dt_sec: float = 0.1,
                 robot_radius: float = 0.275,
                 safety_margin: float = 0.10,
                 narrow_passage_clearance: float = 0.15,
                 heading_gain: float = 1.0,
                 clearance_gain: float = 1.2,
                 velocity_gain: float = 0.6,
                 distance_threshold: float = 0.3,
                 recovery_ang_vel: float = 0.3):
        super().__init__('out_of_map_follower')

        self.target_frame = target_frame
        self.robot_frame = robot_frame
        self.control_period_sec = control_period_sec

        self.max_lin_vel = max_lin_vel
        self.min_lin_vel = min_lin_vel
        self.max_ang_vel = max_ang_vel
        self.max_lin_accel = max_lin_accel
        self.max_ang_accel = max_ang_accel

        self.v_samples = v_samples
        self.w_samples = w_samples
        self.sim_time_sec = sim_time_sec
        self.sim_dt_sec = sim_dt_sec

        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.narrow_passage_clearance = narrow_passage_clearance

        self.heading_gain = heading_gain
        self.clearance_gain = clearance_gain
        self.velocity_gain = velocity_gain

        self.distance_threshold = distance_threshold
        self.recovery_ang_vel = recovery_ang_vel

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_scan = None
        self.latest_odom_twist = (0.0, 0.0)
        self.last_cmd = (0.0, 0.0)  # our own last COMMANDED (v, w) -- used for
                                     # the dynamic window instead of measured
                                     # odom, so a laggy/deadbanded base
                                     # controller can't stall the ramp-up
                                     # (odom staying ~0 would otherwise keep
                                     # re-capping v_hi at one accel-step above
                                     # zero, forever).
        self.last_distance_to_target = None

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self._scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)

        self.timer = self.create_timer(control_period_sec, self._control_tick)

        self.get_logger().info(
            f"OutOfMapFollower started. Tracking '{target_frame}' relative to "
            f"'{robot_frame}', publishing directly to '{cmd_vel_topic}'."
        )

    # -----------------------------------------------------------------
    def _scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def _odom_callback(self, msg: Odometry):
        self.latest_odom_twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def stop(self):
        self._publish_cmd(0.0, 0.0)

    def _publish_cmd(self, v, w):
        self.last_cmd = (v, w)
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)

    # -----------------------------------------------------------------
    def _get_target_relative_pose(self):
        """Returns (distance, bearing_rad) of target_frame relative to
        robot_frame, or None if unavailable."""
        try:
            t = self.tf_buffer.lookup_transform(self.robot_frame, self.target_frame, Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'{self.robot_frame}->{self.target_frame} lookup failed: {ex}',
                throttle_duration_sec=2.0
            )
            return None
        dx = t.transform.translation.x
        dy = t.transform.translation.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    # -----------------------------------------------------------------
    def _scan_to_points(self):
        msg = self.latest_scan
        if msg is None:
            return []
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if not math.isnan(r) and not math.isinf(r) and msg.range_min < r < msg.range_max:
                points.append((r * math.cos(angle), r * math.sin(angle)))
            angle += msg.angle_increment
        return points

    # -----------------------------------------------------------------
    def _dynamic_window(self):
        dt = self.control_period_sec
        cur_v, cur_w = self.last_cmd  # commanded, not measured -- see note in __init__
        v_lo = max(self.min_lin_vel, cur_v - self.max_lin_accel * dt)
        v_hi = min(self.max_lin_vel, cur_v + self.max_lin_accel * dt)
        w_lo = max(-self.max_ang_vel, cur_w - self.max_ang_accel * dt)
        w_hi = min(self.max_ang_vel, cur_w + self.max_ang_accel * dt)
        return v_lo, v_hi, w_lo, w_hi

    def _simulate(self, v, w):
        x = y = th = 0.0
        poses = [(x, y, th)]
        steps = max(1, int(self.sim_time_sec / self.sim_dt_sec))
        for _ in range(steps):
            x += v * math.cos(th) * self.sim_dt_sec
            y += v * math.sin(th) * self.sim_dt_sec
            th += w * self.sim_dt_sec
            poses.append((x, y, th))
        return poses

    def _min_clearance(self, poses, obstacle_points):
        if not obstacle_points:
            return float('inf')
        min_clear = float('inf')
        for (px, py, _th) in poses:
            for (ox, oy) in obstacle_points:
                d = math.hypot(px - ox, py - oy) - self.robot_radius
                if d < min_clear:
                    min_clear = d
        return min_clear

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def _select_trajectory(self, goal_bearing):
        v_lo, v_hi, w_lo, w_hi = self._dynamic_window()
        obstacle_points = self._scan_to_points()

        best_score = -float('inf')
        best_cmd = None
        best_clearance = 0.0

        for i in range(self.v_samples):
            v = v_lo + (v_hi - v_lo) * (i / max(1, self.v_samples - 1))
            for j in range(self.w_samples):
                w = w_lo + (w_hi - w_lo) * (j / max(1, self.w_samples - 1))

                poses = self._simulate(v, w)
                clearance = self._min_clearance(poses, obstacle_points)
                if clearance < self.safety_margin:
                    continue

                final_th = poses[-1][2]
                heading_error = abs(self._normalize_angle(goal_bearing - final_th))
                heading_score = 1.0 - (heading_error / math.pi)
                clearance_score = min(clearance, 2.0) / 2.0
                velocity_score = v / self.max_lin_vel if self.max_lin_vel > 0 else 0.0

                score = (self.heading_gain * heading_score +
                         self.clearance_gain * clearance_score +
                         self.velocity_gain * velocity_score)

                if score > best_score:
                    best_score = score
                    best_cmd = (v, w)
                    best_clearance = clearance

        return best_cmd, best_clearance

    def _recovery_command(self):
        points = self._scan_to_points()
        if not points:
            return 0.0, 0.0
        left = [p for p in points if p[1] > 0]
        right = [p for p in points if p[1] <= 0]
        left_clear = min((math.hypot(*p) for p in left), default=float('inf'))
        right_clear = min((math.hypot(*p) for p in right), default=float('inf'))
        return 0.0, (self.recovery_ang_vel if left_clear > right_clear else -self.recovery_ang_vel)

    # -----------------------------------------------------------------
    def _control_tick(self):
        rel = self._get_target_relative_pose()
        if rel is None:
            self.stop()
            return

        distance, bearing = rel
        self.last_distance_to_target = distance

        if distance <= self.distance_threshold:
            self.stop()
            return

        if self.latest_scan is None:
            self.stop()
            return

        cmd, clearance = self._select_trajectory(bearing)
        if cmd is None:
            v, w = self._recovery_command()
        else:
            v, w = cmd
            if clearance < self.narrow_passage_clearance:
                v *= 0.5

        self._publish_cmd(v, w)


def main():
    rclpy.init()

    # Step 1: pause Nav2's navigation stack so nothing else can publish
    # to /cmd_vel while this script drives directly.
    nav_manager = LifecycleManagerCommander('/lifecycle_manager_navigation')
    nav_manager.pause()

    follower = OutOfMapFollower()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(follower)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        follower.stop()
        # Step 2: resume Nav2 -- this is meant to be temporary.
        nav_manager.resume()
        nav_manager.destroy_node()
        follower.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()