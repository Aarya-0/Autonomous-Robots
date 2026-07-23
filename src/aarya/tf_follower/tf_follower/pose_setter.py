#!/usr/bin/env python3
"""
Reliably set the AMCL initial pose in a running Nav2 stack, selecting from
a fixed set of named poses (task1, task2, task3, task4, fullrun).

Nav2 (Humble) does not expose a callable service to set the initial pose;
AMCL only ever listens on the /initialpose topic. A single publish can be
dropped or missed if AMCL's subscriber isn't connected yet. This node works
around that by publishing repeatedly on /initialpose and confirming, via
/amcl_pose, that AMCL actually adopted the pose (both position AND
orientation) before exiting.

Which pose gets sent is chosen with a ROS 2 parameter named "task",
NOT a plain argparse flag -- because "--ros-args" is the standard ROS 2
mechanism for passing parameters on the command line, and only things
passed via "-p key:=value" after "--ros-args" are visible as ROS
parameters. Plain "--task task1" would just be swallowed/ignored by rclpy.

Usage
-----
    python3 initial_pose_setter.py --ros-args -p task:=task1
    python3 initial_pose_setter.py --ros-args -p task:=task2
    python3 initial_pose_setter.py --ros-args -p task:=task3
    python3 initial_pose_setter.py --ros-args -p task:=task4
    python3 initial_pose_setter.py --ros-args -p task:=fullrun

If this script is installed inside a ROS 2 package as an entry point, the
exact same flags work with `ros2 run`:

    ros2 run <your_package> initial_pose_setter --ros-args -p task:=task1

Optional parameters (all have sane defaults, override the same way):
    -p tolerance:=0.3          # meters, position confirmation radius
    -p yaw_tolerance:=0.15     # radians, orientation confirmation window
    -p timeout:=15.0           # seconds before giving up
    -p retry_period:=1.5       # seconds between /initialpose publishes
    -p frame_id:=map
"""

import sys
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


def _cov36(c00, c01, c10, c11, c55):
    """Build a flat 6x6 covariance array with only x/y/yaw terms populated,
    matching the sparsity pattern AMCL itself publishes/consumes."""
    cov = [0.0] * 36
    cov[0] = c00   # xx
    cov[1] = c01   # xy
    cov[6] = c10   # yx
    cov[7] = c11   # yy
    cov[35] = c55  # yaw-yaw
    return cov


# Hardcoded target poses. Position/orientation come straight from the
# recorded /amcl_pose echoes; covariance keeps only the nonzero terms
# that were actually present (x, y, yaw), everything else is zero.
POSES = {
    "task1": dict(
        x=0.05120338673907255,
        y=-0.015664507376229278,
        qz=0.7561583097597331,
        qw=0.6543887304815874,
        cov=_cov36(
            0.20140191494631504, 0.012530579180847774,
            0.012530579180847773, 0.18966314051328748,
            0.060475768529780254,
        ),
    ),
    "task2": dict(
        x=-1.8620469092038479,
        y=3.761961117199282,
        qz=-0.9973874416540617,
        qw=0.07223774104140933,
        cov=_cov36(
            0.1948871997662467, -0.005087664908682399,
            -0.005087664908682399, 0.222262372323625,
            0.05667874141637424,
        ),
    ),
    "task3": dict(
        x=-5.866050613241715,
        y=-0.9928104074362162,
        qz=-0.9976297741828851,
        qw=0.06881012762526842,
        cov=_cov36(
            0.2027285111045103, 0.0076371618974544475,
            0.007637161897455336, 0.1887619901132812,
            0.06176290605078023,
        ),
    ),
    "task4": dict(
        x=-7.298454776812748,
        y=1.2336242750281663,
        qz=0.08322543873127797,
        qw=0.9965307453099409,
        cov=_cov36(
            0.19936509803782343, 0.00573896958768394,
            0.005738969587682163, 0.22169425686517785,
            0.05803909891004318,
        ),
    ),
}
# "fullrun" starts the run from the same spot as task1.
POSES["fullrun"] = POSES["task1"]


def yaw_from_qz_qw(qz, qw):
    """Yaw (rad) from a quaternion that only rotates about Z (qx=qy=0),
    which is the case for every pose here (planar robot)."""
    return 2.0 * math.atan2(qz, qw)


def angle_diff(a, b):
    """Smallest signed difference a-b, wrapped to [-pi, pi]."""
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return d


class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__("initial_pose_setter")

        self.declare_parameter("task", "task1")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("tolerance", 0.3)        # meters
        self.declare_parameter("yaw_tolerance", 0.15)    # radians (~8.6 deg)
        self.declare_parameter("timeout", 15.0)          # seconds
        self.declare_parameter("retry_period", 1.5)      # seconds

        task_name = self.get_parameter("task").get_parameter_value().string_value
        if task_name not in POSES:
            valid = ", ".join(sorted(set(POSES.keys())))
            self.get_logger().error(
                f'Unknown task "{task_name}". Valid values: {valid}')
            raise SystemExit(2)

        pose = POSES[task_name]
        self.task_name = task_name
        self.x = pose["x"]
        self.y = pose["y"]
        self.qz = pose["qz"]
        self.qw = pose["qw"]
        self.cov = pose["cov"]
        self.target_yaw = yaw_from_qz_qw(self.qz, self.qw)

        self.frame_id = self.get_parameter("frame_id").value
        self.tolerance = self.get_parameter("tolerance").value
        self.yaw_tolerance = self.get_parameter("yaw_tolerance").value
        self.timeout = self.get_parameter("timeout").value
        self.retry_period = self.get_parameter("retry_period").value

        self.confirmed = False

        pub_qos = QoSProfile(depth=1)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", pub_qos)

        sub_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_cb, sub_qos)

        self.get_logger().info(
            f'Task "{self.task_name}": target x={self.x:.3f}, y={self.y:.3f}, '
            f'yaw={math.degrees(self.target_yaw):.1f} deg (frame="{self.frame_id}")')

    def make_msg(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = self.qz
        msg.pose.pose.orientation.w = self.qw
        msg.pose.covariance = self.cov
        return msg

    def amcl_pose_cb(self, msg):
        dx = msg.pose.pose.position.x - self.x
        dy = msg.pose.pose.position.y - self.y
        dist = (dx * dx + dy * dy) ** 0.5

        got_yaw = yaw_from_qz_qw(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        dyaw = abs(angle_diff(got_yaw, self.target_yaw))

        if dist <= self.tolerance and dyaw <= self.yaw_tolerance:
            self.get_logger().info(
                f'AMCL adopted "{self.task_name}" pose '
                f'(offset {dist:.3f} m <= {self.tolerance} m, '
                f'yaw off {math.degrees(dyaw):.1f} deg <= {math.degrees(self.yaw_tolerance):.1f} deg).')
            self.confirmed = True

    def run(self):
        start = time.time()
        last_pub = 0.0
        while rclpy.ok() and not self.confirmed and (time.time() - start) < self.timeout:
            now = time.time()
            if now - last_pub >= self.retry_period:
                self.pub.publish(self.make_msg())
                self.get_logger().info(
                    f'Published /initialpose for "{self.task_name}", waiting for AMCL confirmation...')
                last_pub = now
            rclpy.spin_once(self, timeout_sec=0.2)

        if not self.confirmed:
            self.get_logger().error(
                f'Timed out after {self.timeout}s without AMCL confirming "{self.task_name}". '
                f'Check that AMCL is up and /amcl_pose is publishing.')
        return self.confirmed


def main():
    rclpy.init()
    try:
        node = InitialPoseSetter()
    except SystemExit as e:
        rclpy.shutdown()
        sys.exit(e.code)

    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()