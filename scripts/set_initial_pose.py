#!/usr/bin/env python3
"""
Reliably set the AMCL initial pose in a running Nav2 stack.

Nav2 (Humble) does not expose a callable service to set the initial pose;
AMCL only ever listens on the /initialpose topic. A single "--once" publish
can be dropped or missed if AMCL's subscriber isn't connected yet, which is
almost certainly why it "doesn't always work". This node works around that
by publishing repeatedly on /initialpose and confirming, via /amcl_pose,
that AMCL actually adopted the pose before exiting.
"""

import sys
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPoseSetter(Node):
    def __init__(self, x, y, yaw_z, yaw_w, frame_id, tolerance, timeout, retry_period):
        super().__init__('initial_pose_setter')

        self.x = x
        self.y = y
        self.z = yaw_z
        self.w = yaw_w
        self.frame_id = frame_id
        self.tolerance = tolerance
        self.timeout = timeout
        self.retry_period = retry_period
        self.confirmed = False

        pub_qos = QoSProfile(depth=1)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', pub_qos)

        sub_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_cb, sub_qos)

    def make_msg(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = self.z
        msg.pose.pose.orientation.w = self.w
        # Same covariance defaults you were already using
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891909122467
        return msg

    def amcl_pose_cb(self, msg):
        dx = msg.pose.pose.position.x - self.x
        dy = msg.pose.pose.position.y - self.y
        dist = (dx * dx + dy * dy) ** 0.5
        if dist <= self.tolerance:
            self.get_logger().info(
                f'AMCL adopted pose (offset {dist:.3f} m <= tolerance {self.tolerance} m).')
            self.confirmed = True

    def run(self):
        start = time.time()
        last_pub = 0.0
        while rclpy.ok() and not self.confirmed and (time.time() - start) < self.timeout:
            now = time.time()
            if now - last_pub >= self.retry_period:
                self.pub.publish(self.make_msg())
                self.get_logger().info('Published /initialpose, waiting for AMCL confirmation...')
                last_pub = now
            rclpy.spin_once(self, timeout_sec=0.2)

        if not self.confirmed:
            self.get_logger().error(
                f'Timed out after {self.timeout}s without AMCL confirming the pose. '
                f'Check that AMCL is up and /amcl_pose is publishing.')
        return self.confirmed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw-z', type=float, required=True, help='orientation quaternion z')
    parser.add_argument('--yaw-w', type=float, required=True, help='orientation quaternion w')
    parser.add_argument('--frame-id', type=str, default='map')
    parser.add_argument('--tolerance', type=float, default=0.3, help='meters, confirmation radius')
    parser.add_argument('--timeout', type=float, default=15.0, help='seconds before giving up')
    parser.add_argument('--retry-period', type=float, default=1.5, help='seconds between publishes')
    args, _ = parser.parse_known_args()  # tolerate a trailing --ros-args block

    rclpy.init()
    node = InitialPoseSetter(
        args.x, args.y, args.yaw_z, args.yaw_w,
        args.frame_id, args.tolerance, args.timeout, args.retry_period)
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()