#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class TrajectoryTFPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_tf_publisher")

        self.br = TransformBroadcaster(self)

        self.parent_frame = "map"
        self.child_frame = "target_pose"

        self.speed = 0.5  # m/s
        self.timer_dt = 0.05

        self.points = [
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 0.8),
            (0.0, 1.0),
            (0.0, 2.0),
            (1.0, 2.5),
            (1.0, 3.0),
            (0.0, 4.0),
            (-1.0, 4.0),
            (-2.0, 3.5),
            (-3.0, 3.5),
            (-4.0, 2.5),
            (-4.0, 0.0),
            (-3.0, -1.0),
            (-3.0, -2.0),
            (-4.0, -2.5),
            (-5.3, -2.5),
            (-5.3, -1.0),
            (-8.0, -1.0),
            (-8.0, 1.0),
            (-6.0, 1.0),
            (-4.0, 2.5),
            (-2.0, 3.5),
            (0.0, 1.0),
        ]

        self.segment = 0
        self.progress = 0.0

        self.timer = self.create_timer(self.timer_dt, self.publish_tf)

    def publish_tf(self):
        p0 = self.points[self.segment]
        p1 = self.points[(self.segment + 1) % len(self.points)]

        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        length = math.hypot(dx, dy)

        if length < 1e-6:
            self.segment = (self.segment + 1) % len(self.points)
            self.progress = 0.0
            return

        self.progress += self.speed * self.timer_dt / length

        if self.progress >= 1.0:
            self.segment = (self.segment + 1) % len(self.points)
            self.progress = 0.0
            return

        x = p0[0] + self.progress * dx
        y = p0[1] + self.progress * dy
        yaw = math.atan2(dy, dx)


        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.child_frame

        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = 0.0

        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        self.br.sendTransform(msg)


def main():
    rclpy.init()
    node = TrajectoryTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()