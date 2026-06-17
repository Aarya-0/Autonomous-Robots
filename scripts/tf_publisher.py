#!/usr/bin/env python3

"""
For waiting:
ros2 service call /pause_tf std_srvs/srv/SetBool "{data: true}"
"""

import math
import rclpy
import time
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    return (
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0)
    )


class TrajectoryTFPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_tf_publisher")

        self.br = TransformBroadcaster(self)

        self.parent_frame = "map"
        self.child_frame = "target_pose"

        self.timer_dt = 0.05

        self.areas = [
            {
                "area_number": 1,
                "waypoints": [
                    ((0.0, 0.0), 0.5, (1.0, 0.0)),
                    ((1.0, 0.0), 0.5, (0.0, 0.5)),
                    ((0.0, 0.5), 0.5, (-0.2, 2.0)),
                ]
            },
            {
                "area_number": 2,
                "waypoints": [
                    ((-0.2, 2.0), 0.5, (1.0, 2.5)),
                    ((1.0, 2.5), 0.5, (1.0, 3.0)),
                    ((1.0, 3.0), 0.5, (1.0, 4.0)),
                    ((1.0, 4.0), 0.5, (0.0, 5.0)),
                    ((0.0, 5.0), 0.5, (-1.0, 5.0)),
                    ((-1.0, 5.0), 0.5, (-2.0, 4.0)),
                    ((-2.0, 4.0), 0.5, (-2.5, 3.5)),
                    ((-2.5, 3.5), 0.5, (-3.0, 3.5)),
                ]
            },
            {
                "area_number": 3,
                "waypoints": [
                    ((-3.0, 3.5), 0.5, (-4.5, 3.0)),
                    ((-4.5, 3.0), 0.5, (-4.0, 1.0)),
                    ((-4.0, 1.0), 0.5, (-4.0, 0.0)),
                ]
            },
            {
                "area_number": 4,
                "waypoints": [
                    ((-4.0, 0.0), 0.5, (-3.0, 0.0)),
                    ((-3.0, 0.0), 0.5, (-3.0, -1.0)),
                    ((-3.0, -1.0), 0.5, (-3.0, -2.0)),
                    ((-3.0, -2.0), 0.5, (-3.5, -2.5)),
                    ((-3.5, -2.5), 0.5, (-4.0, -2.5)),
                    ((-4.0, -2.5), 0.5, (-5.5, -2.5)),
                    ((-5.5, -2.5), 0.5, (-6.0, -1.0)),
                    ((-6.0, -1.0), 0.5, (-7.0, -1.2)),
                    ((-7.0, -1.2), 0.5, (-8.0, -1.4)),
                    ((-8.0, -1.4), 1.0, (-8.5, -1.5)),
                    ((-8.5, -1.5), 0.5, (-9.0, -1.5)),
                ]
            },
            {
                "area_number": 5,
                "waypoints": [
                    ((-9.0, -1.5), 0.5, (-9.0, -1.0)),
                    ((-9.0, -1.0), 0.1, (-9.0, 0.0)),
                    ((-9.0, 0.0), 0.5, (-9.0, 3.0)),
                    ((-9.0, 3.0), 0.5, (-8.0, 3.0)),
                    ((-8.0, 3.0), 0.5, (-7.0, 1.5)),
                    ((-7.0, 1.5), 0.5, (-6.0, 1.5)),
                ]
            },
            {
                "area_number": 6,
                "waypoints": [
                    ((-6.0, 1.5), 0.5, (-3.5, 2.5)),
                    ((-3.5, 2.5), 0.5, (-5.0, 3.0)),
                    ((-5.0, 3.0), 0.5, (-3.0, 3.5)),
                    ((-3.0, 3.5), 0.5, (-2.0, 3.5)),
                ]
            },
            {
                "area_number": 7,
                "waypoints": [
                    ((-2.0, 3.5), 0.5, (-0.5, 3.0)),
                    ((-0.5, 3.0), 0.5, (0.0, -2.0)),
                ]
            }
        ]

        self.segments = []
        self.paused = False
        self.pause_until = 0.0

        self.srv = self.create_service(
            SetBool,
            "pause_tf",
            self.pause_callback
        )

        for area in self.areas:
            for segment in area["waypoints"]:
                self.segments.append(segment)

        self.segment_idx = 0
        self.progress = 0.0

        self.timer = self.create_timer(
            self.timer_dt,
            self.publish_tf
        )
    
    def pause_callback(self, request, response):
        if request.data:
            self.paused = True
            self.pause_until = time.time() + 3.0
            response.success = True
            response.message = "Paused for 3 seconds"
        else:
            self.paused = False
            response.success = True
            response.message = "Resumed"
        return response

    def publish_tf(self):
        if self.paused:
            if time.time() < self.pause_until:
                self.get_logger().info(f"still waiting ({time.time() - self.pause_until})s")
                return
            else:
                self.paused = False

        if self.segment_idx >= len(self.segments):
            return

        start, speed, goal = self.segments[self.segment_idx]
        self.get_logger().info(
            f"Segment {self.segment_idx}: "
            f"Goal={goal}, Speed={speed:.2f} m/s"
        )

        dx = goal[0] - start[0]
        dy = goal[1] - start[1]

        distance = math.sqrt(dx * dx + dy * dy)

        if distance < 1e-6:
            self.segment_idx += 1
            self.progress = 0.0
            return

        travel_time = distance / speed

        self.progress += self.timer_dt / travel_time

        if self.progress >= 1.0:
            self.segment_idx += 1
            self.progress = 0.0

            if self.segment_idx >= len(self.segments):
                x = goal[0]
                y = goal[1]
            else:
                x = goal[0]
                y = goal[1]
        else:
            x = start[0] + dx * self.progress
            y = start[1] + dy * self.progress

        yaw = math.atan2(dy, dx)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        tf_msg = TransformStamped()

        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.child_frame

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.br.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = TrajectoryTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
