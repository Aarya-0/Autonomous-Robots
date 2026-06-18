#!/usr/bin/env python3

"""
Trigger for waiting (3sec.):
ros2 service call /pause_tf std_srvs/srv/SetBool "{data: true}"
"""
import math
import rclpy
import time
import sys
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


def yaw_to_quaternion(yaw):
    return (
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0)
    )


class TrajectoryTFPublisher(Node):
    def __init__(self, start_area=1):
        super().__init__("trajectory_tf_publisher")

        self.br = TransformBroadcaster(self)

        self.parent_frame = "map"
        self.child_frame = "target_pose"

        self.timer_dt = 0.05
        self.start_area = start_area

        self.areas = [
            {
                "area_number": 1,
                "waypoints": [
                    ((0.0, 0.0), 0.3, (-0.2, 2.0)),
                ]
            },
            {
                "area_number": 2,
                "waypoints": [
                    ((-0.2, 2.0), 0.3, (1.0, 2.5)),
                    ((1.0, 2.5), 0.3, (1.0, 3.0)),
                    ((1.0, 3.0), 0.2, (1.0, 4.0)),
                    ((1.0, 4.0), 0.2, (0.0, 5.0)),
                    ((0.0, 5.0), 0.2, (-1.0, 5.0)),
                    ((-1.0, 5.0), 0.3, (-2.0, 4.0)),
                    ((-2.0, 4.0), 0.3, (-2.5, 3.5)),
                    ((-2.5, 3.5), 0.3, (-3.0, 3.5)),
                ]
            },
            {
                "area_number": 3,
                "waypoints": [
                    ((-3.0, 3.5), 0.2, (-6.0, 3.0)),
                    ((-6.0, 3.0), 0.2, (-3.5, 2.5)),
                    ((-3.5, 2.5), 0.2, (-4.0, 1.0)),
                    ((-4.0, 1.0), 0.2, (-4.0, 0.0)),
                ]
            },
            {
                "area_number": 4,
                "waypoints": [
                    ((-4.0, 0.0), 0.2, (-3.0, 0.0)),
                    ((-3.0, 0.0), 0.2, (-3.0, -1.0)),
                    ((-3.0, -1.0), 0.2, (-3.0, -2.0)),
                    ((-3.0, -2.0), 0.2, (-3.5, -2.5)),
                    ((-3.5, -2.5), 0.2, (-4.0, -2.5)),
                    ((-4.0, -2.5), 0.2, (-5.5, -2.5)),
                    ((-5.5, -2.5), 0.2, (-6.0, -1.0)),
                ]
            },
            {
                "area_number": 5,
                "waypoints": [
                    ((-6.0, -1.0), 0.2, (-7.0, -1.2)),
                    ((-7.0, -1.2), 0.2, (-8.0, -1.4)),
                    ((-8.0, -1.4), 0.3, (-8.5, -1.5)),
                    ((-8.5, -1.5), 0.3, (-9.0, -1.5)),
                    ((-9.0, -1.5), 0.3, (-9.0, -1.0)),
                    ((-9.0, -1.0), 0.4, (-9.0, 0.0)),
                    ((-9.0, 0.0), 0.3, (-9.0, 3.0)),
                    ((-9.0, 3.0), 0.3, (-8.0, 3.0)),
                    ((-8.0, 3.0), 0.3, (-7.0, 1.5)),
                    ((-7.0, 1.5), 0.3, (-6.5, 1.5)),
                ]
            },
            {
                "area_number": 6,
                "waypoints": [
                    ((-6.5, 1.5), 0.3, (-6.0, 1.5)),
                    ((-6.0, 1.5), 0.3, (-5.0, 3.0)),
                    ((-5.0, 3.0), 0.3, (-2.0, 3.5)),
                ]
            },
            {
                "area_number": 7,
                "waypoints": [
                    ((-2.0, 3.5), 0.3, (-0.3, 3.0)),
                    ((-0.3, 3.0), 0.1, (0.0, -2.0)),
                ]
            }
        ]

        self.segments = []
        self.paused = False
        self.pause_until = 0.0

        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()

        self.srv = self.create_service(
            SetBool,
            "pause_tf",
            self.pause_callback,
            callback_group=self.service_group
        )

        if self.start_area == 1:
            for area in self.areas:
                for segment in area["waypoints"]:
                    self.segments.append(segment)
        else:
            for area in self.areas:
                if area["area_number"] >= (self.start_area):
                    for segment in area["waypoints"]:
                        self.segments.append(segment)

        self.segment_idx = 0
        self.progress = 0.0

        self.timer = self.create_timer(
            self.timer_dt,
            self.publish_tf,
            callback_group=self.timer_group
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


def main(args=None):
    rclpy.init(args=args)
    start_area = 1
    if len(sys.argv) > 1:
        try:
            start_area = int(sys.argv[1])
        except ValueError:
            print("Usage: python3 tf_publisher.py <start_area>")
            return


    node = TrajectoryTFPublisher(start_area=start_area)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
