#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))

class CalibrationLogger(Node):
    def __init__(self):
        super().__init__('calibration_logger')
        self.wheel_start = None
        self.wheel_last = None
        self.wheel_total_yaw = 0.0

        self.gt_start = None
        self.gt_last = None
        self.gt_total_yaw = 0.0

        self.create_subscription(Odometry, '/mobile_base_controller/odom', self.wheel_cb, 10)
        self.create_subscription(Odometry, '/ground_truth_odom', self.gt_cb, 10)
        self.get_logger().info('Logging both odom sources... drive now, Ctrl+C when done.')

    def wheel_cb(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.wheel_start is None:
            self.wheel_start = (x, y, yaw)
            self.wheel_last = (x, y, yaw)
            return
        dyaw = yaw - self.wheel_last[2]
        if dyaw > math.pi: dyaw -= 2*math.pi
        elif dyaw < -math.pi: dyaw += 2*math.pi
        self.wheel_total_yaw += dyaw
        self.wheel_last = (x, y, yaw)

    def gt_cb(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.gt_start is None:
            self.gt_start = (x, y, yaw)
            self.gt_last = (x, y, yaw)
            return
        dyaw = yaw - self.gt_last[2]
        if dyaw > math.pi: dyaw -= 2*math.pi
        elif dyaw < -math.pi: dyaw += 2*math.pi
        self.gt_total_yaw += dyaw
        self.gt_last = (x, y, yaw)

    def print_summary(self):
        if not self.wheel_start or not self.gt_start:
            print("Missing data from one or both topics.")
            return
        wheel_dist = math.hypot(self.wheel_last[0]-self.wheel_start[0],
                                 self.wheel_last[1]-self.wheel_start[1])
        gt_dist = math.hypot(self.gt_last[0]-self.gt_start[0],
                              self.gt_last[1]-self.gt_start[1])
        print("\n--- Calibration Summary ---")
        print(f"Wheel odom distance: {wheel_dist:.4f} m")
        print(f"Ground truth distance: {gt_dist:.4f} m")
        if wheel_dist > 0.01:
            print(f"  => wheel_radius_multiplier ≈ {gt_dist/wheel_dist:.5f}")
        print(f"Wheel odom yaw: {math.degrees(self.wheel_total_yaw):.2f} deg")
        print(f"Ground truth yaw: {math.degrees(self.gt_total_yaw):.2f} deg")
        if abs(self.wheel_total_yaw) > 0.01:
            print(f"  => wheel_separation_multiplier ≈ {abs(self.gt_total_yaw/self.wheel_total_yaw):.5f}")

def main():
    rclpy.init()
    node = CalibrationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()