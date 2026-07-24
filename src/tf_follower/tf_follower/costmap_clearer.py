#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap


class PeriodicCostmapClearer(Node):

    def __init__(self):
        super().__init__('periodic_costmap_clearer')

        self.declare_parameter('clear_interval_sec', 1.0)
        interval = self.get_parameter('clear_interval_sec').value

        self.cli = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )

        self.timer = self.create_timer(interval, self.clear_costmap)
        self._pending = False

        self.get_logger().info(
            f"Periodic costmap clearer started, interval={interval}s"
        )

    def clear_costmap(self):
        if self._pending:
            # last call hasn't returned yet, skip this tick
            return

        if not self.cli.service_is_ready():
            self.get_logger().warn("Clear service not ready, skipping")
            return

        self._pending = True
        future = self.cli.call_async(ClearEntireCostmap.Request())
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        self._pending = False
        try:
            future.result()
        except Exception as e:
            self.get_logger().warn(f"Clear call failed: {e}")


def main():
    rclpy.init()
    node = PeriodicCostmapClearer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()