#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

OCCUPIED_THRESHOLD = 50  # local costmap value above which a cell counts as an obstacle


class SnapshotNode(Node):

    def __init__(self):
        super().__init__('snapshot_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Latched QoS so the snapshot layer persists for late-joining costmap subscribers
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.persistent_grid = None       # nav_msgs/OccupancyGrid, sized to match /map
        self.persistent_data = None       # numpy int8 array, same size

        self.latest_local_costmap = None

        # Get map metadata once (latched topic)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, latched_qos
        )

        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.local_costmap_callback, 10
        )

        self.trigger_sub = self.create_subscription(
            Empty, '/take_snapshot', self.trigger_callback, 10
        )

        self.snapshot_pub = self.create_publisher(
            OccupancyGrid, '/snapshot_obstacles', latched_qos
        )

        self.get_logger().info("Snapshot node started, waiting for /map metadata")

    def map_callback(self, msg: OccupancyGrid):
        if self.persistent_grid is not None:
            return  # only need this once
        self.persistent_grid = OccupancyGrid()
        self.persistent_grid.header.frame_id = 'map'
        self.persistent_grid.info = msg.info
        self.persistent_data = np.full(
            (msg.info.height * msg.info.width,), -1, dtype=np.int8
        )
        self.get_logger().info(
            f"Got map metadata: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m"
        )

    def local_costmap_callback(self, msg: OccupancyGrid):
        self.latest_local_costmap = msg

    def trigger_callback(self, _msg: Empty):
        if self.persistent_grid is None:
            self.get_logger().warn("No /map metadata yet, cannot snapshot")
            return
        if self.latest_local_costmap is None:
            self.get_logger().warn("No local costmap received yet, cannot snapshot")
            return

        self.get_logger().info("Snapshot triggered — processing local costmap")
        costmap = self.latest_local_costmap
        info = costmap.info
        data = np.array(costmap.data, dtype=np.int8).reshape(info.height, info.width)

        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                costmap.header.frame_id,   # normally 'odom'
                rclpy.time.Time()          # latest available, avoids stale-stamp TF errors
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF error during snapshot: {ex}")
            return

        occupied_js, occupied_is = np.where(data >= OCCUPIED_THRESHOLD)
        count = 0

        for j, i in zip(occupied_js, occupied_is):
            wx = info.origin.position.x + (i + 0.5) * info.resolution
            wy = info.origin.position.y + (j + 0.5) * info.resolution

            pt = PointStamped()
            pt.header.frame_id = costmap.header.frame_id
            pt.point.x = wx
            pt.point.y = wy
            pt.point.z = 0.0

            try:
                pt_map = do_transform_point(pt, tf)
            except TransformException:
                continue

            mi = int((pt_map.point.x - self.persistent_grid.info.origin.position.x)
                      / self.persistent_grid.info.resolution)
            mj = int((pt_map.point.y - self.persistent_grid.info.origin.position.y)
                      / self.persistent_grid.info.resolution)

            if 0 <= mi < self.persistent_grid.info.width and 0 <= mj < self.persistent_grid.info.height:
                idx = mj * self.persistent_grid.info.width + mi
                self.persistent_data[idx] = 100
                count += 1

        self.get_logger().info(f"Marked {count} new occupied cells in persistent snapshot grid")

        self.persistent_grid.header.stamp = self.get_clock().now().to_msg()
        self.persistent_grid.data = self.persistent_data.tolist()
        self.snapshot_pub.publish(self.persistent_grid)


def main():
    rclpy.init()
    node = SnapshotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()