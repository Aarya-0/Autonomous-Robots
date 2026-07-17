#!/usr/bin/env python3
import math
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan #Required for scan
from geometry_msgs.msg import Twist #Required for cmdvel
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class tf_follower_node(Node): #Inherits from Node 

    def __init__(self): #Constructor
        super().__init__("tf_follower")#Modify name

        self.tf_buffer = Buffer() #Create tf buffer to store transforms
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("TF Listener Node Started")

    def timer_callback(self):

        try:
            # Equivalent to: tf2_echo map target_pose
            t = self.tf_buffer.lookup_transform(
                'map',          # target frame (reference frame)
                'target_pose',  # source frame (frame you want pose of)
                rclpy.time.Time()
            )

            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            self.get_logger().info(
                f"[map <- target_pose] x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )

        except TransformException as ex:
            self.get_logger().warn(f"TF not available: {ex}")




def main(args=None):
    rclpy.init(args=args)
    # Initialized ros2 communication
    node = tf_follower_node() #Modify Name
    try: #Required cuz tf does not always succeed
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__== "__main__":
    main()