#!/usr/bin/env python3
# import math
import rclpy 
from rclpy.node import Node
# from sensor_msgs.msg import LaserScan #Required for scan
# from geometry_msgs.msg import Twist #Required for cmdvel
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped


class tf_follower_node(Node): #Inherits from Node 

    def __init__(self): #Constructor
        super().__init__("tf_follower")#Modify name

        self.tf_buffer = Buffer() #Create tf buffer to store transforms
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher (goal output)
        self.goal_pub = self.create_publisher(PoseStamped, '/target_goal', 10)
        
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.last_goal = None

        self.get_logger().info("TF Listener Node Started")

    def timer_callback(self):

        try:
            # Equivalent to: tf2_echo map target_pose
            t = self.tf_buffer.lookup_transform(
                'map',          # target frame (reference frame)
                'target_pose',  # source frame (frame you want pose of)
                rclpy.time.Time()
            )

            
            # z = t.transform.translation.z

            # self.get_logger().info(
            #     f"[map <- target_pose] x={x:.3f}, y={y:.3f}, z={z:.3f}"
            # )

        except TransformException as ex:
            self.get_logger().warn(f"TF not available: {ex}")

        x = t.transform.translation.x
        y = t.transform.translation.y

        # avoid spamming same goal
        new_goal = (round(x, 2), round(y, 2))
        if new_goal == self.last_goal:
            return

        self.last_goal = new_goal

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        msg.pose.orientation.w = 1.0

        self.get_logger().info(f"Publishing goal: ({x:.2f}, {y:.2f})")
        self.goal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    # Initialized ros2 communication
    node = tf_follower_node() #Modify Name
    try: #Required cuz tf does not always succeed
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__== "__main__":
    main()