#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class NavGoalClient(Node):

    def __init__(self):
        super().__init__("nav_goal_client")

        # Create action client
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
         # Subscriber to TF-generated goals
        self.sub = self.create_subscription(
            PoseStamped,
            '/target_goal',
            self.goal_callback,
            1
        )

        self.current_goal = None

        self.get_logger().info("Nav Goal Client started")

    def goal_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y

        # optional: ignore repeated goals
        new_goal = (round(x, 2), round(y, 2))
        if new_goal == self.current_goal:
            return

        self.current_goal = new_goal

        self.get_logger().info(f"Received goal: ({x:.2f}, {y:.2f})")
        self.send_goal(msg)

    def send_goal(self, pose: PoseStamped):
        if not self._action_client.server_is_ready():
            self.get_logger().warn("Nav2 action server not ready")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info("Sending goal to Nav2")

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        self.get_logger().info("Navigation completed")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"DIstance remaining: {feedback.distance_remaining:.2f} m")


def main():
    # Initialize ROS2
    rclpy.init()
    node = NavGoalClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()