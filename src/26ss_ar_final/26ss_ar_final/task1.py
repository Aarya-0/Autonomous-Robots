#!/usr/bin/env python3
# Not getting feedback correctly wtf
from ar_final_interfaces.action import ArFinal
from rclpy.action import ActionServer
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import asyncio

from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


# ============================================================
# NODE 1: TF -> Goal Publisher
# ============================================================
class TFGoalPublisher(Node):

    def __init__(self):
        super().__init__('tf_goal_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, '/target_goal', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.last_goal = None

        self.get_logger().info("TF Goal Publisher started")

    def timer_callback(self):

        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'target_pose',
                rclpy.time.Time()
            )

            x = t.transform.translation.x
            y = t.transform.translation.y

        except TransformException as ex:
            self.get_logger().warn(f"TF error: {ex}")
            return

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


# ============================================================
# NODE 2: Nav2 Action Client
# ============================================================
class NavGoalClient(Node):

    def __init__(self):
        super().__init__('nav_goal_client')

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.sub = self.create_subscription(
            PoseStamped,
            '/target_goal',
            self.goal_callback,
            10
        )

        self.current_goal = None

        self.get_logger().info("Nav Goal Client started")

    def goal_callback(self, msg: PoseStamped):

        x = msg.pose.position.x
        y = msg.pose.position.y

        new_goal = (round(x, 2), round(y, 2))
        if new_goal == self.current_goal:
            return

        self.current_goal = new_goal

        self.get_logger().info(f"Received goal: ({x:.2f}, {y:.2f})")
        self.send_goal(msg)

    def send_goal(self, pose: PoseStamped):

        if not self._action_client.server_is_ready():
            self.get_logger().warn("Nav2 not ready")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info("Sending goal to Nav2")

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

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
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Distance remaining: {fb.distance_remaining:.2f}"
        )
# ============================================================
# NODE 3: Distance Action Server
# ============================================================

class DistanceActionServer(Node):

    def __init__(self):
        super().__init__('distance_action_server')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.action_server = ActionServer(
            self,
            ArFinal,
            '/ar_final',
            self.execute_callback
        )

        self.get_logger().info("ArFinal action server started")


    async def execute_callback(self, goal_handle):

        self.get_logger().info(
            f"Received task: {goal_handle.request.task}"
        )

        feedback = ArFinal.Feedback()
        result = ArFinal.Result()

        while rclpy.ok():

            try:

                robot_tf = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time()
                )

                target_tf = self.tf_buffer.lookup_transform(
                    'map',
                    'target_pose',
                    rclpy.time.Time()
                )

                dx = (
                    target_tf.transform.translation.x -
                    robot_tf.transform.translation.x
                )

                dy = (
                    target_tf.transform.translation.y -
                    robot_tf.transform.translation.y
                )

                distance = (dx**2 + dy**2)**0.5

                feedback.distance = float(distance)

                goal_handle.publish_feedback(feedback)

                self.get_logger().info(
                    f"Distance: {distance:.2f} m"
                )


                if distance < 0.2:
                    goal_handle.succeed()
                    result.message = "Reached target"
                    return result


            except TransformException as e:

                self.get_logger().warn(
                    f"TF unavailable: {e}"
                )


            except Exception as e:

                self.get_logger().error(
                    f"Action error: {e}"
                )

                goal_handle.abort()
                result.message = "Action failed"
                return result


            await asyncio.sleep(0.5)

# ============================================================
# MAIN (RUN BOTH NODES)
# ============================================================
def main():
    rclpy.init()

    tf_node = TFGoalPublisher()
    nav_node = NavGoalClient()
    action_node = DistanceActionServer()

    # Use a MultiThreadedExecutor so both nodes run properly
    executor = rclpy.executors.MultiThreadedExecutor()

    executor.add_node(tf_node)
    executor.add_node(nav_node)
    executor.add_node(action_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    tf_node.destroy_node()
    nav_node.destroy_node()
    action_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()