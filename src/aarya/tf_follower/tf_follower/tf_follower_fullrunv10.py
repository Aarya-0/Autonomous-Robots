#!/usr/bin/env python3
# Works at high freq with clearer to tackle moving obstacles well. 3sec is good
# Added head tilt feature
# Too computation heavy and low performance
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


# ============================================================
# NODE 1: TF -> Goal Publisher
# ============================================================
class TFGoalPublisher(Node):

    def __init__(self):
        super().__init__('tf_goal_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, '/target_goal', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

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
# NODE 3: Periodic Head Sweep (ground obstacle coverage)
# ============================================================
class HeadSweepNode(Node):
    """
    Periodically tilts the head down to scan the near-field ground
    blind spot, then returns to neutral. Every few cycles it also
    pans sideways to widen horizontal coverage (doorways, corners).

    Joint order for TIAGo's head_controller is [head_1_joint (pan),
    head_2_joint (tilt)]. Pan: + is left, - is right (radians).
    Tilt: negative looks down, 0.0 is level, positive looks up.

    Tuned to match a 3.0s costmap clear / 3.0s observation_persistence:
    one full dip completes well within that window so a fresh ground
    observation is always in the buffer when the clear runs.
    """

    NEUTRAL = (0.0, -0.3)
    TILT_DOWN = (0.0, -0.5)     # look down at near-field ground
    PAN_LEFT_DOWN = (0.4, -0.4)  # widen coverage to the left
    PAN_RIGHT_DOWN = (-0.4, -0.4)  # widen coverage to the right

    def __init__(self):
        super().__init__('head_sweep_node')

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory'
        )

        self.joint_names = ['head_1_joint', 'head_2_joint']

        # One full sweep cycle: down -> neutral -> (occasionally) side pans -> neutral
        self.sweep_period_sec = 2.5   # < 3.0s clear interval, leaves margin
        self.segment_duration_sec = 0.8

        self.cycle_count = 0
        self.side_pan_every_n = 3  # do a sideways pan every Nth cycle

        self.timer = self.create_timer(self.sweep_period_sec, self.sweep_callback)

        self.get_logger().info("Head Sweep Node started")

    def sweep_callback(self):
        if not self._action_client.server_is_ready():
            self.get_logger().warn("Head controller action server not ready")
            return

        self.cycle_count += 1
        do_side_pan = (self.cycle_count % self.side_pan_every_n == 0)

        if do_side_pan:
            waypoints = [
                self.TILT_DOWN,
                self.PAN_LEFT_DOWN,
                self.PAN_RIGHT_DOWN,
                self.NEUTRAL,
            ]
            self.get_logger().info("Head sweep: down + side pan (left/right)")
        else:
            waypoints = [
                self.TILT_DOWN,
                self.NEUTRAL,
            ]
            self.get_logger().info("Head sweep: down-tilt only")

        self.send_trajectory(waypoints)

    def send_trajectory(self, waypoints):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        points = []
        t = self.segment_duration_sec
        for pan, tilt in waypoints:
            pt = JointTrajectoryPoint()
            pt.positions = [pan, tilt]
            pt.velocities = [0.0, 0.0]
            pt.time_from_start = Duration(seconds=t).to_msg()
            points.append(pt)
            t += self.segment_duration_sec

        goal_msg.trajectory.points = points

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Head sweep goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info("Head sweep segment completed")


# ============================================================
# MAIN (RUN ALL NODES)
# ============================================================
def main():
    rclpy.init()

    tf_node = TFGoalPublisher()
    nav_node = NavGoalClient()
    head_node = HeadSweepNode()

    # Use a MultiThreadedExecutor so all nodes run properly
    executor = rclpy.executors.MultiThreadedExecutor()

    executor.add_node(tf_node)
    executor.add_node(nav_node)
    executor.add_node(head_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    tf_node.destroy_node()
    nav_node.destroy_node()
    head_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()