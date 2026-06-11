#!/usr/bin/env python3

import math, random
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState


class SpawnAreaManager(Node):

    def __init__(self):
        super().__init__('spawn_area_manager')

        self.spawn_client = self.create_client(
            SpawnEntity,
            '/spawn_entity'
        )

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for spawn service...")

        self.robot_x = None
        self.robot_y = None

        self.spawn_tolerance = 0.3
        self.current_spawn_area = 1

        # Areas, die Spawn-Events auslösen
        self.spawn_areas = [
            {
                "name": "area_1",
                "number": 1,
                "x": 1.0,
                "y": 2.0,
                "spawned": False,
                "objects": [
                ]
            },
            {
                "name": "area_2",
                "number": 2,
                "x": -0.46,
                "y": 2.0,
                "spawned": False,
                "objects": [
                  {
                    "name": "cylinder_base",
                    "static": True,
                    "type": "cylinder",
                    "position": (-3.0, 2.0, 0.2),
                    "gravity": True,
                    "radius": 0.1,
                    "length": 0.01
                  },
                  {
                    "name": "cylinder_leg",
                    "static": True,
                    "type": "cylinder",
                    "position": (-3.0, 2.0, 0.2),
                    "gravity": True,
                    "radius": 0.02,
                    "length": 0.5
                  },
                  {
                    "name": "cylinder_plate",
                    "static": True,
                    "type": "cylinder",
                    "position": (-3.0, 2.0, 0.52),
                    "gravity": True,
                    "radius": 0.4,
                    "length": 0.02
                  }
                ]
            },
            {
                "name": "area_3",
                "number": 3,
                "x": -3.0,
                "y": -0.0, 
                "spawned": False,
                "objects": [
                  {
                    "name": "floating_obstacle_1",
                    "type": "floating",
                    "size": (0.3, 0.3, 0.6),
                    "position": (-3.2, -1.0, 0.7),
                    "gravity": True,
                    "static": False,
                    "waypoints": [
                        [-3.2, -1.0, 1.0],
                        [-0.5, -1.0, 1.0],
                        [-0.5, -3.0, 1.0],
                        [-3.2, -3.0, 1.0]
                    ],
                    "speed": 1.0
                  }
                ]
            },
            {
                "name": "area_4",
                "number": 4,
                "x": 0.0,
                "y": -2.0, 
                "spawned": False,
                "objects": [
                    {
                        "name": "dining_table_leg_1",
                        "static": True,
                        "type": "box",
                        "size": (0.1, 0.05, 0.76),
                        "position": (1.05, -1.9125, 0.2),
                        "gravity": True
                    },
                    {
                        "name": "dining_table_leg_2",
                        "static": True,
                        "type": "box",
                        "size": (0.1, 0.05, 0.76),
                        "position": (2.45, -1.9125, 0.2),
                        "gravity": True
                    },
                    {
                        "name": "dining_table_leg_3",
                        "static": True,
                        "type": "box",
                        "size": (0.1, 0.05, 0.76),
                        "position": (2.45, -2.5875, 0.2),
                        "gravity": True
                    },
                    {
                        "name": "dining_table_leg_4",
                        "static": True,
                        "type": "box",
                        "size": (0.1, 0.05, 0.76),
                        "position": (1.05, -2.5875, 0.2),
                        "gravity": True
                    },
                    {
                        "name": "dining_table_plate",
                        "static": True,
                        "type": "box",
                        "size": (1.5, 1.0, 0.025),
                        "position": (1.75, -2.27, 0.781),
                        "gravity": True
                    }
                ]
            },
            {
                "name": "area_5",
                "number": 5,
                "x": 1.0,
                "y": -4.6, 
                "spawned": False,
                "objects": [  
                    {
                        "name": "barrier",
                        "static": True,
                        "type": "box",
                        "size": (0.25, round(random.uniform(1.3, 0.8),2), 0.2),
                        "position": (0.0, -5.2, 0.6),
                        "gravity": False
                    }
                ]
            },
            {
                "name": "area_6",
                "number": 6,
                "x": -1.5,
                "y": -4.7, 
                "spawned": False,
                "objects": [
                ]
            },
            {
                "name": "area_7",
                "number": 7,
                "x": -3.0,
                "y": 0.0, 
                "spawned": False,
                "objects": [
                ]
            },
            {
                "name": "area_8",
                "number": 8,
                "x": 1.0,
                "y": 2.0, 
                "spawned": False,
                "objects": [
                ]
            }
        ]

        self.floating_obstacles = []

        self.model_state_sub = self.create_subscription(
            ModelStates,
            "/gazebo/model_states",
            self.model_states_callback,
            10
        )

        self.state_client = self.create_client(
            SetEntityState,
            "/gazebo/set_entity_state"
        )

        self.motion_timer = self.create_timer(
            0.05,
            self.update_floating_obstacles
        )

        self.timer = self.create_timer(
            0.5,
            self.check_spawn_areas
        )

    def model_states_callback(self, msg):
        try:
            idx = msg.name.index("tiago")

            self.robot_x = msg.pose[idx].position.x
            self.robot_y = msg.pose[idx].position.y

            # Debug
            self.get_logger().debug(
                f"TIAGO: x={self.robot_x:.2f}, y={self.robot_y:.2f}"
            )

        except ValueError:
            # tiago noch nicht vorhanden
            pass
    
    def update_floating_obstacles(self):
      dt = 0.05

      for obstacle in self.floating_obstacles:
        target = obstacle["waypoints"][obstacle["current_wp"]]

        dx = target[0] - obstacle["pos"][0]
        dy = target[1] - obstacle["pos"][1]
        dz = target[2] - obstacle["pos"][2]

        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist < 0.05:
          obstacle["current_wp"] = (
              obstacle["current_wp"] + 1
          ) % len(obstacle["waypoints"])
          continue

        step = min(
          obstacle["speed"] * dt,
          dist
        )

        obstacle["pos"][0] += dx / dist * step
        obstacle["pos"][1] += dy / dist * step
        obstacle["pos"][2] += dz / dist * step

        self.move_model(
          obstacle["name"],
          obstacle["pos"]
        )

    def move_model(self, model_name, pos):
      req = SetEntityState.Request()

      state = EntityState()
      state.name = model_name
      state.reference_frame = "world"

      state.pose.position.x = pos[0]
      state.pose.position.y = pos[1]
      state.pose.position.z = pos[2]
      state.pose.orientation.w = 1.0

      state.twist = Twist()
      req.state = state
      self.state_client.call_async(req)

    def check_spawn_areas(self):
        if self.robot_x is None:
          return
        
        self.get_logger().info(
          f"Robot @ ({self.robot_x:.2f}, {self.robot_y:.2f})"
        )

        for area in self.spawn_areas:
            if area["spawned"]:
              continue

            distance = math.sqrt(
              (self.robot_x - area["x"])**2 +
              (self.robot_y - area["y"])**2
            )

            if distance <= self.spawn_tolerance and area["number"] == self.current_spawn_area:
              self.get_logger().info(
                f"Entered spawn area {area['name']}"
              )

              for obj in area["objects"]:
                if obj.get("type") == "cylinder":
                  self.spawn_cylinder(obj)
                else:
                  self.spawn_box(obj)
                
                if obj.get("type") == "floating":
                  self.floating_obstacles.append({
                    "name": obj["name"],
                    "pos": list(obj["position"]),
                    "waypoints": obj["waypoints"],
                    "current_wp": 0,
                    "speed": obj["speed"]
                  })

              area["spawned"] = True
              self.current_spawn_area += 1

    def spawn_box(self, obj):
        gravity_str = "true" if obj["gravity"] else "false"
        static_obj = "<static>true</static>" if obj["static"] else ""
        px, py, pz = obj["position"]
        sx, sy, sz = obj["size"]

        pose = Pose()
        pose.position.x = px
        pose.position.y = py
        pose.position.z = pz
        pose.orientation.w = 1.0

        req = SpawnEntity.Request()

        req.name = obj["name"]
        req.xml = self.create_box_sdf(
            obj["name"],
            sx,
            sy,
            sz,
            gravity=gravity_str,
            static_obj=static_obj,  
            mass=0.5
        )
        req.initial_pose = pose

        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.spawn_callback)
    
    def spawn_cylinder(self, obj):
        gravity_str = "true" if obj["gravity"] else "false"
        static_obj = "<static>true</static>" if obj["static"] else ""
        px, py, pz = obj["position"]
        radius = obj['radius']
        length = obj['length']
      
        pose = Pose()
        pose.position.x = px
        pose.position.y = py
        pose.position.z = pz
        pose.orientation.w = 1.0

        req = SpawnEntity.Request()

        req.name = obj["name"]
        req.xml = self.create_cylinder_sdf(
            obj["name"],
            gravity=gravity_str,
            static_obj=static_obj,
            radius=radius,
            length=length
        )
        req.initial_pose = pose

        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):

        try:
            response = future.result()

            if response.success:
                self.get_logger().info("Object spawned")
            else:
                self.get_logger().error(
                    f"Spawn failed: {response.status_message}"
                )

        except Exception as e:
            self.get_logger().error(str(e))

    def create_box_sdf(
        self,
        model_name,
        x,
        y,
        z,
        gravity,
        static_obj,
        mass=0.5
    ):

        ixx = mass / 12.0 * (y*y + z*z)
        iyy = mass / 12.0 * (x*x + z*z)
        izz = mass / 12.0 * (x*x + y*y)

        return f"""
<sdf version="1.6">
  <model name="{model_name}">
    <link name="link">
      <gravity>{gravity}</gravity>
      {static_obj}

      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx>
          <iyy>{iyy}</iyy>
          <izz>{izz}</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box>
            <size>{x} {y} {z}</size>
          </box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box>
            <size>{x} {y} {z}</size>
          </box>
        </geometry>
      </visual>

    </link>
  </model>
</sdf>
"""


    def create_cylinder_sdf(
        self,
        model_name,
        gravity,
        static_obj,
        radius,
        length
    ):

        return f"""
<sdf version="1.6">
  <model name="{model_name}">
    <link name="link">
      <gravity>{gravity}</gravity>
      {static_obj}

      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
      </visual>

    </link>
  </model>
</sdf>
"""

def main(args=None):

    rclpy.init(args=args)

    node = SpawnAreaManager()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()