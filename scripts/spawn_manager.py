#!/usr/bin/env python3

"""
Start script: python3 spawn_manager.py
Example for start on area 3: python3 spawn_manager.py 3
"""

import math, random, time
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates, EntityState
from gazebo_msgs.srv import SetEntityState, SpawnEntity, DeleteEntity
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class SpawnAreaManager(Node):
  def __init__(self, start_area:int=1, single_area:bool=False):
    super().__init__('spawn_area_manager')

    self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
    while not self.spawn_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info("Waiting for spawn service...")

    self.robot_x = None
    self.robot_y = None

    self.spawn_tolerance = 0.3
    self.current_spawn_area = start_area
    self.single_area = single_area
    self.spawned_objects = {1:[],2:[],3:[],4:[],5:[],6:[],7:[],8:[]}

    # Areas, die Spawn-Events auslösen
    self.spawn_areas = [
      {
        "name": "area_1",
        "number": 1,
        "single": {
          "x": 1.0,
          "y": 2.0
        },
        "whole": {
          "x": 1.0,
          "y": 2.0
        },
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
        "name": "area_2",
        "number": 2,
        "single": {
          "x": -3.0,
          "y": 1.0
        },
        "whole": {
          "x": 0.0,
          "y": 2.0
        },
        "spawned": False,
        "objects": [
          {
            "name": "small_box_1",
            "static": True,
            "type": "box",
            "size": (0.20, 0.20, 0.09),
            "position": (-1.05, -2.49, 0.2),
            "gravity": True
          },
          {
            "name": "small_box_2",
            "static": True,
            "type": "box",
            "size": (0.20, 0.20, 0.09),
            "position": (-1.25, -2.08, 0.2),
            "gravity": True
          },
          {
            "name": "small_box_3",
            "static": True,
            "type": "box",
            "size": (0.20, 0.20, 0.09),
            "position": (-1.61, -2.22, 0.2),
            "gravity": True
          },
          {
            "name": "small_box_4",
            "static": True,
            "type": "box",
            "size": (0.20, 0.20, 0.09),
            "position": (-2.55, -3.59, 0.2),
            "gravity": True
          },
          {
            "name": "small_box_5",
            "static": True,
            "type": "box",
            "size": (0.20, 0.20, 0.09),
            "position": (-3.4, -2.32, 0.2),
            "gravity": True
          },
          {
            "name": "small_box_6",
            "static": True,
            "type": "box",
            "size": (0.20, 0.20, 0.09),
            "position": (-3.26, -1.81, 0.2),
            "gravity": True
          },
          {
            "name": "small_box_7",
            "static": True,
            "type": "box",
            "size": (0.20, 0.20, 0.09),
            "position": (-3.08, -1.61, 0.2),
            "gravity": True
          },
          {
            "name": "small_box_8",
            "static": True,
            "type": "box",
            "size": (0.20, 0.20, 0.09),
            "position": (-2.81, -1.43, 0.2),
            "gravity": True
          },
          {
            "name": "small_box_9",
            "static": True,
            "type": "box",
            "size": (0.20, 0.20, 0.09),
            "position": (-2.81, -1.13, 0.2),
            "gravity": True
          },
          {
            "name": "small_box_10",
            "static": True,
            "type": "box",
            "size": (0.31, 1.0, 0.477),
            "position": (-2.46, -0.48, 0.2),
            "gravity": True
          },
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
        "name": "area_3",
        "number": 3,
        "single": {
          "x": 1.0,
          "y": -4.0
        },
        "whole": {
          "x": 0.0,
          "y": -2.0
        },
        "spawned": False,
        "objects": [
          {
            "name": "tight_opening_1",
            "static": True,
            "type": "box",
            "size": (0.3, 1.926, 1.0),
            "position": (0.423, -5.52, 0.49),
            "gravity": True
          },
          {
            "name": "tight_opening_2",
            "static": True,
            "type": "box",
            "size": (0.3, 1.0, 1.0),
            "position": (1.57, -5.03, 0.49),
            "gravity": True
          },
          {
            "name": "barrier",
            "static": True,
            "type": "box",
            "size": (2.01, 0.1, 0.1),
            "position": (-0.92, -5.94, 0.61),
            "gravity": False
          },
          {
            "name": "help_localization_small_1",
            "static": True,
            "type": "cylinder",
            "position": (1.4457, -7.9385, 0.415),
            "gravity": True,
            "radius": 0.24,
            "length": 0.83
          },
          {
            "name": "help_localization_1",
            "static": True,
            "type": "cylinder",
            "position": (-4.6664, -6.4318, 0.49999),
            "gravity": True,
            "radius": 0.39,
            "length": 1.00
          },
          {
            "name": "help_localization_2",
            "static": True,
            "type": "cylinder",
            "position": (2.36, -7.8227, 0.49999),
            "gravity": True,
            "radius": 0.39,
            "length": 1.00
          }
        ]
      },
      {
        "name": "area_4",
        "number": 4,
        "single": {
          "x": -1.5,
          "y": -5.0
        },
        "whole": {
          "x": 1.0,
          "y": -4.6
        },
        "spawned": False,
        "objects": [  
          {
            "name": "floating_obstacle_1",
            "type": "floating",
            "size": (0.3, 0.3, 1.0),
            "position": (-2.5, -3.5, 0.7),
            "gravity": True,
            "static": False,
            "waypoints": [
              [-2.5, -3.5, 0.7],
              [-1.0, -1.0, 0.7]
            ],
            "speed": 0.1
          },
          {
            "name": "floating_obstacle_2",
            "type": "floating",
            "size": (0.3, 0.3, 1.0),
            "position": (-4.0, -2.0, 0.7),
            "gravity": True,
            "static": False,
            "waypoints": [
              [-4.0, -2.0, 0.7],
              [-2.0, -1.0, 0.7]
            ],
            "speed": 0.1
          }
        ]
      },
      {
        "name": "area_5",
        "number": 5,
        "single": {
          "x": -1.2,
          "y": 2.0, 
        },
        "whole": {
          "x": -1.2,
          "y": 2.0, 
        },
        "spawned": False,
        "objects": [
          {
            "name": "floating_door_1",
            "type": "floating",
            "size": (0.2, 1.0, 1.0),
            "position": (-0.3, 2.0, 0.2),
            "gravity": True,
            "static": False,
            "waypoints": [
                [-0.3, 2.0, 0.2],
                [-0.3, 4.0, 0.2]
            ],
            "speed": 0.2
          }
        ]
      }
    ]

    self.motion_group = MutuallyExclusiveCallbackGroup()
    self.spawn_group = MutuallyExclusiveCallbackGroup()

    self.floating_obstacles = []
    self.model_state_sub = self.create_subscription(
      ModelStates,
      "/gazebo/model_states",
      self.model_states_callback,
      10,
      callback_group=self.motion_group
    )

    self.state_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
    self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
    self.motion_timer = self.create_timer(0.05, self.update_floating_obstacles, callback_group=self.motion_group)
    self.timer = self.create_timer(0.5, self.check_spawn_areas, callback_group=self.spawn_group)

  def model_states_callback(self, msg):
    try:
      idx = msg.name.index("tiago")
      self.robot_x = msg.pose[idx].position.x
      self.robot_y = msg.pose[idx].position.y
      # self.get_logger().debug(f"TIAGO: x={self.robot_x:.2f}, y={self.robot_y:.2f}")
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

      step = min(obstacle["speed"] * dt, dist)

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
    
    # self.get_logger().info(f"Robot position: x= {self.robot_x:.2f}, y= {self.robot_y:.2f}")
    for area in self.spawn_areas:
      if area["spawned"]:
        continue
      
      if self.single_area:
        distance = math.sqrt(
          (self.robot_x - area["single"]["x"])**2 +
          (self.robot_y - area["single"]["y"])**2
        )
      else:
        distance = math.sqrt(
          (self.robot_x - area["whole"]["x"])**2 +
          (self.robot_y - area["whole"]["y"])**2
        )

      if distance <= self.spawn_tolerance and area["number"] == self.current_spawn_area:
        self.get_logger().info(f"Entered spawn area {area['name']}")

        for obj in area["objects"]:
          if obj.get("type") == "cylinder":
            self.spawn_cylinder(obj)
          else:
            self.spawn_box(obj)
          time.sleep(0.1)
          if obj.get("type") == "floating":
            self.floating_obstacles.append({
              "name": obj["name"],
              "pos": list(obj["position"]),
              "waypoints": obj["waypoints"],
              "current_wp": 0,
              "speed": obj["speed"]
            })
          
          self.spawned_objects[self.current_spawn_area].append(obj.get("name"))
          time.sleep(0.1)
        area["spawned"] = True

        if self.current_spawn_area - 3 >= 1:
          delete_key = self.current_spawn_area - 3
          self.delete_objects(self.spawned_objects[delete_key])

        self.current_spawn_area += 1

  def spawn_box(self, obj):
    self.get_logger().info(f"spawn Obj: {obj['name']}")
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
        sx, sy, sz,
        gravity=gravity_str,
        static_obj=static_obj,  
        mass=0.5
    )
    req.initial_pose = pose

    future = self.spawn_client.call_async(req)
    future.add_done_callback(self.spawn_callback)
    
  def spawn_cylinder(self, obj):
    self.get_logger().info(f"spawn Obj: {obj['name']}")
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
          self.get_logger().error(f"Spawn failed: {response.status_message}")
    except Exception as e:
        self.get_logger().error(str(e))

  def create_box_sdf(self, model_name, x, y, z, gravity, static_obj, mass=0.5):
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
        <material>
          <ambient>1 0.65 0 1</ambient>
          <diffuse>1 0.65 0 1</diffuse>
        </material>
      </visual>

    </link>
  </model>
</sdf>
"""

  def create_cylinder_sdf(self, model_name, gravity, static_obj, radius, length):
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
        <material>
          <ambient>1 0.65 0 1</ambient>
          <diffuse>1 0.65 0 1</diffuse>
        </material>
      </visual>

    </link>
  </model>
</sdf>
"""

  def delete_objects(self, names):
    futures = []
    for name in names:
      request = DeleteEntity.Request()
      request.name = name
      self.get_logger().info(f"Sending delete request for '{name}'")
      future = self.delete_cli.call_async(request)
      futures.append((name, future))

def main(args=None):
  rclpy.init(args=args)
  start_area = 1
  single_area = False
  if len(sys.argv) > 1:
    try:
      start_area = int(sys.argv[1])
      single_area = True
    except ValueError:
      print("Usage: python3 spawn_manager.py <start_area>")
      print("Switch to default start area = 1")
      start_area = 1
      single_area = False

  node = SpawnAreaManager(start_area=start_area, single_area=single_area)
  executor = MultiThreadedExecutor(num_threads=4)
  executor.add_node(node)
  try:
      executor.spin()
  finally:
      node.destroy_node()
      rclpy.shutdown()

if __name__ == "__main__":
  main()