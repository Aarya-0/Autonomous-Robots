# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Changed by Maik Knof for RWU :)

import os
from os import environ, pathsep
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, SetLaunchConfiguration, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.arg_utils import LaunchArgumentsBase
from dataclasses import dataclass
from launch_pal.robot_arguments import CommonArgs
from launch_ros.actions import Node
from tiago_description.launch_arguments import TiagoArgs
from launch_pal.actions import CheckPublicSim

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource

@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    base_type: DeclareLaunchArgument = TiagoArgs.base_type
    has_screen: DeclareLaunchArgument = TiagoArgs.has_screen
    arm_type: DeclareLaunchArgument = TiagoArgs.arm_type
    arm_motor_model: DeclareLaunchArgument = TiagoArgs.arm_motor_model
    end_effector: DeclareLaunchArgument = TiagoArgs.end_effector
    ft_sensor: DeclareLaunchArgument = TiagoArgs.ft_sensor
    wrist_model: DeclareLaunchArgument = TiagoArgs.wrist_model
    camera_model: DeclareLaunchArgument = TiagoArgs.camera_model
    laser_model: DeclareLaunchArgument = TiagoArgs.laser_model

    navigation: DeclareLaunchArgument = CommonArgs.navigation
    advanced_navigation: DeclareLaunchArgument = CommonArgs.advanced_navigation
    slam: DeclareLaunchArgument = CommonArgs.slam
    moveit: DeclareLaunchArgument = CommonArgs.moveit
    namespace: DeclareLaunchArgument = CommonArgs.namespace
    tuck_arm: DeclareLaunchArgument = CommonArgs.tuck_arm
    is_public_sim: DeclareLaunchArgument = CommonArgs.is_public_sim

    map_path: DeclareLaunchArgument = DeclareLaunchArgument(
        "map_path", default_value="", description="Path to map.yaml"
    )
    nav_config: DeclareLaunchArgument = DeclareLaunchArgument(
        "nav_config",
        default_value="",
        description="Absolute path to a Nav2 params YAML file",
    )
    x: DeclareLaunchArgument = DeclareLaunchArgument(
        "x", default_value="1.5", description="X position of the robot"
    )
    y: DeclareLaunchArgument = DeclareLaunchArgument(
        "y", default_value="-4.0", description="Y position of the robot"
    )
    yaw: DeclareLaunchArgument = DeclareLaunchArgument(
        "yaw", default_value="1.5707", description="Yaw of the robot"
    )


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):
    # Set use_sim_time to True
    set_sim_time = SetLaunchConfiguration("use_sim_time", "True")
    launch_description.add_action(set_sim_time)

    # Shows error if is_public_sim is not set to True when using public simulation
    public_sim_check = CheckPublicSim()
    launch_description.add_action(public_sim_check)

    robot_name = 'tiago'
    packages = ['tiago_description', 'pmb2_description',
                'pal_hey5_description', 'pal_gripper_description',
                'pal_robotiq_description', 'omni_base_description',
                'iki_object_models', 'pal_gazebo_worlds']

    model_path = get_model_paths(packages)
    launch_description.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))

    world_path = "/root/ros2_ws/src/prj-iki-ros2/knowledgebase/iki_object_models/iki_object_models/models/ar_ss26/arena.world"
    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory('gazebo_ros'),
            'launch'
        ),
        '/gzserver.launch.py'
    ]),
    launch_arguments={
        'world': world_path,
        'use_sim_time': LaunchConfiguration("use_sim_time"),
        'verbose': 'True',
        'extra_gazebo_args': '-s libgazebo_ros_factoryx.so -s libgazebo_ros_state.so'
        }.items(),
    )
    gazebo_client = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch'), '/gzclient.launch.py']),
            )
    
    launch_description.add_action(gazebo)
    launch_description.add_action(gazebo_client)
    #################################################################################################################

    navigation = include_scoped_launch_py_description(
        pkg_name='rwu_tiago_bringup',
        paths=['launch', 'navigation', 'tiago_nav_bringup.launch.py'],
        launch_arguments={
            "robot_name":  robot_name,
            "is_public_sim": launch_args.is_public_sim,
            "laser":  launch_args.laser_model,
            "base_type": launch_args.base_type,
            "map_path": launch_args.map_path,
            "nav_config": launch_args.nav_config,
            'slam': launch_args.slam,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
        condition=IfCondition(LaunchConfiguration('navigation')))

    launch_description.add_action(navigation)

    advanced_navigation = include_scoped_launch_py_description(
        pkg_name='tiago_advanced_2dnav',
        paths=['launch', 'tiago_advanced_nav_bringup.launch.py'],
        launch_arguments={
            "base_type": launch_args.base_type,
        },
        condition=IfCondition(LaunchConfiguration('advanced_navigation')))

    launch_description.add_action(advanced_navigation)

    move_group = include_scoped_launch_py_description(
        pkg_name='tiago_moveit_config',
        paths=['launch', 'move_group.launch.py'],
        launch_arguments={
            "robot_name": robot_name,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "namespace": launch_args.namespace,
            "base_type": launch_args.base_type,
            "arm_type": launch_args.arm_type,
            "end_effector": launch_args.end_effector,
            "ft_sensor": launch_args.ft_sensor
        },
        condition=IfCondition(LaunchConfiguration('moveit')))

    launch_description.add_action(move_group)

    robot_spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity", robot_name,
            '-x', LaunchConfiguration("x"), '-y', LaunchConfiguration("y"), '-Y', LaunchConfiguration("yaw"),
            'use_sim_time:=True' #todo use launch args
        ],
        output="screen",
    )

    launch_description.add_action(robot_spawn)

    tiago_bringup = include_scoped_launch_py_description(
        pkg_name='tiago_bringup', paths=['launch', 'tiago_bringup.launch.py'],
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "arm_type": launch_args.arm_type,
            "arm_motor_model": launch_args.arm_motor_model,
            "laser_model": launch_args.laser_model,
            "camera_model": launch_args.camera_model,
            "base_type": launch_args.base_type,
            "wrist_model": launch_args.wrist_model,
            "ft_sensor": launch_args.ft_sensor,
            "end_effector": launch_args.end_effector,
            "has_screen": launch_args.has_screen,
            "is_public_sim": launch_args.is_public_sim,
        }
    )

    launch_description.add_action(tiago_bringup)

    tuck_arm = Node(package='tiago_gazebo',
                    executable='tuck_arm.py',
                    emulate_tty=True,
                    output='both',
                    condition=IfCondition(LaunchConfiguration('tuck_arm')))

    launch_description.add_action(tuck_arm)


    torso_incrementer_server = Node(
        package="joy_teleop",
        executable="incrementer_server",
        name="incrementer",
        namespace="torso_controller",
    )

    launch_description.add_action(torso_incrementer_server)

    head_incrementer_server = Node(
        package="joy_teleop",
        executable="incrementer_server",
        name="incrementer",
        namespace="head_controller",
    )

    launch_description.add_action(head_incrementer_server)

    gripper_incrementer_server = Node(
        package="joy_teleop",
        executable="incrementer_server",
        name="incrementer",
        namespace="gripper_controller",
    )

    launch_description.add_action(gripper_incrementer_server)

    return


def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, "share")

        model_paths += model_path

    if 'GAZEBO_MODEL_PATH' in environ:
        model_paths += pathsep + environ['GAZEBO_MODEL_PATH']

    return model_paths
