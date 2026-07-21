# Wrapper around rwu_tiago_bringup's sim_ar_ss26.launch.py.
# Includes the original, unmodified launch file and adds a
# pointcloud_to_laserscan_node on top, which converts the head depth
# camera's pointcloud into a height-filtered virtual 2D scan
# (/head_camera_ground_scan) for permanent ground-obstacle marking
# in the global costmap's ground_memory_layer.
#
# Usage (identical args to the original, run directly by path):
#   ros2 launch /path/to/sim_ar_ss26_ground_scan.launch.py \
#       is_public_sim:=True x:=1.0 y:=2.0 yaw:=3.11 \
#       navigation:=True slam:=False \
#       map_path:=/root/maps/map_ar_ss26.yaml \
#       nav_config:=/root/config/tiago_custom_fullrunv6.yaml

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Re-declare only the args you actually pass on the command line.
    # Anything not listed here (base_type, arm_type, etc.) falls back
    # to the defaults already declared inside sim_ar_ss26.launch.py.
    is_public_sim_arg = DeclareLaunchArgument(
        "is_public_sim", default_value="True", description="Use public simulation"
    )
    x_arg = DeclareLaunchArgument("x", default_value="1.0", description="X position")
    y_arg = DeclareLaunchArgument("y", default_value="2.0", description="Y position")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="3.11", description="Yaw")
    navigation_arg = DeclareLaunchArgument(
        "navigation", default_value="True", description="Launch navigation stack"
    )
    slam_arg = DeclareLaunchArgument(
        "slam", default_value="False", description="Launch SLAM instead of localization"
    )
    map_path_arg = DeclareLaunchArgument(
        "map_path", default_value="", description="Path to map.yaml"
    )
    nav_config_arg = DeclareLaunchArgument(
        "nav_config", default_value="", description="Absolute path to a Nav2 params YAML file"
    )

    # Path to the original, unmodified sim_ar_ss26.launch.py
    original_launch_path = os.path.join(
        get_package_share_directory('rwu_tiago_bringup'),
        'launch', 'sim', 'sim_ar_ss26.launch.py'
    )

    original_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(original_launch_path),
        launch_arguments={
            'is_public_sim': LaunchConfiguration('is_public_sim'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
            'navigation': LaunchConfiguration('navigation'),
            'slam': LaunchConfiguration('slam'),
            'map_path': LaunchConfiguration('map_path'),
            'nav_config': LaunchConfiguration('nav_config'),
        }.items(),
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        parameters=[LaunchConfiguration('nav_config')],
        remappings=[
            ('cloud_in', '/head_front_camera/depth/points'),
            ('scan', '/head_camera_ground_scan'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('navigation')),
    )

    return LaunchDescription([
        is_public_sim_arg,
        x_arg,
        y_arg,
        yaw_arg,
        navigation_arg,
        slam_arg,
        map_path_arg,
        nav_config_arg,
        original_sim,
        pointcloud_to_laserscan,
    ])