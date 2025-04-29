#! /usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def mqtt_bridge():
    return Node(
        package='mqtt_wheel_bridge',
        executable='mqtt_wheel_bridge',
        namespace="/",
        name='mqtt_wheel_bridge'
    )

def control_tower():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('control_tower_ros2'), "control_tower.launch.py")
        )
    )

def generate_launch_description():
    
    ld = LaunchDescription()
    ld.add_action(mqtt_bridge())
    ld.add_action(control_tower())
    return ld