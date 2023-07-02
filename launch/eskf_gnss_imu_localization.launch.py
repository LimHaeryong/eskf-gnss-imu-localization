#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    play_rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '/dataset/utbm_robocar_dataset_20190131_noimage']
    )
    nmea_navsat_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_topic_driver',
        name='nmea_topic_driver',
        arguments=['--ros-args --remap useRMC:=True']
    )
    
    test_exec_node = Node(
        package='eskf_gnss_imu_localization',
        executable='eskf_gnss_imu_localization_node',
        name='eskf_gnss_imu_localization_node',
        prefix=['stdbuf -o L'],
        output='screen'
    )
    
    launch_description = LaunchDescription()
    launch_description.add_action(nmea_navsat_node)
    launch_description.add_action(play_rosbag)
    launch_description.add_action(test_exec_node)
    return launch_description