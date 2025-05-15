#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = '~/team_f_01_ws/src/turtlebot3_rokey_camera/config/birds_eye_points.yaml'
    
    return LaunchDescription([
        Node(
            package='turtlebot3_rokey_camera',
            executable='auto_birdseye_from_yaml',
            name='auto_birdseye_from_yaml',
            output='screen',
            parameters=[{
                'config_file_path': config_path  
            }]
        ),
        # Node(
        #     package='turtlebot3_rokey_camera',
        #     executable='aruco_pose_publisher',
        #     name='aruco_pose_publisher',
        #     output='screen'
        # ),
        Node(
            package='turtlebot3_rokey_camera',
            executable='image_preprocessor',
            name='image_preprocessor',
            output='screen'
        ),
        Node(
            package='turtlebot3_rokey_camera',
            executable='sobel_curve_direction',
            name='sobel_curve_direction',
            output='screen'
        ),
    ])
