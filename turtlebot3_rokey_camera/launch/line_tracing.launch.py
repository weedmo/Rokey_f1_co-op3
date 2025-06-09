#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('turtlebot3_rokey_camera')

    config_file_path = PathJoinSubstitution([
        pkg_share,
        'config',
        'birds_eye_points.yaml'
    ])

    image_preprocessor_param_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'image_preprocessor_params.yaml'
    ])

    sobel_param_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'sobel_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='turtlebot3_rokey_camera',
            executable='auto_birdseye_from_yaml',
            name='auto_birdseye_from_yaml',
            output='screen',
            parameters=[{
                'config_file_path': config_file_path
            }]
        ),
        Node(
            package='turtlebot3_rokey_camera',
            executable='image_preprocessor',  
            name='image_preprocessor',
            output='screen',
            parameters=[image_preprocessor_param_file]
        ),
        Node(
            package='turtlebot3_rokey_camera',
            executable='sobel_curve_direction', 
            name='sobel_curve_direction',
            output='screen',
            parameters=[sobel_param_file]
        ),
    ])
