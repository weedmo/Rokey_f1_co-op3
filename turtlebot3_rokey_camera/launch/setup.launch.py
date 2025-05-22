#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            namespace='pi_camera',  
            name='v4l2_camera_node',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_size': [1280, 720]},
                {'pixel_format': 'YUYV'}
            ],
            output='screen'
        ),
        Node(
            package='turtlebot3_manipulation_bringup',
            executable='init_pose',
            name='init_pose',
            output='screen'
        ),
    ])
