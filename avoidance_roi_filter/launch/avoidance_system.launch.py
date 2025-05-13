from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='avoidance_roi_filter',
            executable='roi_filter',
            name='roi_filter_node',
            output='screen'
        ),
        Node(
            package='avoidance_roi_filter',
            executable='obstacle_stopper',
            name='obstacle_stopper_node',
            output='screen'
        ),
        Node(
            package='avoidance_roi_filter',
            executable='cmd_vel_publisher',
            name='cmd_vel_publisher_node',
            output='screen'
        ),
        Node(  
            package='avoidance_roi_filter',
            executable='plot_cmd_vel',
            name='plot_cmd_vel',
            output='screen'
        )
    ])
