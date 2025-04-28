from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bbot',
            executable='drive_motors',
            name='drive_motors_node',
            output='screen'
        ),
        Node(
            package='bbot',
            executable='conveyor',
            name='conveyor_node',
            output='screen'
        ),
        Node(
            package='bbot',
            executable='bucket_spin',
            name='bucket_spin_node',
            output='screen'
        ),
        Node(
            package='bbot',
            executable='arduino_driver',
            name='arduino_driver_node',
            output='screen'
        ),
        Node(
            package='bbot',
            executable='encoders',
            name='encoder_node',
            output='screen'
        ),
    ])
