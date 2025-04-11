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
            executable='bucket_servos',
            name='bucket_servos_node',
            output='screen'
        ),
        Node(
            package='bbot',
            executable='camera',
            name='camera_node',
            output='screen'
        ),
    ])
