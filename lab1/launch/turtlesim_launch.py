from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='lab1',
            executable='publisher',
            name='lab1',
            prefix="gnome-terminal --"
        )
    ])
