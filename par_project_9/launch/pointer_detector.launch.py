from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='par_project_9',
            executable='aruco_detector',
            name='aruco_detector'
        ),
        Node(
            package='par_project_9',
            executable='pointer_detector',
            name='pointer_detector'
        ),
        Node(
            package='par_project_9',
            executable='visual_servoing_node',
            name='visual_servoing_node'
        )
    ])