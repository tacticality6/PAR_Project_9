from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='par_project_9',
            executable='aruco_detector.py',
            name='aruco_detector'
        ),
        Node(
            package='par_project_9',
            executable='pointer_detector.py',
            name='pointer_detector'
        )
    ])