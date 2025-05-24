from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="aruco_detector",
            executable="detector_node",
            name="aruco_detector",
            parameters=[{
                "image_topic": "/oak/rgb/image_raw",
                "info_topic":  "/oak/rgb/camera_info",
                "marker_size_m": 0.06,
                "aruco_dict":  "DICT_4X4_50",
                "publish_tf":  True
            }]
        )
    ])
