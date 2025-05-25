#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ArUco node
    # oak_rgb_camera_optical_frame
    aruco_detector_node = Node(
        package="par_project_9",
        executable="aruco_detector",
        name="aruco_detector",
        parameters=[{
            "image_topic": "/oak/rgb/image_raw/compressed", # compressed!
            "info_topic":  "/oak/rgb/camera_info",
            "aruco_dict":  "DICT_4X4_50", # ← matched family
            "marker_size_m": 0.06,
            "publish_tf":  True
        }]
        output='screen',
    )

    # 3) assemble and return the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(aruco_detector_node)
    return ld


if __name__ == '__main__':
    generate_launch_description()
