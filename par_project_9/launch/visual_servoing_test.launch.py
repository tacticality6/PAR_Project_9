from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_cam_tf_pub',
            arguments=['--x', '0.05', '--y', '0.0', '--z', '0.15', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'oak_rgb_camera_optical_frame'],
        ),

        Node(
            package='par_project_9',
            executable='delivery_tracker',
            name='delivery_tracking_node'
        ),

        #launch visual servoing node
        Node(
            package='par_project_9',
            executable='visual_servoing',
            name="visual_servoing_node",
            output='screen',
            parameters=[
                {"touched_marker_service_name": "touched_marker"},
                {"touched_distance_tolerance": 0.23},
                {"base_velocity": 0.25},
                {"colour_image_topic": "/oak/rgb/image_raw/compressed"},
                {"depth_image_topic": "/oak/rgb/image_raw/compressedDepth"},
                {"info_topic": "/oak/rgb/camera_info"},
                {"relocalise_pointer_freq": 5.0},
                {"debug_mode": True},
            ],
        ),

        #launch aruco marker detector
        Node(
            package="par_project_9",
            executable="aruco_detector",
            name="aruco_detector",
            parameters=[
                {"image_topic": "/oak/rgb/image_raw/compressed"}, # compressed!
                {"info_topic":  "/oak/rgb/camera_info"},
                {"aruco_dict":  "DICT_4X4_50"},
                {"marker_size_m": 0.06},
                {"publish_tf":  True},
            ],
            output='screen',
        ), 
    ])