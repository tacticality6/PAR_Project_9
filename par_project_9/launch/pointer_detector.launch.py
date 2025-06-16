from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # X: forward, Y: left, Z: up
    cam_x = 0.05   # 5cm forward from base_link center
    cam_y = 0.0    # centered horizontally
    cam_z = 0.15   # 15cm above base_link center

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_cam_tf_pub',
            arguments=[
                '--x', str(cam_x),
                '--y', str(cam_y),
                '--z', str(cam_z),
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'oak_rgb_camera_optical_frame'
            ],
        ),

        Node(
            package='par_project_9',
            executable='delivery_tracker',
            name='delivery_tracking_node'
        ),

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
            executable='visual_servoing',
            name='visual_servoing_node'
        )
    ])