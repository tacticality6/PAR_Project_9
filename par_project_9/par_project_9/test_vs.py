#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from par_project_9_interfaces.srv import MarkerConfirmation
from par_project_9_interfaces.msg import MarkerPointStamped
import message_filters
import numpy as np
import cv2

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('VisualServoingNode')

        self.bridge = CvBridge()
        self.marker_point = None

        # Parameters
        self.target_distance = 0.25  # 25 cm
        self.distance_tolerance = 0.02  # ±2 cm
        self.center_threshold_px = 20  # center margin in pixels
        self.linear_speed = 1.0  # m/s
        self.angular_speed = 0.1  # rad/s

        # Subscribers
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        self.cam_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/color/camera_info')
        self.marker_sub = self.create_subscription(MarkerPointStamped, '/marker_point', self.marker_callback, 10)

        # Synchronizer
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.cam_info_sub], 10, 0.1)
        self.sync.registerCallback(self.image_callback)

        # Velocity publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def marker_callback(self, msg):
        self.marker_point = msg.marker.point  # geometry_msgs/Point
        self.get_logger().info(f"Marker detected at pixel ({self.marker_point.x:.1f}, {self.marker_point.y:.1f})")

    def image_callback(self, rgb_msg, depth_msg, cam_info_msg):
        if self.marker_point is None:
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")
            return

        x, y = int(self.marker_point.x), int(self.marker_point.y)
        if y >= depth_image.shape[0] or x >= depth_image.shape[1]:
            self.get_logger().warn("Marker outside image bounds.")
            return

        # Get depth at pixel (in meters)
        depth = depth_image[y, x] / 1000.0
        if np.isnan(depth) or depth <= 0.01:
            self.get_logger().warn("Invalid or missing depth.")
            return

        # Camera center
        cx = cam_info_msg.k[2]
        cy = cam_info_msg.k[5]
        dx = x - cx  # horizontal offset
        dy = y - cy  # vertical offset

        cmd = Twist()

        # Horizontal (left/right): +dx means marker is to the right of center
        if abs(dx) > self.center_threshold_px:
            cmd.linear.x = -self.linear_speed if dx > 0 else self.linear_speed
        else:
            cmd.linear.x = 0.0

        # Vertical (forward/backward in camera frame = linear.y)
        if depth > self.target_distance + self.distance_tolerance:
            cmd.linear.y = self.linear_speed
        elif depth < self.target_distance - self.distance_tolerance:
            cmd.linear.y = -self.linear_speed
        else:
            cmd.linear.y = 0.0

        # Optional: Rotate if marker is far from center (e.g., ±100 px)
        if abs(dx) > 100:
            cmd.angular.z = -self.angular_speed if dx > 0 else self.angular_speed
        else:
            cmd.angular.z = 0.0

        # Check if all conditions met
        if abs(dx) <= self.center_threshold_px and abs(depth - self.target_distance) <= self.distance_tolerance:
            self.get_logger().info("✅ Marker aligned and at correct distance. Stopping.")
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
