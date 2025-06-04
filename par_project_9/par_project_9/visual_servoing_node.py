#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import CameraInfo
import numpy as np

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')
        
        # Parameters
        self.declare_parameter('k_p_linear', 0.5)
        self.declare_parameter('k_p_angular', 1.0)
        self.declare_parameter('desired_distance', 0.5)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.5)
        
        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_center_x = 320
        
        # Subscribers
        self.marker_sub = self.create_subscription(
            PointStamped, 
            'marker_position', 
            self.marker_callback, 
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/oak/rgb/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Visual Servoing Node initialized")

    def camera_info_callback(self, msg):
        """Update camera parameters from calibration"""
        if not self.camera_matrix:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.image_center_x = self.camera_matrix[0, 2]
            self.get_logger().info("Camera parameters updated")

    def marker_callback(self, msg):
        """Handle marker detections and compute control commands"""
        if not self.camera_matrix:
            self.get_logger().warn("Camera parameters not available yet")
            return
            
        # Get parameters
        k_p_linear = self.get_parameter('k_p_linear').value
        k_p_angular = self.get_parameter('k_p_angular').value
        desired_distance = self.get_parameter('desired_distance').value
        max_linear_speed = self.get_parameter('max_linear_speed').value
        max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Convert pixel coordinates to normalized coordinates
        x_normalized = (msg.point.x - self.image_center_x) / self.camera_matrix[0, 0]
        
        # Create control command
        cmd = Twist()
        
        # Angular control
        cmd.angular.z = -k_p_angular * x_normalized
        cmd.angular.z = np.clip(cmd.angular.z, -max_angular_speed, max_angular_speed)
        
        # Publish 
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().debug(f"Command published: angular={cmd.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()