#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from par_project_9_interfaces.msg import MarkerPointStamped
from sensor_msgs.msg import LaserScan
from enum import Enum, auto
import numpy as np
import math

class State(Enum):
    SEARCHING = auto()
    APPROACHING = auto()
    STOPPED = auto()

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')
        
        # Parameters
        self.declare_parameters(namespace='',
            parameters=[
                ('stop_distance', 0.25),  # 25cm stopping distance
                ('base_speed', 0.15),
                ('angular_speed', 0.3),
                ('k_lin', 0.8),  # Linear gain from working code
                ('k_ang', 2.0),  # Angular gain from working code
                ('base_frame', 'base_link'),
                ('camera_frame', 'oak_rgb_camera_optical_frame')
            ])
        
        # State machine
        self.state = State.SEARCHING
        self.last_marker_time = None
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers/Subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        self.marker_sub = self.create_subscription(
            MarkerPointStamped, 'marker_position', self.marker_callback, 10)
        
        self.get_logger().info("🚀 Visual Servoing Node Ready")

    def marker_callback(self, msg):
        try:
            # Transform marker to base frame
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('base_frame').value,
                msg.header.frame_id,
                rclpy.time.Time())
                
            marker_in_base = do_transform_point(
                PointStamped(header=msg.header, point=msg.point),
                transform)
            
            x = marker_in_base.point.x  # Forward distance
            y = marker_in_base.point.y  # Lateral offset
            distance = math.sqrt(x**2 + y**2)
            
            if distance <= self.get_parameter('stop_distance').value:
                self.state = State.STOPPED
                self.stop_robot()
                self.get_logger().info(f"🛑 Stopped at {distance:.2f}m from marker")
            else:
                self.state = State.APPROACHING
                self.approach_marker(x, y, distance)
                
            self.last_marker_time = self.get_clock().now()
            
        except TransformException as e:
            self.get_logger().warn(f"Transform failed: {e}")
            self.state = State.SEARCHING
            self.search_for_marker()

    def approach_marker(self, x, y, distance):
        """Direct movement toward marker (simplified from working code)"""
        cmd = Twist()
        
        # Linear control (always move forward)
        cmd.linear.x = min(
            self.get_parameter('k_lin').value * distance,
            self.get_parameter('base_speed').value
        )
        
        # Angular control (center marker)
        cmd.angular.z = self.get_parameter('k_ang').value * y
        
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(
            f"→ Approaching: {cmd.linear.x:.2f}m/s | ↺ {cmd.angular.z:.2f}rad/s",
            throttle_duration_sec=0.5)

    def search_for_marker(self):
        """Rotate to find markers"""
        cmd = Twist()
        cmd.angular.z = self.get_parameter('angular_speed').value
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Full stop"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

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