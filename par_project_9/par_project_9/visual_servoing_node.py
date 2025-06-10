#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int32, Bool
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import numpy as np
import math
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class VisualServoingState:
    SEARCHING = 0
    APPROACHING = 1
    TOUCHING = 2
    RETREATING = 3

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('k_p_linear', 0.4),
                ('k_p_angular', 1.2),
                ('desired_distance', 0.15),  # 15cm for pointer touch
                ('max_linear_speed', 0.3),
                ('max_angular_speed', 0.8),
                ('approach_threshold', 1.0),  # Switch to visual servoing at 1m
                ('touch_timeout', 10.0)       # Max time to attempt touch
            ]
        )
        
        # State variables
        self.current_state = VisualServoingState.SEARCHING
        self.target_marker_id = None
        self.target_marker_pose = None
        self.last_marker_time = self.get_clock().now()
        self.touch_confirmed = False
        self.touch_start_time = None
        
        # TF buffer for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = "oak_rgb_camera_optical_frame"
        
        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.touch_status_pub = self.create_publisher(Bool, 'touch_attempt_status', 10)
        
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
        self.target_sub = self.create_subscription(
            Int32,
            'target_marker_id',
            self.target_callback,
            10
        )
        self.touch_sub = self.create_subscription(
            Bool,
            'marker_touched',
            self.touch_callback,
            10
        )
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Visual Servoing Node initialized")

    def camera_info_callback(self, msg):
        """Update camera parameters from calibration"""
        if not self.camera_matrix:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera parameters updated")

    def target_callback(self, msg):
        """Set new target marker for visual servoing"""
        self.target_marker_id = msg.data
        self.current_state = VisualServoingState.SEARCHING
        self.touch_confirmed = False
        self.get_logger().info(f"New target marker set: {self.target_marker_id}")

    def touch_callback(self, msg):
        """Handle touch confirmation from pointer detector"""
        if msg.data:
            self.touch_confirmed = True
            self.get_logger().info("Touch confirmed!")
            
            # Reset state after successful touch
            self.current_state = VisualServoingState.RETREATING
            retreat_cmd = Twist()
            retreat_cmd.linear.x = -0.2  # Back up slowly
            self.cmd_vel_pub.publish(retreat_cmd)
            
            # Return to search state after retreat
            self.create_timer(1.0, lambda: self.set_state(VisualServoingState.SEARCHING))

    def marker_callback(self, msg):
        """Process detected marker information"""
        # Only process if we have an active target
        if self.target_marker_id is None:
            return
            
        # Update last detection time
        self.last_marker_time = self.get_clock().now()
        
        # Transform marker position to camera frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
            
            # Convert to camera frame coordinates
            x = msg.point.x - transform.transform.translation.x
            y = msg.point.y - transform.transform.translation.y
            z = msg.point.z - transform.transform.translation.z
            
            self.target_marker_pose = (x, y, z)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF error: {e}")

    def set_state(self, new_state):
        """Transition to a new state"""
        self.current_state = new_state
        states = ["SEARCHING", "APPROACHING", "TOUCHING", "RETREATING"]
        self.get_logger().info(f"State changed to {states[new_state]}")

    def control_loop(self):
        """Main control loop for visual servoing"""
        # State machine implementation
        if self.current_state == VisualServoingState.SEARCHING:
            self.search_for_marker()
        elif self.current_state == VisualServoingState.APPROACHING:
            self.approach_marker()
        elif self.current_state == VisualServoingState.TOUCHING:
            self.execute_touch()
        elif self.current_state == VisualServoingState.RETREATING:
            self.retreat()

    def search_for_marker(self):
        """Search behavior when marker is not visible"""
        # Send navigation goal to last known position
        if self.target_marker_pose:
            self.send_navigation_goal()
        
        # Simple search pattern if no known position
        else:
            search_cmd = Twist()
            search_cmd.angular.z = 0.5  # Rotate in place
            self.cmd_vel_pub.publish(search_cmd)

    def send_navigation_goal(self):
        """Send goal to navigation system"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.target_marker_pose[0]
        goal_msg.pose.pose.position.y = self.target_marker_pose[1]
        goal_msg.pose.pose.orientation.w = 1.0  # Default orientation
        
        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info("Navigation goal sent")

    def approach_marker(self):
        """Navigate to marker using visual servoing"""
        if not self.target_marker_pose:
            return
            
        x, y, z = self.target_marker_pose
        distance = math.sqrt(x**2 + y**2 + z**2)
        
        # Get control parameters
        k_p_linear = self.get_parameter('k_p_linear').value
        k_p_angular = self.get_parameter('k_p_angular').value
        max_linear = self.get_parameter('max_linear_speed').value
        max_angular = self.get_parameter('max_angular_speed').value
        
        # Calculate errors
        angle_error = math.atan2(x, z)  # Horizontal angle to marker
        distance_error = distance - self.get_parameter('desired_distance').value
        
        # Create control command
        cmd = Twist()
        cmd.linear.x = k_p_linear * distance_error
        cmd.angular.z = -k_p_angular * angle_error  # Negative for proper correction
        
        # Apply speed limits
        cmd.linear.x = max(min(cmd.linear.x, max_linear), -max_linear)
        cmd.angular.z = max(min(cmd.angular.z, max_angular), -max_angular)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Switch to touching state when close
        if distance < self.get_parameter('approach_threshold').value:
            self.set_state(VisualServoingState.TOUCHING)
            self.touch_start_time = self.get_clock().now()

    def execute_touch(self):
        """Final approach and touch execution"""
        if not self.target_marker_pose:
            self.set_state(VisualServoingState.SEARCHING)
            return
            
        x, y, z = self.target_marker_pose
        distance = math.sqrt(x**2 + y**2 + z**2)
        
        # Check for timeout
        if (self.get_clock().now() - self.touch_start_time).nanoseconds > \
           self.get_parameter('touch_timeout').value * 1e9:
            self.get_logger().warn("Touch attempt timed out")
            self.set_state(VisualServoingState.SEARCHING)
            return
            
        # Continue visual servoing
        self.approach_marker()
        
        # Publish touch attempt status
        status_msg = Bool()
        status_msg.data = self.touch_confirmed
        self.touch_status_pub.publish(status_msg)

    def retreat(self):
        """Back away after successful touch"""
        retreat_cmd = Twist()
        retreat_cmd.linear.x = -0.2  # Back up slowly
        self.cmd_vel_pub.publish(retreat_cmd)

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