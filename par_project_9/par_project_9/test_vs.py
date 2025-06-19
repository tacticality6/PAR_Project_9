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
    INITIALIZING = auto()
    SEARCHING = auto()
    APPROACHING = auto()
    TOUCHED = auto()
    TURNING = auto()
    RECOVERY = auto()
    ERROR = auto()

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')
        
        # Parameters
        self.declare_parameters(namespace='',
            parameters=[
                ('stop_distance', 0.25),
                ('linear_speed', 0.2),  # Increased default speed
                ('angular_speed', 0.4),  # Reduced for smoother turns
                ('turn_duration', 2.0),
                ('marker_timeout', 1.0),
                ('recovery_duration', 3.0),
                ('safety_distance', 0.15),
                ('base_frame', 'base_link'),
                ('odom_frame', 'odom'),
                ('camera_frame', 'oak_rgb_camera_optical_frame'),
                ('tf_timeout_sec', 5.0),
                ('tf_retry_interval', 1.0),
                ('approach_angle_threshold', 30.0)  # Degrees for front cone
            ])
        
        # State machine
        self.state = State.INITIALIZING
        self.last_marker_time = None
        self.last_obstacle_distance = float('inf')
        self.tf_ready = False
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timers
        self.control_timer = self.create_timer(0.1, self.update)
        self.health_check_timer = self.create_timer(
            self.get_parameter('tf_retry_interval').value,
            self.check_tf_health
        )
        
        # Emergency stop flag
        self.emergency_stop = False
        
        self.get_logger().info("🚀 Visual Servoing Node Initializing...")

    def check_tf_health(self):
        """Verify critical TF frames are available"""
        if self.tf_ready and self.state != State.ERROR:
            return
            
        try:
            # Check base_link → camera_frame
            self.tf_buffer.lookup_transform(
                self.get_parameter('base_frame').value,
                self.get_parameter('camera_frame').value,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(
                    seconds=self.get_parameter('tf_timeout_sec').value)
            )
            
            self.tf_ready = True
            self.state = State.SEARCHING
            self.initialize_components()
            self.get_logger().info("✅ TF frames verified - Ready to operate")
                
        except TransformException as e:
            self.get_logger().error(f"❌ TF Health Check Failed: {str(e)}")
            self.state = State.ERROR
            self.stop_robot()

    def initialize_components(self):
        """Initialize ROS components after TF validation"""
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        # Subscribers
        self.marker_sub = self.create_subscription(
            MarkerPointStamped, 'marker_position', self.marker_callback, 10)
        
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, QoSProfile(
                depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def laser_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        if not self.tf_ready:
            return
            
        valid_ranges = [r for r in msg.ranges if r > 0.05 and not np.isinf(r)]
        if valid_ranges:
            self.last_obstacle_distance = min(valid_ranges[:30] + valid_ranges[-30:])
            
            if self.last_obstacle_distance < self.get_parameter('safety_distance').value:
                self.emergency_stop = True
                self.stop_robot()
                self.get_logger().warn("🛑 EMERGENCY STOP: Obstacle at {:.2f}m".format(
                    self.last_obstacle_distance))

    def marker_callback(self, msg):
        if not self.tf_ready or self.state in [State.ERROR, State.INITIALIZING]:
            return
            
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('base_frame').value,
                msg.header.frame_id,
                rclpy.time.Time())
                
            marker_in_base = do_transform_point(
                PointStamped(header=msg.header, point=msg.point),
                transform)
                
            distance = math.sqrt(marker_in_base.point.x**2 + marker_in_base.point.y**2)
            
            if distance > self.get_parameter('stop_distance').value:
                self.state = State.APPROACHING
                self.approach_marker(marker_in_base)
            else:
                self.state = State.TOUCHED
                self.stop_robot()
                self.get_logger().info("🎯 Marker Touched at {:.2f}m".format(distance))
            
            self.last_marker_time = self.get_clock().now()
            
        except TransformException as e:
            self.get_logger().error(f"🔧 Transform Error: {str(e)}")
            self.state = State.RECOVERY
            self.recovery_move()

    def approach_marker(self, marker):
        """Guaranteed forward motion with smooth steering"""
        if self.emergency_stop:
            return
            
        cmd = Twist()
        x = marker.point.x  # Forward distance
        y = marker.point.y  # Lateral offset
        distance = math.sqrt(x**2 + y**2)
        angle = math.degrees(math.atan2(y, x))
        
        angle_threshold = self.get_parameter('approach_angle_threshold').value
        
        if abs(angle) > angle_threshold:
            # Rotate to center marker
            cmd.angular.z = 0.3 if angle > 0 else -0.3
            self.get_logger().info(
                "↻ Aligning to marker (angle: {:.1f}°)".format(angle),
                throttle_duration_sec=1.0)
        else:
            # Move forward with steering
            forward_speed = min(
                0.1 + 0.15 * (distance / 2.0),  # Dynamic speed scaling
                self.get_parameter('linear_speed').value
            )
            cmd.linear.x = forward_speed
            
            # Steering proportional to lateral error
            steering = -0.5 * y  # Negative because camera y-axis is inverted
            cmd.angular.z = np.clip(
                steering,
                -self.get_parameter('angular_speed').value,
                self.get_parameter('angular_speed').value
            )
            
            self.get_logger().info(
                "↑ Moving forward: {:.2f}m/s | ↺ Steering: {:.2f}rad/s".format(
                    forward_speed, cmd.angular.z),
                throttle_duration_sec=0.5)
        
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def search_for_marker(self):
        if self.emergency_stop:
            return
            
        cmd = Twist()
        cmd.angular.z = self.get_parameter('angular_speed').value
        self.cmd_vel_pub.publish(cmd)

    def recovery_move(self):
        if self.emergency_stop:
            return
            
        cmd = Twist()
        cmd.linear.x = 0.1
        self.cmd_vel_pub.publish(cmd)
        self.recovery_end_time = self.get_clock().now() + rclpy.time.Duration(
            seconds=self.get_parameter('recovery_duration').value)

    def update(self):
        if not self.tf_ready or self.state == State.ERROR:
            return
            
        now = self.get_clock().now()
        
        # Reset emergency stop if obstacle cleared
        if (self.emergency_stop and 
            self.last_obstacle_distance > self.get_parameter('safety_distance').value * 1.5):
            self.emergency_stop = False
            self.state = State.SEARCHING
            self.get_logger().info("🚦 Emergency Clear - Resuming Operation")
        
        # State transitions
        if self.state == State.APPROACHING:
            if (now - self.last_marker_time).nanoseconds > self.get_parameter('marker_timeout').value * 1e9:
                self.get_logger().info("🔍 Marker Lost - Entering Recovery")
                self.state = State.RECOVERY
                self.recovery_move()
        
        elif self.state == State.TOUCHED:
            self.state = State.TURNING
            self.turn_start_time = now
            self.search_for_marker()
        
        elif self.state == State.TURNING:
            if (now - self.turn_start_time).nanoseconds > self.get_parameter('turn_duration').value * 1e9:
                self.state = State.SEARCHING
                self.stop_robot()
        
        elif self.state == State.RECOVERY:
            if now >= self.recovery_end_time:
                self.state = State.SEARCHING
                self.stop_robot()

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