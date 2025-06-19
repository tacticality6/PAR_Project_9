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

# Fallback PID implementation if simple_pid not available
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self._last_error = 0
        self._integral = 0
    
    def __call__(self, measurement):
        error = self.setpoint - measurement
        self._integral += error
        derivative = error - self._last_error
        self._last_error = error
        return self.Kp * error + self.Ki * self._integral + self.Kd * derivative

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
        
        # Parameters (with OAK camera frame as default)
        self.declare_parameters(namespace='',
            parameters=[
                ('stop_distance', 0.25),
                ('linear_speed', 0.15),
                ('angular_speed', 0.5),
                ('turn_duration', 2.0),
                ('marker_timeout', 1.0),
                ('recovery_duration', 3.0),
                ('safety_distance', 0.15),
                ('base_frame', 'base_link'),
                ('odom_frame', 'odom'),
                ('camera_frame', 'oak_rgb_camera_optical_frame'),  # Updated for OAK
                ('tf_timeout_sec', 5.0),
                ('tf_retry_interval', 1.0)
            ])
        
        # PID Controllers (using built-in implementation)
        self.linear_pid = PID(0.5, 0.01, 0.05, setpoint=0)
        self.angular_pid = PID(1.0, 0.01, 0.1, setpoint=0)
        
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
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('base_frame').value,
                self.get_parameter('camera_frame').value,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(
                    seconds=self.get_parameter('tf_timeout_sec').value)
            )
            
            self.get_logger().info(
                f"✅ TF Frames Verified:\n"
                f"Translation: {transform.transform.translation}\n"
                f"Rotation: {transform.transform.rotation}",
                throttle_duration_sec=5.0)
            
            self.tf_ready = True
            self.state = State.SEARCHING
            self.initialize_components()
                
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
        
        self.get_logger().info("🔄 Components Initialized - Ready for Operation")

    def laser_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        if not self.tf_ready:
            return
            
        valid_ranges = [r for r in msg.ranges if r > 0.05 and not np.isinf(r)]  # Filter noise
        if valid_ranges:
            self.last_obstacle_distance = min(valid_ranges[:30] + valid_ranges[-30:])
            
            # Emergency stop if too close
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
                
            distance = np.sqrt(marker_in_base.point.x**2 + marker_in_base.point.y**2)
            
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

    # def approach_marker(self, marker):
    #     """PID-controlled approach to marker (FIXED VERSION)"""
    #     if self.emergency_stop:
    #         return
        
    #     cmd = Twist()
    
    #     # Get marker position in base frame
    #     x = marker.point.x  # Forward/backward relative to robot
    #     y = marker.point.y  # Left/right relative to robot
    
    #     # Calculate distance and angle to marker
    #     distance = np.sqrt(x**2 + y**2)
    #     angle = np.arctan2(y, x)  # Angle relative to robot's forward
    
    #     # Only move forward if marker is in front (±60 degrees)
    #     if abs(angle) < np.radians(60):  # 60 degree forward cone
    #         # Linear control (always move forward toward marker)
    #         self.linear_pid.setpoint = self.get_parameter('stop_distance').value
    #         cmd.linear.x = np.clip(
    #             self.linear_pid(distance),
    #             0.0,  # Minimum speed (always forward)
    #             self.get_parameter('linear_speed').value
    #         )
        
    #         # Angular control (center marker)
    #         self.angular_pid.setpoint = 0
    #         cmd.angular.z = np.clip(
    #             self.angular_pid(y),  # Use y-error for steering
    #             -self.get_parameter('angular_speed').value,
    #             self.get_parameter('angular_speed').value
    #         )
    #     else:
    #         # Rotate to face marker first
    #         cmd.angular.z = 0.3 if y > 0 else -0.3
        
    #     self.get_logger().info("Aligning to marker...", throttle_duration_sec=1.0)
    
    #     self.cmd_vel_pub.publish(cmd)

    def approach_marker(self, marker):
        """PID-controlled approach with balanced forward/angular movement"""
        if self.emergency_stop:
            return
        
        cmd = Twist()
    
        # Get marker position in base frame
        x = marker.point.x  # Forward distance (always positive)
        y = marker.point.y  # Lateral offset
    
        # Calculate absolute distance
        distance = np.sqrt(x**2 + y**2)
    
        # Only proceed if marker is in front (x > 0)
        if x > 0:
            # 1. Forward motion (scaled by distance and x-component)
            forward_speed = min(
                self.linear_pid(distance),
                self.get_parameter('linear_speed').value
            )
            cmd.linear.x = forward_speed * (x / distance)  # Scale by forward component
        
            # 2. Angular correction (reduced gain when far away)
            angular_gain = 0.5 if distance > 1.0 else 1.0  # Reduce turning when far
            cmd.angular.z = np.clip(
                self.angular_pid(y) * angular_gain,
                -self.get_parameter('angular_speed').value,
                self.get_parameter('angular_speed').value
            )
        
            # 3. Minimum forward motion guarantee
            if cmd.linear.x < 0.05:  # Always move at least 5cm/s forward
                cmd.linear.x = 0.05
        else:
            # Pure rotation if marker is behind
            cmd.angular.z = 0.3 if y > 0 else -0.3
            self.get_logger().info("Marker behind - aligning first...", throttle_duration_sec=1.0)
    
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Stop all motion"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def search_for_marker(self):
        """Rotate to search for markers"""
        if self.emergency_stop:
            return
            
        cmd = Twist()
        cmd.angular.z = self.get_parameter('angular_speed').value
        self.cmd_vel_pub.publish(cmd)

    def recovery_move(self):
        """Move forward to regain marker detection"""
        if self.emergency_stop:
            return
            
        cmd = Twist()
        cmd.linear.x = 0.1  # Slow forward
        self.cmd_vel_pub.publish(cmd)
        self.recovery_end_time = self.get_clock().now() + rclpy.time.Duration(
            seconds=self.get_parameter('recovery_duration').value)

    def update(self):
        """Main state machine update"""
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