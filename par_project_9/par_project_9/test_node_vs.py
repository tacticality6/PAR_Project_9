import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import Twist
from enum import Enum
import math

from par_project_9_interfaces.srv import MarkerConfirmation
from par_project_9_interfaces.msg import MarkerPointStamped, Marker

class VisualServoingState(Enum):
    IDLE = 0
    APPROACHING = 1

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__("visual_servoing_node")

        # --- Parameters ---
        self.declare_parameter('k_p_linear', 0.5, "Proportional gain for forward/backward speed")
        self.declare_parameter('k_p_angular', 1.5, "Proportional gain for turning speed")
        self.declare_parameter('max_linear_speed', 0.08, "Max forward/backward speed in m/s")
        self.declare_parameter('max_angular_speed', 0.4, "Max turning speed in rad/s")
        self.declare_parameter('standoff_distance_m', 0.05, "Target distance from marker for the 'touch'")
        self.declare_parameter('disappearance_timeout_s', 1.0, "Time marker must be unseen to count as a touch")

        # --- State Variables ---
        self.state = VisualServoingState.IDLE
        self.target_marker: Marker | None = None
        self.disappearance_timer: rclpy.timer.Timer | None = None

        # --- ROS2 Communications ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribes to the stream of detected markers from aruco_detector
        self.marker_sub = self.create_subscription(
            MarkerPointStamped,
            "marker_position",
            self.marker_callback,
            10
        )

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.touch_confirm_client = self.create_client(MarkerConfirmation, "touched_marker")
        while not self.touch_confirm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("'touched_marker' service not available, trying again...")

        self.get_logger().info("Visual Servoing Node (Touch by Disappearance) Initialized.")

    def marker_callback(self, msg: MarkerPointStamped):
        # If idle, acquire the first seen marker as a new target
        if self.state == VisualServoingState.IDLE:
            self.state = VisualServoingState.APPROACHING
            self.target_marker = msg.marker
            self.get_logger().info(f"New target acquired: Marker ID {self.target_marker.id}. Starting approach.")

        # If we are approaching, only act on messages for our target marker
        if self.state == VisualServoingState.APPROACHING and self.target_marker is not None:
            if msg.marker.id == self.target_marker.id:
                # We see our target. Reset the disappearance timer.
                self.reset_disappearance_timer()
                
                # Move towards the marker
                self.move_towards_marker(msg)

    def move_towards_marker(self, marker_msg: MarkerPointStamped):
        try:
            # We want the marker's position relative to the robot's center (base_link)
            marker_frame = f"aruco_{marker_msg.marker.id}"
            transform = self.tf_buffer.lookup_transform('base_link', marker_frame, rclpy.time.Time())
            
            # The error is the difference between where the marker is and where we want it to be
            # We want it to be at x=standoff_distance, y=0
            error_x = transform.transform.translation.x - self.get_parameter('standoff_distance_m').value
            error_y = transform.transform.translation.y
            
            # P-Controller for Movement
            linear_vel = self.get_parameter('k_p_linear').value * error_x
            angular_vel = self.get_parameter('k_p_angular').value * error_y
            
            # Clamp velocities to safe maximums
            linear_vel = max(min(linear_vel, self.get_parameter('max_linear_speed').value), -self.get_parameter('max_linear_speed').value)
            angular_vel = max(min(angular_vel, self.get_parameter('max_angular_speed').value), -self.get_parameter('max_angular_speed').value)

            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.vel_pub.publish(cmd)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # This can happen if the marker is briefly out of view
            self.get_logger().warn(f"Could not get transform for target marker {self.target_marker.id}: {e}")

    def reset_disappearance_timer(self):
        # If a timer is already running, cancel it
        if self.disappearance_timer is not None:
            self.disappearance_timer.cancel()
        
        # Create a new one-shot timer. If it ever fires, it means we lost the marker.
        timeout = self.get_parameter('disappearance_timeout_s').value
        self.disappearance_timer = self.create_timer(timeout, self.handle_touch)

    def handle_touch(self):
        # This function is ONLY called if the timer fires, meaning the marker disappeared.
        self.get_logger().info(f"MARKER TOUCHED: Target marker {self.target_marker.id} disappeared, assuming contact.")
        
        # Stop the robot
        self.vel_pub.publish(Twist())
        
        # Call the confirmation service
        if self.target_marker is not None:
            req = MarkerConfirmation.Request()
            req.marker = self.target_marker
            self.touch_confirm_client.call_async(req)
        
        # Reset state to IDLE to look for a new target
        self.state = VisualServoingState.IDLE
        self.target_marker = None
        if self.disappearance_timer is not None:
            self.disappearance_timer.cancel()
            self.disappearance_timer = None

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

if __name__ == "__main__":
    main()