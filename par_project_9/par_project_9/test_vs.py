import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from par_project_9_interfaces.msg import MarkerPointStamped
from enum import Enum
import math


class VisualServoingState(Enum):
    IDLE = 0          # Waiting for target marker
    APPROACHING = 1   # Moving toward marker
    TOUCHED = 2       # Successfully reached marker


class VisualServoingNode(Node):
    def __init__(self):
        super().__init__("visual_servoing_node")

        # Parameters
        self.base_velocity = self.declare_parameter(
            "base_velocity", 0.25).get_parameter_value().double_value
        self.stop_distance = self.declare_parameter(
            "stop_distance", 0.3).get_parameter_value().double_value  # meters
        self.slowdown_distance = self.declare_parameter(
            "slowdown_distance", 1.0).get_parameter_value().double_value  # meters

        # Publishers
        self.vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Subscriptions
        self.marker_sub = self.create_subscription(
            MarkerPointStamped,
            "marker_position",
            self.marker_callback,
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # State
        self.state = VisualServoingState.IDLE
        self.current_marker = None
        self.get_logger().info("Distance-based Visual Servoing Node initialized.")

    def marker_callback(self, msg: MarkerPointStamped):
        if self.state == VisualServoingState.TOUCHED:
            return

        # Calculate distance to marker (Euclidean distance)
        marker_distance = math.sqrt(msg.point.x**2 + msg.point.y**2)
        
        if self.state == VisualServoingState.IDLE:
            # New marker detected - start approaching
            self.current_marker = msg.marker
            self.state = VisualServoingState.APPROACHING
            self.get_logger().info(
                f"New marker detected (ID: {msg.marker.id}). Distance: {marker_distance:.2f}m. Starting approach.")
        
        elif self.state == VisualServoingState.APPROACHING:
            # Continue approaching or stop if close enough
            if marker_distance <= self.stop_distance:
                # Reached the marker
                self.stop_robot()
                self.state = VisualServoingState.TOUCHED
                self.get_logger().info(
                    f"Marker touched! (ID: {msg.marker.id}). Ready for next marker.")
                # Reset after a short delay to find next marker
                self.create_timer(2.0, self.reset_state)
            else:
                # Continue approaching with speed adjustment
                self.approach_marker(marker_distance)

    def approach_marker(self, distance):
        cmd = Twist()
        
        # Slow down as we get closer to the marker
        if distance < self.slowdown_distance:
            # Linear interpolation between base_velocity and 0
            speed = self.base_velocity * (distance - self.stop_distance) / (
                self.slowdown_distance - self.stop_distance)
            cmd.linear.x = max(speed, 0.05)  # Minimum speed to keep moving
        else:
            cmd.linear.x = self.base_velocity
            
        self.vel_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()  # All zeros
        self.vel_pub.publish(cmd)

    def reset_state(self):
        self.state = VisualServoingState.IDLE
        self.current_marker = None
        self.get_logger().info("Ready to detect next marker.")


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