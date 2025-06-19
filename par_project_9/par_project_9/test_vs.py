#!/usr/bin/env python3

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
    TOUCHED = 2       # Successfully reached marker (25cm)


class VisualServoingNode(Node):
    def __init__(self):
        super().__init__("visual_servoing_node")

        # Parameters
        self.declare_parameter("base_velocity", 0.25)  # m/s
        self.declare_parameter("stop_distance", 0.25)  # 25cm in meters
        self.declare_parameter("angular_gain", 0.5)    # For turning correction

        # Publishers
        self.vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Subscriptions
        self.marker_sub = self.create_subscription(
            MarkerPointStamped,
            "marker_position",
            self.marker_callback,
            10
        )

        # State variables
        self.state = VisualServoingState.IDLE
        self.current_marker = None

        self.get_logger().info("Visual Servoing Node initialized - will stop at 25cm")

    def marker_callback(self, msg):
        try:
            # Get current parameters
            base_velocity = self.get_parameter("base_velocity").value
            stop_distance = self.get_parameter("stop_distance").value
            angular_gain = self.get_parameter("angular_gain").value

            # Calculate distance and angle to marker
            distance = math.sqrt(msg.point.x**2 + msg.point.y**2)
            angle = math.atan2(msg.point.y, msg.point.x)  # Angle from robot center to marker

            if self.state == VisualServoingState.IDLE:
                # New marker detected - start approaching
                self.current_marker = msg.marker
                self.state = VisualServoingState.APPROACHING
                self.get_logger().info(f"Approaching marker ID {msg.marker.id} at {distance:.2f}m")

            elif self.state == VisualServoingState.APPROACHING:
                if distance <= stop_distance:
                    # Reached target distance
                    self.stop_robot()
                    self.state = VisualServoingState.TOUCHED
                    self.get_logger().info(f"Reached marker (distance: {distance:.2f}m)")
                    # Reset after delay to find next marker
                    self.create_timer(2.0, self.reset_state)
                else:
                    # Move toward marker with heading correction
                    cmd = Twist()
                    cmd.linear.x = base_velocity
                    cmd.angular.z = angular_gain * angle  # Correct heading toward marker
                    self.vel_pub.publish(cmd)

        except Exception as e:
            self.get_logger().error(f"Error in marker callback: {str(e)}")

    def stop_robot(self):
        """Stop all robot movement"""
        cmd = Twist()  # All zeros
        self.vel_pub.publish(cmd)

    def reset_state(self):
        """Return to IDLE state after reaching marker"""
        self.state = VisualServoingState.IDLE
        self.current_marker = None
        self.get_logger().info("Ready for next marker")


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