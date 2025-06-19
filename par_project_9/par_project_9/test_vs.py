#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, Twist
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from par_project_9_interfaces.srv import MarkerConfirmation
from par_project_9_interfaces.msg import MarkerPointStamped
from enum import Enum


class VisualServoingState(Enum):
    IDLE = 0
    ACTIVE = 1


class VisualServoingNode(Node):
    def __init__(self):
        super().__init__("visual_servoing_node")

        # Parameters
        self.touched_marker_service = self.declare_parameter(
            "touched_marker_service_name", 
            "touched_marker"
        ).get_parameter_value().string_value
        self.tolerance = self.declare_parameter(
            "touched_distance_tolerance", 
            0.05
        ).get_parameter_value().double_value
        self.max_lin_vel = self.declare_parameter(
            "max_linear_velocity", 
            0.15
        ).get_parameter_value().double_value
        self.max_ang_vel = self.declare_parameter(
            "max_angular_velocity", 
            0.70
        ).get_parameter_value().double_value
        self.k_lin = self.declare_parameter(
            "k_linear", 
            0.8
        ).get_parameter_value().double_value
        self.k_ang = self.declare_parameter(
            "k_angular", 
            2.0
        ).get_parameter_value().double_value

        # Marker offset from detected center
        self.marker_offset = {'x': 0.18, 'y': 0.0}

        # State
        self.state = VisualServoingState.ACTIVE

        # TF Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers & Subscribers
        self.vel_pub = self.create_publisher(
            Twist, 
            "/cmd_vel", 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.marker_sub = self.create_subscription(
            MarkerPointStamped,
            "marker_position",
            self.marker_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.touch_client = self.create_client(
            MarkerConfirmation, 
            self.touched_marker_service
        )
        while not self.touch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Marker Touch Service...")

        self.get_logger().info("Visual Servoing Node initialized - Fixed Syntax")

    def marker_callback(self, msg: MarkerPointStamped):
        if self.state != VisualServoingState.ACTIVE:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            p_base = do_transform_point(
                PointStamped(header=msg.header, point=msg.point), 
                transform
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        error_x = p_base.point.x - self.marker_offset['x']
        error_y = -(p_base.point.y - self.marker_offset['y'])

        self.get_logger().info(f"Marker error - X: {error_x:.3f}, Y: {error_y:.3f}")

        if abs(error_x) < self.tolerance and abs(error_y) < self.tolerance:
            self.get_logger().info("Reached marker - stopping.")
            self.vel_pub.publish(Twist())

            req = MarkerConfirmation.Request()
            req.marker = msg.marker
            future = self.touch_client.call_async(req)
            future.add_done_callback(self.handle_service_future)

            self.state = VisualServoingState.IDLE
            return

        cmd = Twist()
        cmd.linear.x = min(self.k_lin * error_x, self.max_lin_vel)
        cmd.angular.z = max(min(-self.k_ang * error_y, self.max_ang_vel), -self.max_ang_vel)
        self.vel_pub.publish(cmd)

    def handle_service_future(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Marker Touch Confirmed.")
            else:
                self.get_logger().warn("Marker Touch Rejected.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


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