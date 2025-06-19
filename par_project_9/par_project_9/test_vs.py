import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from par_project_9_interfaces.msg import MarkerPointStamped
from enum import Enum


class VisualServoingState(Enum):
    IDLE = 0
    ACTIVE = 1


class VisualServoingNode(Node):
    def __init__(self):
        super().__init__("visual_servoing_node")

        # Parameters
        self.baseVelocity = self.declare_parameter("base_velocity", 0.25).get_parameter_value().double_value

        # Publishers
        self.vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Subscriptions
        self.marker_position_sub = self.create_subscription(
            MarkerPointStamped,
            "marker_position",
            self.marker_callback,
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # State
        self.state = VisualServoingState.IDLE
        self.get_logger().info("Simplified Visual Servoing Node initialized.")

    def marker_callback(self, msg: MarkerPointStamped):
        if self.state == VisualServoingState.ACTIVE:
            return

        if "aurora" in msg.marker.lower():
            self.get_logger().info("Aurora marker detected. Moving forward...")
            self.move_forward()
            self.state = VisualServoingState.ACTIVE

    def move_forward(self):
        cmd = Twist()
        cmd.linear.x = self.baseVelocity
        self.vel_pub.publish(cmd)


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
