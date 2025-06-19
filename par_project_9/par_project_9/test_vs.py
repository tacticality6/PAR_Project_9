import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import tf2_ros
import tf2_geometry_msgs
import numpy as np


class MarkerFollower(Node):
    def __init__(self):
        super().__init__('marker_follower')

        # Publisher to cmd_vel
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber to the marker position (in camera frame)
        self.marker_sub = self.create_subscription(
            PointStamped,
            '/marker_position',
            self.marker_callback,
            10
        )

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Target offset from the marker (desired stopping point)
        self.marker_offset = {'x': 0.4, 'y': 0.0}  # Stop 0.4m in front

        self.get_logger().info("Marker Follower Node Started")

    def marker_callback(self, msg):
        try:
            # Transform marker point to base_link (robot) frame
            transform = self.tf_buffer.lookup_transform(
                'odom',
                msg.header.frame_id,
                rclpy.time.Time()
            )
            p_base = tf2_geometry_msgs.do_transform_point(msg, transform)

            # Compute errors (desired - current)
            error_x = p_base.point.x - self.marker_offset['x']
            error_y = p_base.point.y - self.marker_offset['y']

            # Proportional control
            k_lin = 0.8
            k_ang = 2.0

            fwd_cmd = k_lin * error_x         # +ve error_x = go forward
            turn_cmd = -k_ang * error_y       # +ve error_y = turn right (need -ve angular.z)

            # Clamp velocities
            fwd_cmd = np.clip(fwd_cmd, -0.15, 0.15)
            turn_cmd = np.clip(turn_cmd, -0.70, 0.70)

            # Send command
            cmd = Twist()
            cmd.linear.x = fwd_cmd
            cmd.angular.z = turn_cmd
            self.vel_pub.publish(cmd)

            self.get_logger().info(
                f"Marker x={p_base.point.x:.2f}, y={p_base.point.y:.2f} -> cmd: lin={fwd_cmd:.2f}, ang={turn_cmd:.2f}"
            )

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = MarkerFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
