import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import CameraInfo, CompressedImage
from enum import Enum
import cv2
import numpy as np
from cv_bridge import CvBridge
from par_project_9_interfaces.srv import MarkerConfirmation
from par_project_9_interfaces.msg import MarkerPointStamped

class VisualServoingState(Enum):
    IDLE = 0
    APPROACHING = 1
    ALIGNING = 2
    TOUCHED = 3

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__("visual_servoing_node")

        # Parameters
        self.declare_parameters(namespace='',
            parameters=[
                ('touched_marker_service_name', 'touched_marker'),
                ('touched_distance_tolerance', 0.005),      # 0.5cm tolerance
                ('base_velocity', 1.0),
                ('alignment_tolerance', 0.005),             # 0.5cm alignment tolerance
                ('colour_image_topic', '/oak/rgb/image_raw/compressed'),
                ('depth_image_topic', '/oak/stereo/image_raw/compressedDepth'),
                ('camera_info_topic', '/oak/rgb/camera_info'),
                ('relocalise_pointer_freq', 10.0),
                ('debug_mode', True)
            ])

        # ROS setup
        self.marker_sub = self.create_subscription(
            MarkerPointStamped, 'marker_position', self.marker_callback, 10)

        self.get_logger().info(f"Subscribed to: {self.marker_sub.topic_name}")

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.touch_client = self.create_client(
            MarkerConfirmation, 
            self.get_parameter('touched_marker_service_name').value)

        self.color_sub = self.create_subscription(
            CompressedImage, 
            self.get_parameter('colour_image_topic').value, 
            self.color_callback, 10)

        self.depth_sub = self.create_subscription(
            CompressedImage,
            self.get_parameter('depth_image_topic').value,
            self.depth_callback, 10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback, 10)

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # CV setup
        self.bridge = CvBridge()

        # State variables
        self.state = VisualServoingState.APPROACHING
        self.color_image = None
        self.depth_image = None
        self.camera_info = None

        # Target stopping point
        self.target_x = -0.0700
        self.target_y = 0.0650

        self.get_logger().info("Visual Servoing Node initialized")

    def color_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Color image processing failed: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth image processing failed: {e}")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def marker_callback(self, msg: MarkerPointStamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))

            p_base = do_transform_point(
                PointStamped(header=msg.header, point=msg.point),
                transform)
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        error_x = p_base.point.x - self.target_x
        error_y = p_base.point.y - self.target_y

        if self.get_parameter('debug_mode').value:
            self.get_logger().info(
                f"[STATE: {self.state.name}] Marker pos: x={p_base.point.x:.4f}, y={p_base.point.y:.4f}, "
                f"errors: x={error_x:.4f}, y={error_y:.4f}")

        cmd = Twist()
        base_vel = self.get_parameter('base_velocity').value
        tol_x = self.get_parameter('touched_distance_tolerance').value
        tol_y = self.get_parameter('alignment_tolerance').value

        if self.state == VisualServoingState.APPROACHING:
            if abs(error_x) > tol_x:
                cmd.linear.x = np.clip(error_x * 0.5, -base_vel, base_vel)
                cmd.linear.y = 0.0
            else:
                self.get_logger().info("Reached target X. Switching to ALIGNING.")
                self.state = VisualServoingState.ALIGNING
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0

        elif self.state == VisualServoingState.ALIGNING:
            if abs(error_y) > tol_y:
                cmd.linear.y = np.clip(error_y * 0.5, -base_vel, base_vel)
                cmd.linear.x = 0.0
            else:
                self.get_logger().info("Aligned with marker. Confirming touch.")
                self.state = VisualServoingState.TOUCHED

                req = MarkerConfirmation.Request()
                req.marker = msg.marker
                future = self.touch_client.call_async(req)
                future.add_done_callback(self.handle_service_response)

                self.vel_pub.publish(Twist())
                return

        elif self.state == VisualServoingState.TOUCHED:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0

        self.vel_pub.publish(cmd)

    def handle_service_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Marker is touched - service confirmed")
            else:
                self.get_logger().warn("Marker touch service failed")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        self.state = VisualServoingState.IDLE

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
