import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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
    ACTIVE = 1

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__("visual_servoing_node")

        # ─── Parameters ─────────────────────────────────────────────────
        self.touchedMarkerServiceName = self.declare_parameter("touched_marker_service_name", "touched_marker").get_parameter_value().string_value
        self.touchedDistanceTolerance = self.declare_parameter("touched_distance_tolerance", 0.05).get_parameter_value().double_value
        self.baseVelocity = self.declare_parameter("base_velocity", 0.25).get_parameter_value().double_value
        self.colourImageTopic = self.declare_parameter("colour_image_topic", "/oak/rgb/image_raw/compressed").get_parameter_value().string_value
        self.depthImageTopic = self.declare_parameter("depth_image_topic", "/oak/stereo/image_raw/compressedDepth").get_parameter_value().string_value
        self.cameraInfoTopic = self.declare_parameter("info_topic", "/oak/rgb/camera_info").get_parameter_value().string_value
        self.relocaliseFreq = self.declare_parameter("relocalise_pointer_freq", 10.0).get_parameter_value().double_value
        self.debugMode = self.declare_parameter("debug_mode", False).get_parameter_value().bool_value

        # ─── Subscriptions & Publishers ─────────────────────────────────
        self.marker_position_sub = self.create_subscription(
            MarkerPointStamped,
            "marker_position",
            self.marker_callback,
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.touch_confirm_client = self.create_client(
            MarkerConfirmation,
            self.touchedMarkerServiceName,
        )
        while not self.touch_confirm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Marker Touch Service not available, retrying...")

        self.relocalise_pointer_timer = self.create_timer(
            1.0 / self.relocaliseFreq,
            self.localise_pointer
        )

        self.color_image_sub = self.create_subscription(
            CompressedImage,
            self.colourImageTopic,
            self.color_callback,
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.depth_image_sub = self.create_subscription(
            CompressedImage,
            self.depthImageTopic,
            self.depth_callback,
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.cameraInfoTopic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        # ─── TF Setup ──────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ─── CV Setup ──────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ─── Internal State ─────────────────────────────────────────────
        self.marker_offset = {'x': 0.18, 'y': 0.0, 'z': 0.10}
        self.state = VisualServoingState.ACTIVE if self.debugMode else VisualServoingState.IDLE 
        self.color_image = None
        self.depth_image = None
        self.camera_info = None

        self.get_logger().info("Visual Servoing Node has been initialized.")

    def color_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Could not convert depth image: {e}")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def localise_pointer(self):
        if self.color_image is None or self.depth_image is None or self.camera_info is None:
            return

        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([0, 100, 100])
        upper_orange = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        depth = self.depth_image[cy, cx] / 1000.0  # convert mm to meters

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx_cam = self.camera_info.k[2]
        cy_cam = self.camera_info.k[5]

        x = (cx - cx_cam) * depth / fx
        y = (cy - cy_cam) * depth / fy
        z = depth

        point = PointStamped()
        point.header.frame_id = self.camera_info.header.frame_id
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = x
        point.point.y = y
        point.point.z = z

        try:
            tf = self.tf_buffer.lookup_transform('base_link', point.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            base_point = do_transform_point(point, tf)
            self.marker_offset = {
                'x': base_point.point.x,
                'y': base_point.point.y,
                'z': base_point.point.z
            }
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")

    def marker_callback(self, msg: MarkerPointStamped):
        if self.state == VisualServoingState.IDLE:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            p_base = do_transform_point(PointStamped(header=msg.header, point=msg.point), transform)
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        error_x = p_base.point.x - self.marker_offset['x']
        error_y = p_base.point.y - self.marker_offset['y']

        if abs(error_x) < self.touchedDistanceTolerance and abs(error_y) < self.touchedDistanceTolerance:
            self.get_logger().info("Pointer aligned with marker — stopping.")
            self.vel_pub.publish(Twist())

            req = MarkerConfirmation.Request()
            req.marker = msg.marker
            future = self.touch_confirm_client.call_async(req)
            future.add_done_callback(self.handle_service_future)

            self.state = VisualServoingState.IDLE
            return

        # Proportional controller
        k_lin, k_ang = 0.8, 2.0
        fwd_cmd = k_lin * (-error_x)
        turn_cmd = k_ang * error_y

        cmd = Twist()
        cmd.linear.x = max(min(fwd_cmd, 0.15), -0.15)
        cmd.angular.z = max(min(turn_cmd, 0.70), -0.70)

        self.vel_pub.publish(cmd)

    def handle_service_future(self, future):
        try:
            response = future.result()
        except Exception:
            self.get_logger().error("Marker Touch Service Failed")
            return

        if response is None:
            self.get_logger().error("Marker Touch Service Returned Nothing")
        else:
            self.get_logger().info(f"Marker Touch Service Result: {'Succeeded' if response.success else 'Failed'}")

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
