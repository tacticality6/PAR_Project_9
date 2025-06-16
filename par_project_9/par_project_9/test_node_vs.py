import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
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
        # Parameters
        self.touchedMarkerServiceName = self.declare_parameter("touched_marker_service_name", "touched_marker").get_parameter_value().string_value
        self.touchedDistanceTolerance = self.declare_parameter("touched_distance_tolerance", 0.05).get_parameter_value().double_value
        self.baseVelocity = self.declare_parameter("base_velocity", 0.25).get_parameter_value().double_value
        self.colourImageTopic = self.declare_parameter("colour_image_topic", "/oak/rgb/image_raw/compressed").get_parameter_value().string_value
        # Using the compressed depth topic, consistent with your launch file
        self.depthImageTopic = self.declare_parameter("depth_image_topic", "/oak/rgb/image_raw/compressedDepth").get_parameter_value().string_value
        self.cameraInfoTopic = self.declare_parameter("info_topic", "/oak/rgb/camera_info").get_parameter_value().string_value
        self.relocaliseFreq = self.declare_parameter("relocalise_pointer_freq", 5.0).get_parameter_value().double_value
        self.debugMode = self.declare_parameter("debug_mode", False).get_parameter_value().bool_value
        
        # A robust QoS profile for camera data
        camera_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10 
        )

        # ROS Communications
        self.marker_position_sub = self.create_subscription(MarkerPointStamped, "marker_position", self.marker_callback, 10)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.touch_confirm_client = self.create_client(MarkerConfirmation, self.touchedMarkerServiceName)

        while not self.touch_confirm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Marker Touched Service not available, Trying Again...")

        self.relocalise_pointer_timer = self.create_timer(1.0 / self.relocaliseFreq, self.localise_pointer)

        # Subscriptions using the robust QoS profile
        self.color_image_sub = self.create_subscription(CompressedImage, self.colourImageTopic, self.color_callback, camera_qos_profile)
        # Subscribing to the depth topic expecting a CompressedImage
        self.depth_image_sub = self.create_subscription(CompressedImage, self.depthImageTopic, self.depth_callback, camera_qos_profile)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.cameraInfoTopic, self.camera_info_callback, camera_qos_profile)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()

        # Local variables
        self.marker_offset = None
        self.state = VisualServoingState.ACTIVE if self.debugMode else VisualServoingState.IDLE 
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.get_logger().info("Visual Servoing Node has been initialized.")

    def color_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def depth_callback(self, msg):
        # CORRECTED: Handles CompressedImage messages for depth data
        np_arr = np.frombuffer(msg.data, np.uint8)
        # Use IMREAD_UNCHANGED to preserve the 16-bit depth data from the compressed stream
        self.depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        
        if self.depth_image is None:
            self.get_logger().warn("Failed to decode compressed depth image")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def localise_pointer(self):
        if self.color_image is None or self.depth_image is None or self.camera_info is None:
            return 
        
        # Check if depth image is color (3 channels) or raw (2 dimensions)
        if len(self.depth_image.shape) > 2:
            self.get_logger().error("Depth image appears to be a color image, not raw depth data! Cannot proceed.")
            return

        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([0, 100, 100])
        upper_orange = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return

        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0: return

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        depth_mm = self.depth_image[cy, cx]
        if depth_mm == 0:
            self.get_logger().warn("Pointer tip has no depth data, may be too close/far from camera.")
            return

        depth_m = float(depth_mm) / 1000.0
        self.get_logger().info(f"Pointer localized at depth: {depth_m:.2f}m")

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx_cam = self.camera_info.k[2]
        cy_cam = self.camera_info.k[5]

        x = (cx - cx_cam) * depth_m / fx
        y = (cy - cy_cam) * depth_m / fy
        z = depth_m

        point = PointStamped(header=self.camera_info.header)
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x, point.point.y, point.point.z = x, y, z

        try:
            tf = self.tf_buffer.lookup_transform('base_link', point.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            base_point = do_transform_point(point, tf)
            self.marker_offset = { 'x': base_point.point.x, 'y': base_point.point.y, 'z': base_point.point.z }
            self.get_logger().info(f"Successfully calculated pointer offset in base_link: {self.marker_offset}")
        except Exception as e:
            self.get_logger().error(f"TF transform failed while localizing pointer: {e}")

    def marker_callback(self, msg):
        if self.state == VisualServoingState.IDLE: return
        if self.marker_offset is None:
             self.get_logger().warn("Cannot servo: Pointer position has not been calculated yet.")
             return
        try:
            transform = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            pointstamped = PointStamped(header=msg.header, point=msg.point)
            marker_in_base = do_transform_point(pointstamped, transform)
        except Exception as e:
            self.get_logger().warn(f'Could not transform marker to base_link: {e}')
            return
        error_x = marker_in_base.point.x - self.marker_offset['x']
        error_y = marker_in_base.point.y - self.marker_offset['y']
        if abs(error_x) < self.touchedDistanceTolerance and abs(error_y) < self.touchedDistanceTolerance:
            self.get_logger().info('Pointer aligned with marker — stopping and confirming touch.')
            self.vel_pub.publish(Twist())
            srv_call = MarkerConfirmation.Request(marker=msg.marker)
            self.touch_confirm_client.call_async(srv_call).add_done_callback(self.handle_service_future)
            self.state = VisualServoingState.IDLE
            return
        cmd = Twist()
        cmd.linear.x = self.baseVelocity * error_x
        cmd.linear.y = self.baseVelocity * error_y
        cmd.angular.z = -self.baseVelocity * marker_in_base.point.y
        self.vel_pub.publish(cmd)

    def handle_service_future(self, future):
        try:
            response = future.result()
            if response:
                self.get_logger().info(f"Marker Touch Service Result: {'Succeeded' if response.success else 'Failed'}")
            else:
                self.get_logger().error("Marker Touch Service returned Nothing")
        except Exception as e:
            self.get_logger().error(f"Marker Touch Service call failed: {e}")

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