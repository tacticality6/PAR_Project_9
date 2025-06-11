import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import CameraInfo, CompressedImage
from enum import Enum
import cv2, numpy as np
from cv_bridge import CvBridge
from par_project_9_interfaces.srv import MarkerConfirmation

class VisualServoingState(Enum):
    IDLE = 0
    ACTIVE = 1

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__("visual_servoing_node")
        #parameters
        self.touchedMarkerServiceName = self.declare_parameter("touched_marker_service_name", "touched_marker").get_parameter_value().string_value
        self.touchedDistanceTolerance = self.declare_parameter("touched_distance_tolerance", 0.05).get_parameter_value().double_value
        self.baseVelocity = self.declare_parameter("base_velocity", 0.25).get_parameter_value().double_value
        self.touchedMarkerServiceName = self.declare_parameter("touched_marker_service_name", "touched_marker").get_parameter_value().string_value
        self.colourImageTopic = self.declare_parameter("colour_image_topic", "/oak/rgb/image_raw/compressed").get_parameter_value().string_value
        self.depthImageTopic = self.declare_parameter("depth_image_topic", "/oak/rgb/image_raw/compressedDepth").get_parameter_value().string_value
        self.cameraInfoTopic = self.declare_parameter("info_topic", "/oak/rgb/camera_info").get_parameter_value().string_value
        self.relocaliseFreq = self.declare_parameter("relocalise_pointer_freq", 10.0).get_parameter_value().double_value

        # ROS listeners / publishers
        self.marker_position_sub = self.create_subscription(
            PointStamped,
            self.touchedMarkerServiceName,
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
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.relocalise_pointer_timer = self.create_timer(
            self.relocaliseFreq,
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
            self.color_callback,
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.color_image_sub = self.create_subscription(
            CompressedImage,
            self.colourImageTopic,
            self.depth_callback,
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.cameraInfoTopic,
            self.camera_info_callback,
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        #tf setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #cv2 setup
        self.bridge = CvBridge()

        #local variables
        self.marker_offset = None
        self.state = VisualServoingState.IDLE
        self.color_image = None
        self.depth_image = None
        self.camera_info = None

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        self.camera_info = msg


    

    def localise_pointer(self):
        self.get_logger().info("Localising pointer position...")
        if self.color_image is None or self.depth_image is None or self.camera_info is None:
            self.get_logger().info("Waiting for camera data...")
            return

        # Define orange color range in HSV
        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([5, 100, 100])
        upper_orange = np.array([25, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Find contours and centroid
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().warn("No orange marker detected.")
            return

        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            self.get_logger().warn("Zero-area marker.")
            return

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # Get depth at centroid
        depth = self.depth_image[cy, cx] / 1000.0  # convert mm to meters
        if depth == 0:
            self.get_logger().warn("No depth info at marker centroid.")
            return

        # Intrinsics
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx_cam = self.camera_info.k[2]
        cy_cam = self.camera_info.k[5]

        x = (cx - cx_cam) * depth / fx
        y = (cy - cy_cam) * depth / fy
        z = depth

        self.get_logger().info(f"Pointer in camera frame: ({x:.2f}, {y:.2f}, {z:.2f})")

        # Package into PointStamped
        point = PointStamped()
        point.header.frame_id = self.camera_info.header.frame_id
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = x
        point.point.y = y
        point.point.z = z

        # Transform to base_link
        try:
            tf = self.tf_buffer.lookup_transform('base_link', point.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            base_point = do_transform_point(point, tf)
            offset = {
                'x': base_point.point.x,
                'y': base_point.point.y,
                'z': base_point.point.z
            }
            self.get_logger().info(f"Pointer offset in base_link: {offset}")
            self.marker_offset = offset
            # Save or publish this offset to use in your servoing node
            # e.g., write to shared param, file, or topic
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")


    def marker_callback(self, msg):
        if self.state == VisualServoingState.IDLE:
            return
        
        #transform marker to base_link
        try:
            # Transform marker point into robot base_link frame
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            marker_in_base = do_transform_point(msg, transform)
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return
        
        #calculate differential from marker and pointer
        try:
            error_x = marker_in_base.point.x - self.marker_offset['x']
            error_y = marker_in_base.point.y - self.marker_offset['y']
            error_z = marker_in_base.point.z - self.marker_offset['z']
        except Exception as e:
            self.get_logger().warn(f'Differential Calculation Fialed: {e}')
            return
        
        
        #PID loop
        if abs(error_x) < self.touchedDistanceTolerance and abs(error_y) < self.touchedDistanceTolerance:
            self.get_logger().info('Pointer aligned with marker — stopping.')
            self.publisher.publish(Twist())  # Zero velocity
            #TODO - MARK complete
            return
        

        cmd = Twist()
        cmd.linear.x = self.baseVelocity * error_x
        cmd.linear.y = self.baseVelocity * error_y
        cmd.angular.z = -self.baseVelocity * marker_in_base.point.x  # turn toward marker

        self.vel_pub.publish(cmd)
        