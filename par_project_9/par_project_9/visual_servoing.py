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
        #parameters
        self.touchedMarkerServiceName = self.declare_parameter("touched_marker_service_name", "touched_marker").get_parameter_value().string_value
        self.touchedDistanceTolerance = self.declare_parameter("touched_distance_tolerance", 0.25).get_parameter_value().double_value
        self.baseVelocity = self.declare_parameter("base_velocity", 0.25).get_parameter_value().double_value
        self.colourImageTopic = self.declare_parameter("colour_image_topic", "/oak/rgb/image_raw/compressed").get_parameter_value().string_value
        self.depthImageTopic = self.declare_parameter("depth_image_topic", "/oak/stereo/image_raw/compressedDepth").get_parameter_value().string_value
        # self.depthImageTopic = self.declare_parameter("depth_image_topic", "/oak/stereo/image_raw").get_parameter_value().string_value
        self.cameraInfoTopic = self.declare_parameter("info_topic", "/oak/rgb/camera_info").get_parameter_value().string_value
        self.relocaliseFreq = self.declare_parameter("relocalise_pointer_freq", 10.0).get_parameter_value().double_value
        self.debugMode = self.declare_parameter("debug_mode", False).get_parameter_value().bool_value

        # ROS listeners / publishers
        self.marker_position_sub = self.create_subscription(
            MarkerPointStamped,
            "marker_position",
            # self.touchedMarkerServiceName,
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
            # qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        while not self.touch_confirm_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Marker Touched Service not available, Trying Again...")

        self.relocalise_pointer_timer = self.create_timer(
            1.0/self.relocaliseFreq,
            self.localise_pointer
        )

        self.color_image_sub = self.create_subscription(
            CompressedImage,
            self.colourImageTopic,
            self.color_callback,
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            # qos_profile_sensor_data
        )

        self.depth_image_sub = self.create_subscription(
            CompressedImage,
            self.depthImageTopic,
            self.depth_callback,
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            # qos_profile_sensor_data
        )


        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.cameraInfoTopic,
            self.camera_info_callback,
            # qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)4
            qos_profile_sensor_data
        )

        #tf setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #cv2 setup
        self.bridge = CvBridge()

        #local variables
        self.marker_offset = {'x': 0.18, 'y': 0.0, 'z': 0.10}
        self.state = VisualServoingState.ACTIVE if self.debugMode else VisualServoingState.IDLE 
        self.last_seen_stamp   = None   # rclpy.time.Time of most-recent detection
        self.close_enough      = False  # True once error_x & error_y are inside tolerance
        self.missing_timeout_s = 0.8    # tweak: how long marker can vanish before we call "touch"
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.get_logger().info("Visual Servoing Node has been initialized.")

    def color_callback(self, msg):
        # self.get_logger().info("--- Color callback received! ---")
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if self.color_image is None:
            self.get_logger().warn("Failed to decode compressed image in visual_servoing_node")
            return

        # self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Could not convert depth image: {e}")

    # def depth_callback(self, msg):
    #     # Decompress the message manually from the data buffer
    #     np_arr = np.frombuffer(msg.data, np.uint8)

    #     self.depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        
    #     # Check if decoding was successful
    #     if self.depth_image is None:
    #         self.get_logger().warn("Failed to decode compressed depth image")
    #         return

    def camera_info_callback(self, msg):
        # self.get_logger().info("--- Info callback received! ---")
        self.camera_info = msg



    def localise_pointer(self):
        # self.get_logger().info("Localising pointer position...")
        if self.color_image is None or self.depth_image is None or self.camera_info is None:
            # self.get_logger().info("Waiting for camera data...")
            return
        
        self.get_logger().info("Attempting to localise pointer position...")

        # Define orange color range in HSV
        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        
        lower_orange = np.array([0, 100, 100])
        upper_orange = np.array([30, 255, 255])

        # lower_orange = np.array([5, 100, 100])
        # upper_orange = np.array([25, 255, 255])
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
        # depth = self.depth_image[cy, cx]
        # if depth == 0:
        #     self.get_logger().warn("No depth info at marker centroid.")
        #     return
        
        # depth = float(depth)/1000.00

        # Intrinsics
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx_cam = self.camera_info.k[2]
        cy_cam = self.camera_info.k[5]

        x = (cx - cx_cam) * depth / fx
        y = (cy - cy_cam) * depth / fy
        z = depth

        self.get_logger().info(f"Pointer in camera frame: ({x}, {y}, {z})")

        # Package into PointStamped
        point = PointStamped()
        point.header.frame_id = self.camera_info.header.frame_id
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = x
        point.point.y = y
        point.point.z = z

        # Transform to base_link
        try:
            tf = self.tf_buffer.lookup_transform('odom', point.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            base_point = do_transform_point(point, tf)
            offset = {
                'x': base_point.point.x,
                'y': base_point.point.y,
                'z': base_point.point.z
            }
            self.get_logger().info(f"Pointer offset in base_link: {offset}")
            self.marker_offset = offset

        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")


    def marker_callback(self, msg: MarkerPointStamped):
        # Ignore if we’re not active
        if self.state == VisualServoingState.IDLE:
            return

        # ─── 1. Transform marker position into base_link ────────────────
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            p_base = do_transform_point(PointStamped(header=msg.header,
                                                    point=msg.point),
                                        transform)
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        # ─── 2. Compute errors: marker minus pointer ────────────────────
        error_x = p_base.point.x - self.marker_offset['x']   # + → tag in front
        error_y = p_base.point.y - self.marker_offset['y']   # + → tag left

        # ─── 3. Touch window check ─────────────────────────────────────
        # if abs(error_x) < self.touchedDistanceTolerance and \
        # abs(error_y) < self.touchedDistanceTolerance:
        #     self.get_logger().info("Pointer aligned with marker — stopping.")
        #     self.vel_pub.publish(Twist())                    # hard stop

        #     req = MarkerConfirmation.Request()
        #     req.marker = msg.marker
        #     future = self.touch_confirm_client.call_async(req)
        #     future.add_done_callback(self.handle_service_future)

        #     self.state = VisualServoingState.IDLE
        #     return

        # Compute Euclidean distance to marker
        
        distance = (error_x**2 + error_y**2)**0.5

        if distance < 0.25:  # 25 cm stopping distance
            self.get_logger().info("Pointer within 25cm of marker — stopping.")
            self.vel_pub.publish(Twist())  # hard stop

            req = MarkerConfirmation.Request()
            req.marker = msg.marker
            future = self.touch_confirm_client.call_async(req)
            future.add_done_callback(self.handle_service_future)

            self.state = VisualServoingState.IDLE
            return


        # ─── 4. Proportional controller (diff-drive by default) ────────
        k_lin, k_ang = 0.8, 2.0

        # If X-axis points *backward* on your robot, flip the sign:
        fwd_cmd  =  k_lin * (-error_x)      # tag ahead → positive speed
        turn_cmd =  k_ang *   error_y       # tag left  → turn left (CCW)

        cmd = Twist()
        cmd.linear.x  = max(min(fwd_cmd,  0.15), -0.15)   # ±0.15 m/s
        # cmd.angular.z = max(min(turn_cmd, 0.70), -0.70)   # ±0.70 rad/s

        # For a mecanum/holonomic base, comment the line above and use:
        cmd.linear.y = max(min(0.8 * error_y, 0.15), -0.15)

        self.vel_pub.publish(cmd)

    

    def handle_service_future(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Marker Touch Service Failed")
        
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
