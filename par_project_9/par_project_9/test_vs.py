import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
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
    APPROACHING = 1      # Moving toward marker to reach target distance
    ALIGNING = 2         # Sideways alignment of pointer with marker
    TOUCHED = 3          # Successfully touched marker

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__("visual_servoing_node")
        
        # Parameters
        self.declare_parameters(namespace='',
            parameters=[
                ('touched_marker_service_name', 'touched_marker'),
                ('touched_distance_tolerance', 0.05),      # 5cm tolerance
                ('base_velocity', 0.25),
                ('target_distance', 0.25),                # 25cm stopping distance
                ('alignment_tolerance', 0.02),            # 2cm alignment tolerance
                ('colour_image_topic', '/oak/rgb/image_raw/compressed'),
                ('depth_image_topic', '/oak/stereo/image_raw/compressedDepth'),
                ('camera_info_topic', '/oak/rgb/camera_info'),
                ('relocalise_pointer_freq', 10.0),
                ('debug_mode', False)
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
        self.state = VisualServoingState.IDLE
        self.marker_offset = {'x': 0.18, 'y': 0.0, 'z': 0.10}  # Pointer offset from base
        self.last_marker_time = self.get_clock().now()
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        
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

        self.get_logger().info(f"Received marker at X={msg.point.x}, Y={msg.point.y}")

        current_time = self.get_clock().now()
        
        if self.state == VisualServoingState.IDLE:
            self.get_logger().info("Marker detected — starting to approach")
            self.state = VisualServoingState.APPROACHING

            
        # Transform marker position to base_link frame
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            
            p_base = do_transform_point(
                PointStamped(header=msg.header, point=msg.point),
                transform)
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        # Calculate errors
        error_x = p_base.point.x - self.marker_offset['x']  # Forward distance to marker
        error_y = p_base.point.y - self.marker_offset['y']  # Sideways offset
        
        # State machine for visual servoing
        cmd = Twist()
        
        if self.state == VisualServoingState.APPROACHING:
            # Move toward marker until target distance (25cm)
            if abs(error_x) <= self.get_parameter('target_distance').value:
                self.get_logger().info("Reached target distance, starting alignment")
                self.state = VisualServoingState.ALIGNING
            else:
                # Proportional control for forward/backward motion
                cmd.linear.x = np.clip(
                    0.5 * error_x, 
                    -self.get_parameter('base_velocity').value,
                    self.get_parameter('base_velocity').value)
                
        elif self.state == VisualServoingState.ALIGNING:
            # Sideways movement to align pointer with marker
            if abs(error_y) <= self.get_parameter('alignment_tolerance').value:
                self.get_logger().info("Pointer aligned with marker - logging touch")
                self.state = VisualServoingState.TOUCHED
                
                # Call service to log touch
                req = MarkerConfirmation.Request()
                req.marker = msg.marker
                future = self.touch_client.call_async(req)
                future.add_done_callback(self.handle_service_response)
                
                # Stop robot
                self.vel_pub.publish(Twist())
            else:
                # Proportional control for sideways motion (mecanum)
                cmd.linear.y = np.clip(
                    0.8 * error_y,
                    -self.get_parameter('base_velocity').value,
                    self.get_parameter('base_velocity').value)
        
        # Publish velocity command if not in TOUCHED state
        if self.state != VisualServoingState.TOUCHED:
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
        
        # Reset state after touch is completed
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