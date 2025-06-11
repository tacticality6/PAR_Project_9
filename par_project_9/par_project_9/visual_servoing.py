import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped, Twist
from enum import Enum
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
        self.cameraInfoTopic = self.declare_parameter("info_topic", "/oak/rgb/camera_info").get_parameter_value().string_value

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

        #tf setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #local variables
        self.marker_offset = None
        self.state = VisualServoingState.IDLE


    

    def localise_pointer(self):
        pass


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
        