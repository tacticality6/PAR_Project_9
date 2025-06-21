import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from enum import Enum
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

# Import the new service and other interfaces
from par_project_9_interfaces.srv import MarkerConfirmation, StartServoing
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
                ('base_velocity', 0.15), # Reduced for more stable control
                ('target_distance', 0.25),
                ('alignment_tolerance', 0.02),
            ])

        # State variables
        self.state = VisualServoingState.IDLE
        self.target_marker_id = None # To store the ID of the marker we are approaching
        self.pointer_offset = {'x': 0.20, 'y': 0.0} # Offset of the pointer from the robot's base_link

        # ROS setup
        self.marker_sub = self.create_subscription(
            MarkerPointStamped, 'marker_position', self.marker_callback, 10)
        self.get_logger().info(f"Subscribed to: {self.marker_sub.topic_name}")

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service client to confirm touch
        self.touch_client = self.create_client(
            MarkerConfirmation, self.get_parameter('touched_marker_service_name').value)

        # --- SOLUTION: Add Service Server to start the process ---
        self.start_servoing_service = self.create_service(
            StartServoing, 'start_servoing', self.start_servoing_callback)

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Visual Servoing Node initialized and ready.")

    # --- Callback for the new service ---
    def start_servoing_callback(self, request, response):
        if self.state != VisualServoingState.IDLE:
            self.get_logger().warn("Cannot start a new servoing task while another is active.")
            response.success = False
            return response

        self.target_marker_id = request.marker_id
        self.state = VisualServoingState.APPROACHING
        self.get_logger().info(f"Starting visual servoing for marker ID: '{self.target_marker_id}'")
        response.success = True
        return response

    def marker_callback(self, msg: MarkerPointStamped):
        # --- SOLUTION: Ignore if we are idle OR if it's not our target marker ---
        if self.state == VisualServoingState.IDLE or msg.marker_id != self.target_marker_id:
            return

        self.get_logger().info(f"Processing marker '{msg.marker_id}'...")

        # Transform marker position to the robot's base_link frame for control
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', # Target frame
                msg.header.frame_id, # Source frame
                rclpy.time.Time())
            p_base = do_transform_point(PointStamped(header=msg.header, point=msg.point), transform)
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        # Calculate errors relative to the pointer's position
        error_x = p_base.point.x - self.pointer_offset['x'] # Forward/backward error
        error_y = p_base.point.y - self.pointer_offset['y'] # Sideways error

        cmd = Twist()

        if self.state == VisualServoingState.APPROACHING:
            # Check if we have reached the stopping distance
            if abs(error_x) < 0.02: # 2cm tolerance for stopping
                self.get_logger().info("Reached target distance. Switching to ALIGNING.")
                self.state = VisualServoingState.ALIGNING
                self.vel_pub.publish(Twist()) # Stop the robot
                return
            else:
                # Proportional control for forward motion
                cmd.linear.x = np.clip(0.7 * error_x, -self.get_parameter('base_velocity').value, self.get_parameter('base_velocity').value)
                # Add slight angular correction to keep facing the marker
                cmd.angular.z = np.clip(-1.0 * error_y, -0.5, 0.5)

        elif self.state == VisualServoingState.ALIGNING:
             # Check if we are aligned sideways
            if abs(error_y) < self.get_parameter('alignment_tolerance').value:
                self.get_logger().info("Pointer is aligned. Marker is 'touched'.")
                self.state = VisualServoingState.TOUCHED
                self.vel_pub.publish(Twist()) # Stop the robot

                # Call service to log the touch
                req = MarkerConfirmation.Request()
                req.marker_id = self.target_marker_id
                
                self.touch_client.call_async(req)
                # Reset state to IDLE after a short delay or upon service response
                self.get_logger().info("Task complete. Returning to IDLE state.")
                self.state = VisualServoingState.IDLE
                self.target_marker_id = None
                return
            else:
                 # Proportional control for sideways (mecanum) motion
                cmd.linear.y = np.clip(1.2 * error_y, -self.get_parameter('base_velocity').value, self.get_parameter('base_velocity').value)
                # We might still need small forward adjustments if the robot drifts
                cmd.linear.x = np.clip(0.5 * error_x, -0.05, 0.05) # Slower correction

        self.vel_pub.publish(cmd)

# --- Main function remains the same ---
def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the robot stops if the node is shut down
        node.vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()