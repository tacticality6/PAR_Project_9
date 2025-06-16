#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import rclpy.duration
import math
from enum import Enum

# Import the service and message from your custom interfaces package
from par_project_9_interfaces.srv import MarkerConfirmation
from par_project_9_interfaces.msg import Marker

class ApproachState(Enum):
    IDLE = 0
    APPROACHING = 1
    RETREATING = 2

class FinalApproachNode(Node):
    def __init__(self):
        super().__init__('final_approach_node')

        # --- Parameters ---
        self.declare_parameter('k_p_linear', 0.7, "Proportional gain for forward/backward speed")
        self.declare_parameter('k_p_angular', 1.0, "Proportional gain for turning speed")
        self.declare_parameter('max_linear_speed', 0.07, "Max forward/backward speed in m/s")
        self.declare_parameter('max_angular_speed', 0.35, "Max turning speed in rad/s")
        self.declare_parameter('touch_threshold_m', 0.05, "Distance in meters to consider a 'touch'")
        
        # Pointer offset from base_link (MUST be measured from your physical robot)
        self.declare_parameter('pointer_offset_x', 0.18, "Forward(+) from base_link center")
        self.declare_parameter('pointer_offset_y', 0.0,  "Left(+) from base_link center")
        self.declare_parameter('pointer_offset_z', 0.10, "Up(+) from base_link center")

        # TF Frame Names
        self.declare_parameter('camera_frame', 'oak_camera_rgb_camera_optical_frame')
        self.declare_parameter('base_frame', 'base_link')

        # --- State Variables ---
        self.current_state = ApproachState.IDLE
        self.target_marker_info: Marker | None = None

        # --- ROS2 Communications ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # This node gets its target from another node (like a TaskController/TaskPlanner)
        self.target_sub = self.create_subscription(
            Marker, 
            '/visual_servoing/target_marker', # Topic to command this node
            self.target_callback,
            10
        )

        # Client to call the service on your delivery_tracker_node
        self.touch_confirm_client = self.create_client(MarkerConfirmation, 'touched_marker')
        while not self.touch_confirm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("'/touched_marker' Service not available, waiting...")
        
        self.control_timer = self.create_timer(0.1, self.control_loop) # 10Hz control loop

        self.get_logger().info("Final Approach (Visual Servoing) Node Initialized.")

    def target_callback(self, msg: Marker):
        if self.current_state == ApproachState.IDLE:
            self.target_marker_info = msg
            self.set_state(ApproachState.APPROACHING)
            self.get_logger().info(f"New target received: Marker ID {msg.id}. Engaging visual servoing.")
        else:
            self.get_logger().warn(f"Already busy. Ignoring new target {msg.data}.")

    def control_loop(self):
        if self.current_state != ApproachState.APPROACHING:
            return

        if self.target_marker_info is None:
            self.set_state(ApproachState.IDLE)
            return

        try:
            camera_frame = self.get_parameter('camera_frame').value
            
            # 1. Get Marker Pose in Camera Frame
            marker_frame = f"aruco_{self.target_marker_info.id}"
            marker_tf = self.tf_buffer.lookup_transform(camera_frame, marker_frame, rclpy.time.Time())
            marker_pos_cam = marker_tf.transform.translation

            # 2. Get Pointer Pose in Camera Frame
            base_frame = self.get_parameter('base_frame').value
            base_tf = self.tf_buffer.lookup_transform(camera_frame, base_frame, rclpy.time.Time())
            
            # Simple vector addition for the offset in the camera frame is not fully correct if the base_link is rotated.
            # A more robust way is to transform a point.
            pointer_in_base = PointStamped()
            pointer_in_base.header.frame_id = base_frame
            pointer_in_base.point.x = self.get_parameter('pointer_offset_x').value
            pointer_in_base.point.y = self.get_parameter('pointer_offset_y').value
            pointer_in_base.point.z = self.get_parameter('pointer_offset_z').value
            
            # We need the transform from base_frame to camera_frame
            # But lookup_transform gives camera->base. We need to invert it or transform the point.
            # Let's do this correctly by transforming the point.
            # Since tf_buffer.transform is not in tf2_ros for python yet, we will do it manually.
            # This is a bit complex, so we will stick to the simpler logic from your pointer_detector.
            # It assumes the pointer's orientation doesn't matter much for this calculation, which is often true.
            pointer_pos_cam = Point()
            pointer_pos_cam.x = base_tf.transform.translation.x + self.get_parameter('pointer_offset_x').value
            pointer_pos_cam.y = base_tf.transform.translation.y + self.get_parameter('pointer_offset_y').value
            pointer_pos_cam.z = base_tf.transform.translation.z + self.get_parameter('pointer_offset_z').value

            # 3. Calculate Distance and check for SUCCESSFUL TOUCH
            distance_3d = math.sqrt(
                (marker_pos_cam.x - pointer_pos_cam.x)**2 +
                (marker_pos_cam.y - pointer_pos_cam.y)**2 +
                (marker_pos_cam.z - pointer_pos_cam.z)**2
            )

            if distance_3d < self.get_parameter('touch_threshold_m').value:
                self.get_logger().info(f"TOUCH DETECTED on marker {self.target_marker_info.id}!")
                self.handle_touch_success()
                return

            # 4. Calculate Error for Controller
            # We want the pointer to align with the marker, with a small standoff if desired.
            # The error is the difference between where the pointer is and where the marker is.
            lateral_error = marker_pos_cam.x - pointer_pos_cam.x
            distance_error = marker_pos_cam.z - pointer_pos_cam.z # Simple Z-distance error

            self.get_logger().info(f"Approaching Marker {self.target_marker_info.id}: Dist: {distance_3d:.3f}m, FwdErr: {distance_error:.3f}m, LatErr: {lateral_error:.3f}m")

            # 5. P-Controller for Movement
            linear_vel = self.get_parameter('k_p_linear').value * distance_error
            angular_vel = -self.get_parameter('k_p_angular').value * lateral_error

            # Clamp velocities
            linear_vel = max(min(linear_vel, self.get_parameter('max_linear_speed').value), -self.get_parameter('max_linear_speed').value)
            angular_vel = max(min(angular_vel, self.get_parameter('max_angular_speed').value), -self.get_parameter('max_angular_speed').value)

            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Marker {self.target_marker_info.id} not visible. Stopping. TF error: {e}")
            self.stop_robot()
            
    def handle_touch_success(self):
        self.stop_robot()
        self.set_state(ApproachState.IDLE)
        
        req = MarkerConfirmation.Request()
        req.marker = self.target_marker_info
        
        self.get_logger().info(f"Calling '/touched_marker' service for marker ID {req.marker.id}...")
        future = self.touch_confirm_client.call_async(req)
        future.add_done_callback(self.service_call_done_callback)
    
    def service_call_done_callback(self, future):
        try:
            response = future.result()
            if response.success: self.get_logger().info("'/touched_marker' service call SUCCEEDED.")
            else: self.get_logger().error("'/touched_marker' service call reported FAILURE.")
        except Exception as e:
            self.get_logger().error(f"Service call failed with exception: {e}")
        
        self.set_state(ApproachState.RETREATING)
        self.retreat()

    def retreat(self):
        self.get_logger().info("Retreating from marker...")
        retreat_cmd = Twist()
        retreat_cmd.linear.x = -0.1
        self.cmd_vel_pub.publish(retreat_cmd)
        
        self.create_timer(1.5, self.reset_after_retreat, oneshot=True)

    def reset_after_retreat(self):
        self.get_logger().info("Retreat complete. Resetting to IDLE.")
        self.stop_robot()
        self.target_marker_info = None
        self.set_state(ApproachState.IDLE)
            
    def set_state(self, new_state: ApproachState):
        if self.current_state != new_state:
            self.get_logger().info(f"State changed from {self.current_state.name} to {new_state.name}")
            self.current_state = new_state
            
    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    final_approach_node = FinalApproachNode()
    try:
        rclpy.spin(final_approach_node)
    except KeyboardInterrupt: pass
    finally: final_approach_node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()