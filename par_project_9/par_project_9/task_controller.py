#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from vision_msgs.msg import Detection2DArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy.duration
import math
from enum import Enum

# --- Import definitions of markers ---
try:
    from .marker_definitions import MARKER_MAP
except ImportError:
    print("ERROR: Could not import marker_definitions. Using fallback empty definitions.")
    MARKER_MAP = {}

# --- Enums and Classes ---
# class RobotState(Enum):
#     IDLE = 0
#     NAVIGATING_TO_TARGET = 1
#     PERFORMING_ACTION = 2
#     SEARCHING_WHEN_IDLE = 3

class TargetActionType(Enum): 
    PICKUP = 1
    DELIVERY = 2
    # Add OTHER if have markers like 'home_base' that aren't pickup/delivery
    OTHER = 3

class DetectedMarkerInfo: # Simplified class to hold processed info for logging
    def __init__(self, marker_id: int, action_type: TargetActionType, item_id: str | None, pose: PoseStamped, description: str = ""):
        self.marker_id: int = marker_id
        self.action_type: TargetActionType = action_type
        self.item_id: str | None = item_id
        self.pose: PoseStamped = pose
        self.description: str = description

    def __str__(self):
        return (f"DetectedMarker(ID: {self.marker_id}, Type: {self.action_type.name}, "
                f"Item: {self.item_id or 'N/A'}, Desc: '{self.description}', "
                f"Pose(odom): X={self.pose.pose.position.x:.2f}, Y={self.pose.pose.position.y:.2f})")

class TaskController(Node):
    def __init__(self):
        super().__init__('task_controller')

        if not MARKER_MAP:
            self.get_logger().error("MARKER_MAP is empty! Please ensure marker_definitions.py is correctly populated.")
            rclpy.shutdown()
            return

        # self.current_robot_state = RobotState.IDLE # Not used
        # self.items_on_board = [] # Not used in this step
        # self.current_target_info: ActiveTarget | None = None # Not used

        # self.visible_actionable_markers_data = [] # Logic moved into callback

        # pub/subs
        # self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10) # Not used
        self.aruco_subscriber = self.create_subscription(Detection2DArray, '/aruco_detections', self.detections_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # self.idle_search_actions_count = 0 # Not used
        # self.max_idle_search_actions = int(2 * math.pi / (math.pi / 12)) + 1 # Not used
        # self.decision_timer = self.create_timer(1.0, self.make_decision_and_act) # Not used

        self.get_logger().info('Marker Detection Logger Node Initialized.')
        self.get_logger().info(f"Monitoring for markers defined in MARKER_MAP ({len(MARKER_MAP)} entries).")

    def detections_callback(self, msg: Detection2DArray):
        # This callback will now:
        # 1. Identify known markers from the incoming Detection2DArray.
        # 2. Get their pose in the 'odom' frame.
        # 3. Log this information.
        
        self.get_logger().debug(f"Received {len(msg.detections)} raw detections.")
        detected_and_identified_markers = []

        for detection in msg.detections:
            if not detection.results: continue
            marker_id_str = detection.results[0].hypothesis.class_id
            try:
                detected_id = int(marker_id_str)
            except ValueError:
                self.get_logger().warn(f"Invalid marker ID format in detection: {marker_id_str}")
                continue

            if detected_id in MARKER_MAP:
                marker_config = MARKER_MAP[detected_id]
                item_id = marker_config.get("item_id")
                marker_type_str = marker_config.get("type")
                description = marker_config.get("description", "")

                action_type = TargetActionType.OTHER # Default for non-pickup/delivery
                if marker_type_str == "pickup":
                    action_type = TargetActionType.PICKUP
                elif marker_type_str == "delivery":
                    action_type = TargetActionType.DELIVERY
                
                try:
                    transform_stamped = self.tf_buffer.lookup_transform(
                        'odom', # Target frame (world frame)
                        f'aruco_{detected_id}', # Source frame (marker's TF name from ArucoDetector)
                        msg.header.stamp, # Use the timestamp from the detection message for TF
                        rclpy.duration.Duration(seconds=0.2) # Timeout for lookup
                    )
                    
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp = transform_stamped.header.stamp 
                    pose_stamped.header.frame_id = 'odom' # The frame this pose is in
                    pose_stamped.pose.position.x = transform_stamped.transform.translation.x
                    pose_stamped.pose.position.y = transform_stamped.transform.translation.y
                    pose_stamped.pose.position.z = transform_stamped.transform.translation.z
                    pose_stamped.pose.orientation = transform_stamped.transform.rotation
                    
                    identified_marker = DetectedMarkerInfo(
                        marker_id=detected_id,
                        action_type=action_type,
                        item_id=item_id,
                        pose=pose_stamped,
                        description=description
                    )
                    detected_and_identified_markers.append(identified_marker)
                    
                except Exception as e:
                    self.get_logger().warn(f"TF lookup failed for 'aruco_{detected_id}' to 'odom': {e}")
            # else:
                # self.get_logger().debug(f"Detected marker ID {detected_id} not in MARKER_MAP.")

        if detected_and_identified_markers:
            log_message = "--- Currently Visible & Identified Markers ---\n"
            for marker in detected_and_identified_markers:
                log_message += f"  - {marker}\n"
            self.get_logger().info(log_message.strip())
        else:
            self.get_logger().debug("No markers from MARKER_MAP currently visible or TF lookup failed.")

    # --- Functions below are commented out as they are part of the advanced logic ---
    # def make_decision_and_act(self):
    #     pass

    # def perform_idle_search(self):
    #     pass

    # def initiate_navigation(self, target_pose: PoseStamped):
    #     pass

    # def placeholder_nav_complete(self):
    #     pass

    # def initiate_perform_action(self):
    #     pass

    # def stop_robot_motion(self):
    #     # It's good to have stop_robot_motion available if you ever publish to cmd_vel from here
    #     # For just logging, it's not strictly needed unless a previous state left the robot moving.
    #     # self.get_logger().debug("Sending stop command to /cmd_vel.")
    #     # twist_msg = Twist()
    #     # twist_msg.linear.x = 0.0
    #     # twist_msg.angular.z = 0.0
    #     # if hasattr(self, 'cmd_vel_publisher'): # Check if publisher exists
    #     #    self.cmd_vel_publisher.publish(twist_msg)
    #     pass


    def destroy_node(self):
        self.get_logger().info("Shutting down Marker Detection Logger.")
        # if hasattr(self, 'decision_timer') and self.decision_timer: # Check if timer exists
        #    self.decision_timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    if not MARKER_MAP:
        print("FATAL: MARKER_MAP is empty. Please check marker_definitions.py.")
        return

    marker_logger_node = TaskController()
    try:
        rclpy.spin(marker_logger_node)
    except KeyboardInterrupt:
        marker_logger_node.get_logger().info("Keyboard interrupt received.")
    finally:
        marker_logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()