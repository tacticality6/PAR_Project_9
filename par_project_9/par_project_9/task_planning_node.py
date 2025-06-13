#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time
import json
import math
from enum import Enum
from typing import Dict, List, Optional, Tuple

# Standard ROS2 messages
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import PoseStamped, PointStamped
from vision_msgs.msg import Detection2DArray

# Custom interfaces
from par_project_9_interfaces.msg import Package, Packages, Marker, Delivery, Deliveries
from par_project_9_interfaces.srv import MarkerConfirmation

# Import marker definitions
try:
    from .marker_definitions import MARKER_MAP
except ImportError:
    print("ERROR: Could not import marker_definitions. Using fallback empty definitions.")
    MARKER_MAP = {}


class RobotState(Enum):
    """Robot state enumeration for task planning state machine"""
    IDLE = "idle"
    EXPLORING = "exploring" 
    NAVIGATING_TO_PICKUP = "navigating_to_pickup"
    PICKING_UP = "picking_up"
    NAVIGATING_TO_DELIVERY = "navigating_to_delivery"
    DELIVERING = "delivering"
    PLANNING = "planning"


class DetectedLocation:
    """Class to store information about detected pickup/delivery locations"""
    def __init__(self, marker_id: int, location_type: str, item_id: str, 
                 position: Optional[Tuple[float, float]] = None, 
                 last_seen: Optional[Time] = None):
        self.marker_id = marker_id
        self.location_type = location_type  # 'pickup' or 'delivery'
        self.item_id = item_id
        self.position = position  # (x, y) in map frame
        self.last_seen = last_seen
        
    def update_position(self, x: float, y: float, timestamp: Time):
        """Update the position and last seen time"""
        self.position = (x, y)
        self.last_seen = timestamp
        
    def is_recent(self, current_time: Time, timeout_seconds: float = 30.0) -> bool:
        """Check if the location was seen recently"""
        if not self.last_seen:
            return False
        elapsed = (current_time - self.last_seen).nanoseconds / 1e9
        return elapsed < timeout_seconds


class TaskPlanningNode(Node):
    def __init__(self, node_name):
        self._node_name = node_name
        super().__init__(node_name)
        
        # Quality of Service profiles
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        
        # === PARAMETERS ===
        self.declare_parameters(
            namespace='',
            parameters=[
                ('navigation_timeout', 30.0),   # Max time to wait for navigation
                ('pickup_timeout', 15.0),       # Max time to attempt pickup
                ('delivery_timeout', 15.0),     # Max time to attempt delivery
                ('location_timeout', 30.0),     # Time before location is considered stale
                ('decision_frequency', 2.0),    # Hz for main decision loop
            ]
        )

        # === STATE VARIABLES ===
        self.current_state = RobotState.IDLE
        self.state_start_time = self.get_clock().now()
        
        # Package and delivery tracking
        self.packages_on_board: List[Package] = []
        self.completed_deliveries: List[Delivery] = []
        self.total_deliveries_completed = 0
        
        # Location tracking - maps marker_id to DetectedLocation
        self.known_locations: Dict[int, DetectedLocation] = {}
        
        # Current task tracking
        self.current_target_marker_id: Optional[int] = None
        self.current_target_item_id: Optional[str] = None
        self.current_navigation_goal: Optional[PoseStamped] = None
        
        # Mission statistics
        self.mission_start_time = self.get_clock().now()
        self.exploration_cycles = 0
        self.failed_pickup_attempts = 0
        self.failed_delivery_attempts = 0

        # === PUBLISHERS ===
        # Exploration control
        self.explore_trigger_pub = self.create_publisher(
            String, "explore_trigger", qos_profile
        )
        
        # Visual servoing control
        self.target_marker_pub = self.create_publisher(
            Int32, "target_marker_id", qos_profile
        )
        
        # Navigation status
        self.navigation_command_pub = self.create_publisher(
            PoseStamped, "navigation_goal", qos_profile
        )
        
        # Mission status and logging
        self.mission_status_pub = self.create_publisher(
            String, "mission_status", qos_profile
        )
        
        # Task planning state for debugging
        self.state_pub = self.create_publisher(
            String, "task_planning_state", qos_profile
        )

        # === SUBSCRIBERS ===
        # Package tracking from delivery tracker
        self.packages_sub = self.create_subscription(
            Packages, "packages_onboard", self.packages_callback, qos_profile
        )
        
        # Completed deliveries from delivery tracker
        self.deliveries_sub = self.create_subscription(
            Deliveries, "completed_deliveries", self.deliveries_callback, qos_profile
        )
        
        # ArUco marker detections for location mapping
        self.markers_sub = self.create_subscription(
            Detection2DArray, "aruco_detections", self.markers_callback, qos_profile
        )
        
        # Navigation status updates
        self.nav_status_sub = self.create_subscription(
            String, "navigation_status", self.navigation_status_callback, qos_profile
        )
        
        # Touch confirmation from visual servoing
        self.touch_status_sub = self.create_subscription(
            Bool, "marker_touched", self.touch_status_callback, qos_profile
        )

        # === SERVICE CLIENTS ===
        # Service client for confirming marker touches
        self.marker_confirmation_client = self.create_client(
            MarkerConfirmation, "touched_marker"
        )

        # === TIMERS ===
        # Main decision loop timer
        decision_freq = self.get_parameter('decision_frequency').value
        self.decision_timer = self.create_timer(1.0 / decision_freq, self.main_decision_loop)
        
        # Status reporting timer
        self.status_timer = self.create_timer(5.0, self.publish_status)

        # === INITIALISATION ===
        self.get_logger().info(f"Task Planning Node '{node_name}' initialised")
        self.get_logger().info(f"- Decision frequency: {decision_freq} Hz")
        self.get_logger().info(f"- Available marker definitions: {len(MARKER_MAP)}")
        
        # Log available marker pairs
        self._log_available_markers()
        
        # Start in planning state to assess initial conditions
        self.transition_to_state(RobotState.PLANNING)

    def _log_available_markers(self):
        """Log information about available pickup/delivery marker pairs"""
        pickup_markers = []
        delivery_markers = []
        
        for marker_id, info in MARKER_MAP.items():
            if info['type'] == 'pickup':
                pickup_markers.append((marker_id, info['item_id']))
            elif info['type'] == 'delivery':
                delivery_markers.append((marker_id, info['item_id']))
        
        self.get_logger().info(f"Available pickup markers: {pickup_markers}")
        self.get_logger().info(f"Available delivery markers: {delivery_markers}")

    # === CALLBACK METHODS ===
    
    def packages_callback(self, msg: Packages):
        """Handle updates to packages currently on board"""
        previous_count = len(self.packages_on_board)
        self.packages_on_board = list(msg.packages)
        current_count = len(self.packages_on_board)
        
        if current_count != previous_count:
            self.get_logger().info(f"Packages on board updated: {previous_count} -> {current_count}")
            
            # Log current packages
            for pkg in self.packages_on_board:
                item_name = self._get_item_name_for_destination(pkg.destination_id)
                self.get_logger().info(f"  - Package {pkg.package_id} for {item_name} (dest: {pkg.destination_id})")

    def deliveries_callback(self, msg: Deliveries):
        """Handle updates to completed deliveries"""
        previous_count = len(self.completed_deliveries)
        self.completed_deliveries = list(msg.deliveries)
        current_count = len(self.completed_deliveries)
        
        if current_count > previous_count:
            self.total_deliveries_completed = current_count
            new_deliveries = current_count - previous_count
            self.get_logger().info(f"New deliveries completed! Total: {current_count} (+{new_deliveries})")
            
            # Log recent deliveries
            for delivery in self.completed_deliveries[-new_deliveries:]:
                item_name = self._get_item_name_for_destination(delivery.destination_id)
                trip_duration = delivery.trip_time.sec + delivery.trip_time.nanosec / 1e9
                self.get_logger().info(f"  - Delivered {item_name} in {trip_duration:.1f}s")

    def markers_callback(self, msg: Detection2DArray):
        """Handle ArUco marker detections to update location map"""
        current_time = self.get_clock().now()
        
        for detection in msg.detections:
            if not detection.results:
                continue
                
            try:
                marker_id = int(detection.results[0].hypothesis.class_id)
            except ValueError:
                continue
            
            # Only process markers that are in our definition map
            if marker_id not in MARKER_MAP:
                continue
                
            marker_info = MARKER_MAP[marker_id]
            location_type = marker_info['type']
            item_id = marker_info.get('item_id')
            
            # Only track pickup and delivery locations
            if location_type not in ['pickup', 'delivery']:
                continue
            
            # Extract position from detection (this is simplified - in practice you'd 
            # want to transform to map coordinates using TF)
            x = detection.bbox.center.position.x
            y = detection.bbox.center.position.y
            
            # Update or create location entry
            if marker_id in self.known_locations:
                self.known_locations[marker_id].update_position(x, y, current_time)
            else:
                self.known_locations[marker_id] = DetectedLocation(
                    marker_id, location_type, item_id, (x, y), current_time
                )
                self.get_logger().info(f"New {location_type} location discovered: {item_id} (marker {marker_id})")

    def navigation_status_callback(self, msg: String):
        """Handle navigation status updates"""
        status = msg.data.lower()
        
        if status == "arrived" or status == "goal_reached":
            self.get_logger().info("Navigation goal reached")
            self._handle_navigation_success()
        elif status == "failed" or status == "aborted":
            self.get_logger().warn("Navigation failed")
            self._handle_navigation_failure()
        elif status == "planning" or status == "following_path":
            # These are normal status updates during navigation
            pass

    def touch_status_callback(self, msg: Bool):
        """Handle touch confirmation from visual servoing system"""
        if msg.data:
            self.get_logger().info("Marker touch confirmed by visual servoing")
            self._handle_successful_touch()
        else:
            self.get_logger().warn("Marker touch failed")
            self._handle_failed_touch()

    # === MAIN DECISION LOOP ===
    
    def main_decision_loop(self):
        """Main decision-making loop that runs the task planning state machine"""
        current_time = self.get_clock().now()
        time_in_state = (current_time - self.state_start_time).nanoseconds / 1e9
        
        # Check for timeouts in current state
        if self._check_state_timeout(time_in_state):
            return
        
        # State machine implementation
        if self.current_state == RobotState.IDLE:
            self._handle_idle_state()
        elif self.current_state == RobotState.PLANNING:
            self._handle_planning_state()
        elif self.current_state == RobotState.EXPLORING:
            self._handle_exploring_state(time_in_state)
        elif self.current_state == RobotState.NAVIGATING_TO_PICKUP:
            self._handle_navigating_to_pickup_state(time_in_state)
        elif self.current_state == RobotState.PICKING_UP:
            self._handle_picking_up_state(time_in_state)
        elif self.current_state == RobotState.NAVIGATING_TO_DELIVERY:
            self._handle_navigating_to_delivery_state(time_in_state)
        elif self.current_state == RobotState.DELIVERING:
            self._handle_delivering_state(time_in_state)

    def _check_state_timeout(self, time_in_state: float) -> bool:
        """Check if current state has timed out and handle accordingly"""
        timeout_map = {
            RobotState.NAVIGATING_TO_PICKUP: self.get_parameter('navigation_timeout').value,
            RobotState.NAVIGATING_TO_DELIVERY: self.get_parameter('navigation_timeout').value,
            RobotState.PICKING_UP: self.get_parameter('pickup_timeout').value,
            RobotState.DELIVERING: self.get_parameter('delivery_timeout').value,
        }
        
        if self.current_state in timeout_map:
            timeout = timeout_map[self.current_state]
            if time_in_state > timeout:
                self.get_logger().warn(f"State {self.current_state.value} timed out after {time_in_state:.1f}s")
                self._handle_state_timeout()
                return True
        
        return False

    def _handle_state_timeout(self):
        """Handle timeout conditions for different states"""
        if self.current_state == RobotState.EXPLORING:
            self.get_logger().info("Exploration timeout - returning to planning")
            self.transition_to_state(RobotState.PLANNING)
        elif self.current_state in [RobotState.NAVIGATING_TO_PICKUP, RobotState.NAVIGATING_TO_DELIVERY]:
            self.get_logger().info("Navigation timeout - returning to planning")
            self.transition_to_state(RobotState.PLANNING)
        elif self.current_state == RobotState.PICKING_UP:
            self.failed_pickup_attempts += 1
            self.get_logger().info(f"Pickup timeout - failed attempts: {self.failed_pickup_attempts}")
            self.transition_to_state(RobotState.PLANNING)
        elif self.current_state == RobotState.DELIVERING:
            self.failed_delivery_attempts += 1
            self.get_logger().info(f"Delivery timeout - failed attempts: {self.failed_delivery_attempts}")
            self.transition_to_state(RobotState.PLANNING)

    # === STATE HANDLERS ===
    
    def _handle_idle_state(self):
        """Handle idle state - typically transitions immediately to planning"""
        self.transition_to_state(RobotState.PLANNING)

    def _handle_planning_state(self):
        """Handle planning state - make decisions about what to do next"""
        self.get_logger().debug("In planning state - assessing situation")
        
        # First priority: deliver packages we already have
        if self.packages_on_board:
            delivery_target = self._find_best_delivery_target()
            if delivery_target:
                self._initiate_delivery(delivery_target)
                return
            else:
                self.get_logger().info("Have packages but no known delivery locations - exploring")
                self._initiate_exploration()
                return
        
        # Second priority: pickup new packages
        self.get_logger().info("No known pickup locations - exploring")
        self._initiate_exploration()
        return

    def _handle_exploring_state(self, time_in_state: float):
        """Handle exploration state"""
        # Exploration is handled by sending explore_trigger commands
        # The actual exploration behavior is implemented by other nodes
        if time_in_state > 5.0:  # Check every 5 seconds if we've found new locations
            if self._has_actionable_locations():
                self.get_logger().info("Found actionable locations during exploration")
                self.stop_exploration()
                self.transition_to_state(RobotState.PLANNING)

    def _handle_navigating_to_pickup_state(self, time_in_state: float):
        """Handle navigation to pickup location"""
        # Navigation progress is handled by navigation_status_callback
        # This state waits for navigation completion
        pass

    def _handle_picking_up_state(self, time_in_state: float):
        """Handle pickup action state"""
        # Pickup is handled by visual servoing and marker confirmation
        # This state waits for touch confirmation
        pass

    def _handle_navigating_to_delivery_state(self, time_in_state: float):
        """Handle navigation to delivery location"""
        # Navigation progress is handled by navigation_status_callback
        # This state waits for navigation completion
        pass

    def _handle_delivering_state(self, time_in_state: float):
        """Handle delivery action state"""
        # Delivery is handled by visual servoing and marker confirmation
        # This state waits for touch confirmation
        pass

    # === NAVIGATION AND ACTION METHODS ===
    
    def _handle_navigation_success(self):
        """Handle successful navigation arrival"""
        if self.current_state == RobotState.NAVIGATING_TO_PICKUP:
            self.get_logger().info("Arrived at pickup location - initiating pickup")
            self.transition_to_state(RobotState.PICKING_UP)
            self._activate_visual_servoing()
        elif self.current_state == RobotState.NAVIGATING_TO_DELIVERY:
            self.get_logger().info("Arrived at delivery location - initiating delivery")
            self.transition_to_state(RobotState.DELIVERING)
            self._activate_visual_servoing()

    def _handle_navigation_failure(self):
        """Handle navigation failure"""
        self.get_logger().warn("Navigation failed - returning to planning")
        self.transition_to_state(RobotState.PLANNING)

    def _handle_successful_touch(self):
        """Handle successful marker touch"""
        if self.current_state == RobotState.PICKING_UP:
            self.get_logger().info("Pickup touch successful - confirming with delivery tracker")
            self._confirm_pickup()
        elif self.current_state == RobotState.DELIVERING:
            self.get_logger().info("Delivery touch successful - confirming with delivery tracker")
            self._confirm_delivery()

    def _handle_failed_touch(self):
        """Handle failed marker touch"""
        self.get_logger().warn("Marker touch failed - returning to planning")
        self.transition_to_state(RobotState.PLANNING)

    def _activate_visual_servoing(self):
        """Activate visual servoing for the current target marker"""
        if self.current_target_marker_id is not None:
            target_msg = Int32()
            target_msg.data = self.current_target_marker_id
            self.target_marker_pub.publish(target_msg)
            self.get_logger().info(f"Activated visual servoing for marker {self.current_target_marker_id}")

    def _confirm_pickup(self):
        """Confirm pickup action with delivery tracking service"""
        if self.current_target_marker_id is None:
            self.get_logger().error("Cannot confirm pickup - no target marker set")
            self.transition_to_state(RobotState.PLANNING)
            return
        
        # Create marker message for pickup confirmation
        marker_msg = Marker()
        marker_msg.id = self.current_target_marker_id
        marker_msg.is_pickup = True
        
        # Find destination ID from marker definitions
        if self.current_target_marker_id in MARKER_MAP:
            item_id = MARKER_MAP[self.current_target_marker_id]['item_id']
            # Find the corresponding delivery marker for this item
            dest_marker_id = self._find_delivery_marker_for_item(item_id)
            if dest_marker_id:
                marker_msg.dest_id = dest_marker_id
            else:
                self.get_logger().error(f"Cannot find delivery marker for item {item_id}")
                self.transition_to_state(RobotState.PLANNING)
                return
        else:
            self.get_logger().error(f"Marker {self.current_target_marker_id} not in definitions")
            self.transition_to_state(RobotState.PLANNING)
            return
        
        # Call the marker confirmation service
        self._call_marker_confirmation_service(marker_msg)

    def _confirm_delivery(self):
        """Confirm delivery action with delivery tracking service"""
        if self.current_target_marker_id is None:
            self.get_logger().error("Cannot confirm delivery - no target marker set")
            self.transition_to_state(RobotState.PLANNING)
            return
        
        # Create marker message for delivery confirmation
        marker_msg = Marker()
        marker_msg.id = self.current_target_marker_id
        marker_msg.dest_id = self.current_target_marker_id  # For delivery, dest_id is the marker itself
        marker_msg.is_pickup = False
        
        # Call the marker confirmation service
        self._call_marker_confirmation_service(marker_msg)

    def _call_marker_confirmation_service(self, marker_msg: Marker):
        """Call the marker confirmation service"""
        if not self.marker_confirmation_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Marker confirmation service not available")
            self.transition_to_state(RobotState.PLANNING)
            return
        
        request = MarkerConfirmation.Request()
        request.marker = marker_msg
        
        future = self.marker_confirmation_client.call_async(request)
        future.add_done_callback(self._marker_confirmation_callback)

    def _marker_confirmation_callback(self, future):
        """Handle response from marker confirmation service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Marker confirmation successful")
                self.transition_to_state(RobotState.PLANNING)
            else:
                self.get_logger().warn("Marker confirmation failed")
                self.transition_to_state(RobotState.PLANNING)
        except Exception as e:
            self.get_logger().error(f"Marker confirmation service call failed: {e}")
            self.transition_to_state(RobotState.PLANNING)

    # === LOCATION FINDING AND TARGETING ===
    
    def _find_best_delivery_target(self) -> Optional[int]:
        """Find the best delivery location for packages we're carrying"""
        if not self.packages_on_board:
            return None
        
        current_time = self.get_clock().now()
        
        # Get all destination IDs we need to deliver to
        needed_destinations = set()
        for package in self.packages_on_board:
            needed_destinations.add(package.destination_id)
        
        # Find delivery markers we know about that match our packages
        available_deliveries = []
        for marker_id, location in self.known_locations.items():
            if (location.location_type == 'delivery' and 
                location.is_recent(current_time) and
                marker_id in needed_destinations):
                available_deliveries.append(marker_id)
        
        if not available_deliveries:
            return None
        
        # For now, just return the first available delivery
        # TODO: Implement distance-based prioritization
        return available_deliveries[0]

    def _has_actionable_locations(self) -> bool:
        """Check if we have any actionable locations (pickups or needed deliveries)"""
        current_time = self.get_clock().now()
        
        # Check for pickups
        for location in self.known_locations.values():
            if (location.location_type == 'pickup' and 
                location.is_recent(current_time)):
                return True
        
        # Check for deliveries if we have packages
        if self.packages_on_board:
            needed_destinations = {pkg.destination_id for pkg in self.packages_on_board}
            for marker_id, location in self.known_locations.items():
                if (location.location_type == 'delivery' and 
                    location.is_recent(current_time) and
                    marker_id in needed_destinations):
                    return True
        
        return False

    def _initiate_pickup(self, marker_id: int):
        """Initiate pickup sequence for the specified marker"""
        self.current_target_marker_id = marker_id
        
        if marker_id in MARKER_MAP:
            item_id = MARKER_MAP[marker_id]['item_id']
            self.current_target_item_id = item_id
            self.get_logger().info(f"Initiating pickup of {item_id} at marker {marker_id}")
        
        # Send navigation goal (simplified - in practice you'd use the actual map coordinates)
        if marker_id in self.known_locations:
            location = self.known_locations[marker_id]
            if location.position:
                self._send_navigation_goal(location.position[0], location.position[1])
                self.transition_to_state(RobotState.NAVIGATING_TO_PICKUP)
            else:
                self.get_logger().error(f"No position available for marker {marker_id}")
                self.transition_to_state(RobotState.PLANNING)
        else:
            self.get_logger().error(f"Marker {marker_id} not in known locations")
            self.transition_to_state(RobotState.PLANNING)

    def _initiate_delivery(self, marker_id: int):
        """Initiate delivery sequence for the specified marker"""
        self.current_target_marker_id = marker_id
        
        if marker_id in MARKER_MAP:
            item_id = MARKER_MAP[marker_id]['item_id']
            self.current_target_item_id = item_id
            self.get_logger().info(f"Initiating delivery of {item_id} at marker {marker_id}")
        
        # Send navigation goal (simplified - in practice you'd use the actual map coordinates)
        if marker_id in self.known_locations:
            location = self.known_locations[marker_id]
            if location.position:
                self._send_navigation_goal(location.position[0], location.position[1])
                self.transition_to_state(RobotState.NAVIGATING_TO_DELIVERY)
            else:
                self.get_logger().error(f"No position available for marker {marker_id}")
                self.transition_to_state(RobotState.PLANNING)
        else:
            self.get_logger().error(f"Marker {marker_id} not in known locations")
            self.transition_to_state(RobotState.PLANNING)

    def _initiate_exploration(self):
        """Start exploration mode to find new markers"""
        self.exploration_cycles += 1
        self.get_logger().info(f"Starting exploration cycle {self.exploration_cycles}")
        
        explore_msg = String()
        explore_msg.data = "start"
        self.explore_trigger_pub.publish(explore_msg)
        
        self.transition_to_state(RobotState.EXPLORING)

    def stop_exploration(self):
        """Stop exploration mode"""
        explore_msg = String()
        explore_msg.data = "stop"
        self.explore_trigger_pub.publish(explore_msg)
        self.get_logger().info("Stopped exploration")

    def _send_navigation_goal(self, x: float, y: float):
        """Send a navigation goal to the navigation system"""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0  # Default orientation
        
        self.navigation_command_pub.publish(goal_msg)
        self.current_navigation_goal = goal_msg
        self.get_logger().info(f"Sent navigation goal: ({x:.2f}, {y:.2f})")

    # === UTILITY METHODS ===
    
    def _get_item_name_for_destination(self, destination_id: int) -> str:
        """Get the item name for a given destination marker ID"""
        if destination_id in MARKER_MAP:
            item_id = MARKER_MAP[destination_id].get('item_id', 'unknown')
            return item_id or f"marker_{destination_id}"
        return f"unknown_dest_{destination_id}"

    def _find_delivery_marker_for_item(self, item_id: str) -> Optional[int]:
        """Find the delivery marker ID for a given item ID"""
        for marker_id, info in MARKER_MAP.items():
            if (info['type'] == 'delivery' and 
                info.get('item_id') == item_id):
                return marker_id
        return None

    def transition_to_state(self, new_state: RobotState):
        """Transition to a new state with logging and cleanup"""
        if new_state == self.current_state:
            return
        
        old_state = self.current_state
        self.current_state = new_state
        self.state_start_time = self.get_clock().now()
        
        self.get_logger().info(f"State transition: {old_state.value} -> {new_state.value}")
        
        # Publish state change for debugging
        state_msg = String()
        state_msg.data = new_state.value
        self.state_pub.publish(state_msg)
        
        # State-specific cleanup and initialization
        if new_state == RobotState.PLANNING:
            # Clear current targets when returning to planning
            self.current_target_marker_id = None
            self.current_target_item_id = None
            self.current_navigation_goal = None

    def publish_status(self):
        """Publish periodic status updates"""
        current_time = self.get_clock().now()
        mission_duration = (current_time - self.mission_start_time).nanoseconds / 1e9
        
        status = {
            "mission_time": f"{mission_duration:.1f}s",
            "state": self.current_state.value,
            "packages_onboard": len(self.packages_on_board),
            "total_deliveries": self.total_deliveries_completed,
            "known_locations": len(self.known_locations),
            "exploration_cycles": self.exploration_cycles,
            "failed_pickups": self.failed_pickup_attempts,
            "failed_deliveries": self.failed_delivery_attempts,
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.mission_status_pub.publish(status_msg)
        
        # Log periodic status
        self.get_logger().info(
            f"Mission Status: {mission_duration:.1f}s | "
            f"State: {self.current_state.value} | "
            f"Packages: {len(self.packages_on_board)} | "
            f"Deliveries: {self.total_deliveries_completed} | "
            f"Locations: {len(self.known_locations)}"
        )


def main(args=None):
    rclpy.init(args=args)

    task_planning_node = TaskPlanningNode("task_planning_node")

    try:
        rclpy.spin(task_planning_node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    task_planning_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
