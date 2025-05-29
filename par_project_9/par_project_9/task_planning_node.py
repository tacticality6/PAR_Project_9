#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy


class TaskPlanningNode(Node):
    def __init__(self, node_name):
        self._node_name = node_name
        super().__init__(node_name)
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        # State variables
        self.packages_on_board = []  # List of Package objects
        self.mapped_dropoff_locations = {}  # Dict: location_id -> DropOffLocation
        self.completed_deliveries = 0
        self.current_target_location = None
        self.is_exploring = False

        # Publishers
        # TODO: explore_trigger topic controls exploration behavior
        self.explore_trigger_pub = self.create_publisher(
            String, "explore_trigger", qos_profile
        )

        # TODO: more publishers...

        # Subscribers
        # TODO: packages_on_board topic provides package information
        self.packages_sub = self.create_subscription(
            String, "packages_on_board", self.packages_callback, 10
        )

        # TODO: markers_detected topic provides detected marker information
        self.markers_sub = self.create_subscription(
            String, "markers_detected", self.markers_callback, 10
        )

        # TODO: navigation_status topic indicates when navigation is complete
        self.nav_status_sub = self.create_subscription(
            String, "navigation_status", self.navigation_status_callback, 10
        )

        # TODO: more subscribers...

        # Timer for main decision loop
        self.decision_timer = self.create_timer(1.0, self.main_decision_loop)

        self.get_logger().info(f"Task Planning Node '{node_name}' initialized")
        # TODO: verify the following topics and their formats
        self.get_logger().info(
            "- packages_on_board topic format: JSON with package_id, destination_id, pickup_time"
        )
        self.get_logger().info(
            "- markers_detected topic format: JSON with marker_id, type, position, destination_id"
        )
        self.get_logger().info("- explore_trigger topic: 'start'/'stop' commands")

    def packages_callback(self, msg):
        """
        Callback for packages_on_board topic
        """
        # TODO: implement this function
        pass

    def markers_callback(self, msg):
        """
        Callback for markers_detected topic
        TODO: Message format is JSON string with marker information
        Example: {"marker_id": "M001", "type": "pickup", "position": {"x": 1.0, "y": 2.0, "z": 0.0}, "destination_id": "D001"}
        """
        # TODO: implement this function
        pass

    def navigation_status_callback(self, msg):
        """
        Callback for navigation status updates
        TODO: Message contains 'arrived' when robot reaches target
        """
        # TODO: implement this function
        pass

    def main_decision_loop(self):
        """
        Main decision loop
        This runs periodically to make decisions based on current state
        """
        # Check if we have parcels
        if self.has_parcels():
            # Check if we have mapped drop-off locations
            if len(self.mapped_dropoff_locations) > 0:
                # Calculate closest drop-off and navigate there
                closest_dropoff = self.calculate_closest_dropoff()
                if closest_dropoff:
                    self.navigate_to_dropoff(closest_dropoff)
            else:
                # No mapped locations - start exploring
                self.start_exploration()
        else:
            # No parcels - start exploring to find pickup markers
            self.start_exploration()

    def has_parcels(self):
        """Check if robot has any parcels on board"""
        return len(self.packages_on_board) > 0

    def has_parcel_for_location(self, location_id):
        """Check if robot has a parcel for the specified location"""
        return any(
            package.destination_id == location_id for package in self.packages_on_board
        )

    def calculate_closest_dropoff(self):
        """Calculate the closest drop-off location that we have a parcel for"""
        # TODO: Implement this function
        return None

    def navigate_to_dropoff(self, dropoff_location):
        """Navigate to the specified drop-off location"""
        # TODO: Implement this function
        pass

    def increment_completed_deliveries(self):
        """Increment the completed deliveries counter"""
        # TODO: confirm this function
        self.completed_deliveries += 1
        self.get_logger().info(f"Completed deliveries: {self.completed_deliveries}")

    def start_exploration(self):
        """Start exploration mode"""
        # TODO: confirm this function
        if not self.is_exploring:
            explore_msg = String()
            explore_msg.data = "start"
            self.explore_trigger_pub.publish(explore_msg)
            self.is_exploring = True
            self.get_logger().info("Started exploration")

    def stop_exploration(self):
        """Stop exploration mode"""
        # TODO: confirm this function
        if self.is_exploring:
            explore_msg = String()
            explore_msg.data = "stop"
            self.explore_trigger_pub.publish(explore_msg)
            self.is_exploring = False
            self.get_logger().info("Stopped exploration")


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
