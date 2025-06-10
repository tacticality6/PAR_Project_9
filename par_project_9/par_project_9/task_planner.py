#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PointStamped

class Package:
    PICKUP = 0
    DELIVERY = 1

class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')
        
        # ---------- Parameters ----------
        self.declare_parameter("pickup_markers", [0, 1, 2, 3])     # Example IDs
        self.declare_parameter("delivery_markers", [10, 11, 12, 13])
        self.declare_parameter("delivery_assignments", {
            0: 10,
            1: 11,
            2: 12,
            3: 13
        })
        self.declare_parameter("max_capacity", 2)
        
        # ---------- State ----------
        self.available_pickups = set(self.get_parameter("pickup_markers").value)
        self.available_deliveries = set(self.get_parameter("delivery_markers").value)
        self.carried_packages = {}  # pickup_id: delivery_id
        self.completed_deliveries = set()
        
        # Current target
        self.current_target = None
        self.current_goal_type = None
        
        # ---------- ROS Communications ----------
        # Publishers
        self.target_pub = self.create_publisher(Int32, 'target_marker_id', 10)
        self.status_pub = self.create_publisher(String, 'delivery_status', 10)
        
        # Subscribers
        self.create_subscription(Bool, 'marker_touched', self.touch_callback, 10)
        self.create_subscription(Int32, 'touched_marker_id', self.touch_id_callback, 10)
        
        # Initial planning
        self.create_timer(1.0, self.plan_next_action)
        
    def touch_id_callback(self, msg):
        """Handle touched marker ID"""
        marker_id = msg.data
        self.get_logger().info(f"Touched marker: {marker_id}")
        
        # Handle pickup
        if marker_id in self.available_pickups:
            delivery_id = self.get_parameter("delivery_assignments").value.get(marker_id)
            if delivery_id and len(self.carried_packages) < self.get_parameter("max_capacity").value:
                self.carried_packages[marker_id] = delivery_id
                self.available_pickups.remove(marker_id)
                self.get_logger().info(f"Picked up package at {marker_id}, deliver to {delivery_id}")
        
        # Handle delivery
        elif marker_id in self.available_deliveries:
            # Find which package this delivery corresponds to
            for pickup_id, delivery_id in self.carried_packages.items():
                if delivery_id == marker_id:
                    self.completed_deliveries.add(pickup_id)
                    del self.carried_packages[pickup_id]
                    self.get_logger().info(f"Delivered package from {pickup_id} to {delivery_id}")
                    break
    
    def plan_next_action(self):
        """Determine next target based on state"""
        # 1. Deliver carried packages if close to delivery point
        if self.carried_packages:
            # Simple strategy: deliver first available package
            for pickup_id, delivery_id in self.carried_packages.items():
                if delivery_id in self.available_deliveries:
                    self.set_target(delivery_id, Package.DELIVERY)
                    return
        
        # 2. Pick up new packages if capacity available
        if self.available_pickups and len(self.carried_packages) < self.get_parameter("max_capacity").value:
            pickup_id = next(iter(self.available_pickups))  # Simple: take first available
            self.set_target(pickup_id, Package.PICKUP)
            return
        
        # 3. Explore if no clear targets
        if not self.current_target:
            self.get_logger().info("No targets available, exploring...")
            # Implement exploration behavior here
            
    def set_target(self, marker_id, goal_type):
        """Set new target for visual servoing"""
        if marker_id == self.current_target:
            return
            
        self.current_target = marker_id
        self.current_goal_type = goal_type
        target_msg = Int32()
        target_msg.data = marker_id
        self.target_pub.publish(target_msg)
        
        goal_type_str = "PICKUP" if goal_type == Package.PICKUP else "DELIVERY"
        self.get_logger().info(f"New target: {marker_id} ({goal_type_str})")
        
    def publish_status(self):
        """Publish current delivery status"""
        status = (
            f"Packages carried: {len(self.carried_packages)}\n"
            f"Available pickups: {len(self.available_pickups)}\n"
            f"Completed deliveries: {len(self.completed_deliveries)}"
        )
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

def main():
    rclpy.init()
    node = TaskPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()