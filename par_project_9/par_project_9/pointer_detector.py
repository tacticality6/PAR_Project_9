#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

class PointerDetector(Node):
    def __init__(self):
        super().__init__("pointer_detector")
        self.get_logger().info("🚦 Pointer Detector node starting up...")

def main():
    rclpy.init()
    node = PointerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()