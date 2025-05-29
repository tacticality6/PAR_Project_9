#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

class PointerDetector(Node):
    def __init__(self):
        super().__init__("pointer_detector")

        # ---------- Parameters ----------
        self.declare_parameter("pointer_hsv_lower", [5, 100, 100])
        self.declare_parameter("pointer_hsv_upper", [15, 255, 255])
        self.declare_parameter("touch_threshold", 0.15)  # meters
        self.declare_parameter("blob_size_threshold", 500)  # pixels

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