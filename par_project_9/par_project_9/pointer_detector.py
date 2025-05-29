#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
import numpy as np
import visp_ros
import visp

class PointerDetector(Node):
    def __init__(self):
        super().__init__("pointer_detector")

        # ---------- Parameters ----------
        self.declare_parameter("pointer_hsv_lower", [5, 100, 100])
        self.declare_parameter("pointer_hsv_upper", [15, 255, 255])
        self.declare_parameter("touch_threshold", 0.15)  # meters
        self.declare_parameter("blob_size_threshold", 500)  # pixels

        # ---------- ViSP Initialization ----------
        self.tracker = visp_ros.BlobTracker()
        self.cam = visp.CameraParameters()
        # Dummy initialization, will be updated by camera_info_callback
        self.cam.initPersProjWithoutDistortion(600, 600, 320, 240)
        self.tracker.setCameraParameters(self.cam)

        # Set orange color parameters
        lower = np.array(self.get_parameter("pointer_hsv_lower").value)
        upper = np.array(self.get_parameter("pointer_hsv_upper").value)
        self.tracker.setHSVColorRange(
            visp.ColorHSV(lower[0], upper[0], lower[1], upper[1], lower[2], upper[2])
        )

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