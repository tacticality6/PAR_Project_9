#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
import numpy as np
import visp_ros
import visp
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge


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
        # Dummy initialization
        self.cam.initPersProjWithoutDistortion(600, 600, 320, 240)
        self.tracker.setCameraParameters(self.cam)

        # Set orange color parameters
        lower = np.array(self.get_parameter("pointer_hsv_lower").value)
        upper = np.array(self.get_parameter("pointer_hsv_upper").value)
        self.tracker.setHSVColorRange(
            visp.ColorHSV(lower[0], upper[0], lower[1], upper[1], lower[2], upper[2])
        )

        # ---------- ROS Communications ----------
        qos = QoSProfile(depth=10)

        # Publishers
        self.touch_pub = self.create_publisher(Bool, "marker_touched", qos)
        self.touch_id_pub = self.create_publisher(Int32, "touched_marker_id", qos)
        self.pointer_pub = self.create_publisher(PointStamped, "pointer_position", qos)

        # Subscribers
        self.create_subscription(
            Detection2DArray,
            "aruco_detections",
            self.marker_callback,
            qos)

        self.create_subscription(
            Image,
            "/oak/rgb/image_raw",
            self.image_callback,
            qos_profile_sensor_data)

        self.create_subscription(
            CameraInfo,
            "/oak/rgb/camera_info",
            self.camera_info_callback,
            qos)

        self.get_logger().info("Pointer Detector node starting up...")

    def camera_info_callback(self, msg):
        """Update camera parameters from calibration"""
        self.cam.initPersProjWithoutDistortion(
            msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.tracker.setCameraParameters(self.cam)
        self.get_logger().info("Updated camera parameters")

    def marker_callback(self, msg):
        """Handle marker detections"""
        self.current_markers = {}
        for detection in msg.detections:
            marker_id = int(detection.results[0].hypothesis.class_id)
            center = detection.bbox.center.position
            self.current_markers[marker_id] = (center.x, center.y)
            self.get_logger().debug(f"Marker {marker_id} at ({center.x}, {center.y})")

    def image_callback(self, msg):
        """Placeholder for image processing"""
        if not hasattr(self, 'bridge'):
            self.bridge = CvBridge()
        
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Detect pointer using ViSP
        self.tracker.track(cv_image)
        blobs = self.tracker.getBlob()

        if not blobs:
            return

        # Find largest the pointer
        largest_blob = max(blobs, key=lambda b: b.getArea())

        # Publish pointer position
        pointer_pos = PointStamped()
        pointer_pos.header = msg.header
        pointer_pos.point.x = float(largest_blob.getCog().x)
        pointer_pos.point.y = float(largest_blob.getCog().y)
        self.pointer_pub.publish(pointer_pos)


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