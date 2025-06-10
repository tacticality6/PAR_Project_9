#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import math
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PointStamped, Point
from vision_msgs.msg import Detection2DArray
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time

class PointerDetector(Node):
    def __init__(self):
        super().__init__("pointer_detector")
        
        # ---------- Parameters ----------
        self.declare_parameter("touch_threshold", 0.15)  # meters
        self.declare_parameter("marker_timeout", 5.0)    # seconds
        self.declare_parameter("pointer_offset_x", 0.0)  # Relative to base_link
        self.declare_parameter("pointer_offset_y", 0.0)
        self.declare_parameter("pointer_offset_z", 0.0)
        self.declare_parameter("camera_frame", "oak_rgb_camera_optical_frame")
        self.declare_parameter("base_frame", "base_link")
        
        # Get parameters
        self.touch_threshold = self.get_parameter("touch_threshold").value
        self.marker_timeout = self.get_parameter("marker_timeout").value
        self.offset_x = self.get_parameter("pointer_offset_x").value
        self.offset_y = self.get_parameter("pointer_offset_y").value
        self.offset_z = self.get_parameter("pointer_offset_z").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        
        # TF initialization
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Marker storage: {marker_id: (x, y, z, last_seen)}
        self.markers = {}
        
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
            
        # Timer for touch detection
        self.touch_timer = self.create_timer(0.1, self.check_touch_3d)
        
        self.get_logger().info("Pointer Detector initialized")
        self.get_logger().info(f"Using pointer offset: X={self.offset_x}, Y={self.offset_y}, Z={self.offset_z}")
        self.get_logger().info(f"Touch threshold: {self.touch_threshold}m")

    def marker_callback(self, msg):
        """Update marker positions from detections"""
        current_time = self.get_clock().now()
        
        for detection in msg.detections:
            marker_id = int(detection.results[0].hypothesis.class_id)
            
            try:
                # Get marker position in camera frame
                transform = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    f"aruco_{marker_id}",
                    rclpy.time.Time())
                
                t = transform.transform.translation
                self.markers[marker_id] = (t.x, t.y, t.z, current_time)
                
                self.get_logger().debug(f"Marker {marker_id} at ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")
                
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"TF error for marker {marker_id}: {e}")

    def check_touch_3d(self):
        """Check 3D distance between pointer and markers"""
        current_time = self.get_clock().now()
        
        # Clean up old markers
        stale_markers = []
        for marker_id, (_, _, _, last_seen) in self.markers.items():
            if (current_time - last_seen).nanoseconds > self.marker_timeout * 1e9:
                stale_markers.append(marker_id)
        
        for marker_id in stale_markers:
            del self.markers[marker_id]
            self.get_logger().info(f"Removed stale marker {marker_id}")
        
        # Get pointer position in camera frame
        try:
            # Get base_link position in camera frame
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.base_frame,
                rclpy.time.Time())
            
            # Apply pointer offset relative to base_link
            base_x = transform.transform.translation.x
            base_y = transform.transform.translation.y
            base_z = transform.transform.translation.z
            
            # Create a point in base_link frame
            pointer_pos = Point()
            pointer_pos.x = base_x + self.offset_x
            pointer_pos.y = base_y + self.offset_y
            pointer_pos.z = base_z + self.offset_z
            
            # Publish pointer position for visualization/debugging
            ptr_msg = PointStamped()
            ptr_msg.header.stamp = self.get_clock().now().to_msg()
            ptr_msg.header.frame_id = self.camera_frame
            ptr_msg.point = pointer_pos
            self.pointer_pub.publish(ptr_msg)
            
            # Check distance to each marker
            for marker_id, (marker_x, marker_y, marker_z, _) in self.markers.items():
                distance = math.sqrt(
                    (pointer_pos.x - marker_x)**2 +
                    (pointer_pos.y - marker_y)**2 +
                    (pointer_pos.z - marker_z)**2
                )
                
                if distance < self.touch_threshold:
                    self.get_logger().info(f"Touch detected on marker {marker_id} (distance: {distance:.3f}m)")
                    
                    # Publish touch event
                    touch_msg = Bool()
                    touch_msg.data = True
                    self.touch_pub.publish(touch_msg)
                    
                    id_msg = Int32()
                    id_msg.data = marker_id
                    self.touch_id_pub.publish(id_msg)
                    
                    # Remove marker after touch to prevent multiple detections
                    if marker_id in self.markers:
                        del self.markers[marker_id]
                    return  # Only register one touch per cycle
                    
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF error: {e}", throttle_duration_sec=5)

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