
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from std_srvs.srv import Trigger
from std_msgs.msg import Header
from cv_bridge import CvBridge
import message_filters
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs
from par_project_9_interfaces.srv import MarkerConfirmation
from par_project_9_interfaces.msg import MarkerPointStamped

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')
        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_received = False
        self.marker_pose = None

        # Subscriptions
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/color/camera_info')

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.camera_info_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.rgbd_callback)

        self.marker_sub = self.create_subscription(
            MarkerPointStamped, '/marker_point', self.marker_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.touch_client = self.create_client(Trigger, '/touch_confirm')
        while not self.touch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /touch_confirm service...')

    def rgbd_callback(self, rgb_msg, depth_msg, cam_info_msg):
        if not self.marker_received:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        x, y = int(self.marker_pose.x), int(self.marker_pose.y)
        if y >= depth_image.shape[0] or x >= depth_image.shape[1]:
            self.get_logger().warn('Marker point out of depth image bounds.')
            return

        depth = depth_image[y, x] / 1000.0  # Convert mm to meters
        if np.isnan(depth) or depth <= 0.0:
            self.get_logger().warn("Invalid depth at marker location")
            return

        fx = cam_info_msg.k[0]
        fy = cam_info_msg.k[4]
        cx = cam_info_msg.k[2]
        cy = cam_info_msg.k[5]

        # Deproject pixel to 3D
        X = (x - cx) * depth / fx
        Y = (y - cy) * depth / fy
        Z = depth

        camera_point = PointStamped()
        camera_point.header = depth_msg.header
        camera_point.point.x = X
        camera_point.point.y = Y
        camera_point.point.z = Z

        try:
            target_point = self.tf_buffer.transform(camera_point, 'base_link', timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error(f"Transform error: {e}")
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        pose.pose.position = target_point.point
        pose.pose.orientation.w = 1.0  # No rotation for simplicity

        self.pose_pub.publish(pose)
        self.get_logger().info(f"Published goal pose at ({target_point.point.x:.2f}, {target_point.point.y:.2f}, {target_point.point.z:.2f})")

        self.call_touch_confirm()

        self.marker_received = False  # Reset until next marker

    def marker_callback(self, msg):
        self.marker_pose = msg.marker.point
        self.marker_received = True
        self.get_logger().info(f"Received marker point at ({self.marker_pose.x}, {self.marker_pose.y})")

    def call_touch_confirm(self):
        req = Trigger.Request()
        future = self.touch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info("Touch confirmed")
        else:
            self.get_logger().warn("Touch failed or not confirmed")

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
