#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2, numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ArucoDetector(Node):

    def __init__(self):
        super().__init__("aruco_detector")

        # ---------- params ----------
        # which camera topics to listen to
        self.declare_parameter("image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("info_topic", "/oak/rgb/camera_info")
        # aruco marker parameters
        self.declare_parameter("marker_size_m", 0.06) # 6 cm
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("publish_tf", True)

        # ---------- internal ----------
        self._dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self.get_parameter("aruco_dict").value))
        self._bridge = CvBridge() # to convert ROS images to OpenCV
        self._camera_matrix = None
        self._dist_coeffs = None

        # pubs/subs
        self._tfbr = TransformBroadcaster(self)
        self._pub = self.create_publisher(Detection2DArray,
                                           "aruco_detections", 10)

        self.create_subscription(CameraInfo,
                                 self.get_parameter("info_topic").value,
                                 self.info_cb, 10)

        self.create_subscription(Image,
                                 self.get_parameter("image_topic").value,
                                 self.image_cb, qos_profile_sensor_data)

        self.get_logger().info("\n 👻 💀 ☠️ 👽 👾 🤖 🦾 🦿 ArucoDetector ready to roll! 🤮 🤢 🤧 🤒 🤕 😭 😤 😵")


    # ---------- callbacks ----------
    def info_cb(self, msg: CameraInfo):
        if self._camera_matrix is None:
            k = np.array(msg.k).reshape(3, 3)
            d = np.array(msg.d)
            self._camera_matrix, self._dist_coeffs = k, d
            self.get_logger().info("Camera received")

    def image_cb(self, msg: Image):
        if self._camera_matrix is None:
            return  # wait for calibration

        img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, _ = cv2.aruco.detectMarkers(img, self._dict)

        if ids is None:
            return # nothing this frame

        # pose
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.get_parameter("marker_size_m").value,
            self._camera_matrix,
            self._dist_coeffs)

        det_array = Detection2DArray()
        det_array.header = msg.header

        for i, marker_id in enumerate(ids.flatten()):
            ## 1. Detection message
            det = Detection2D()
            det.results.append(ObjectHypothesisWithPose(
                id=marker_id, score=1.0))           # score dummy =1
            det.bbox.center.x = np.mean(corners[i][0][:, 0])
            det.bbox.center.y = np.mean(corners[i][0][:, 1])
            det.bbox.center.theta = 0.0
            det.bbox.size_x = det.bbox.size_y = 1.0 # pixels, ignored
            det_array.detections.append(det)

            ## 2. TF (optional)
            if self.get_parameter("publish_tf").value:
                t = TransformStamped()
                t.header = msg.header
                t.child_frame_id = f"aruco_{marker_id}"
                t.transform.translation.x = float(tvecs[i][0][0])
                t.transform.translation.y = float(tvecs[i][0][1])
                t.transform.translation.z = float(tvecs[i][0][2])

                # rvec → quaternion
                rmat,_ = cv2.Rodrigues(rvecs[i])
                # cheap conversion because Rⱼⱼ small; use scipy if installed
                qw = np.sqrt(1 + rmat[0,0] + rmat[1,1] + rmat[2,2]) / 2
                qx = (rmat[2,1] - rmat[1,2]) / (4*qw)
                qy = (rmat[0,2] - rmat[2,0]) / (4*qw)
                qz = (rmat[1,0] - rmat[0,1]) / (4*qw)
                t.transform.rotation.x, t.transform.rotation.y, \
                t.transform.rotation.z, t.transform.rotation.w = qx,qy,qz,qw
                self._tfbr.sendTransform(t)

        self._pub.publish(det_array)

def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
