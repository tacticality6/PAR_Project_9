#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2, numpy as np
from cv_bridge import CvBridge
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
=======
from sensor_msgs.msg import Image, CameraInfo
>>>>>>> 4fac40f (feat: implement aruco detect)
=======
from sensor_msgs.msg import CameraInfo, CompressedImage
>>>>>>> ae945db (bug)
=======
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
>>>>>>> 926bf52 (feat: impleent task controller to receive aruco detector and creating marker map)
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

<<<<<<< HEAD
from geometry_msgs.msg import PointStamped

=======
>>>>>>> 4fac40f (feat: implement aruco detect)
class ArucoDetector(Node):

    def __init__(self):
        super().__init__("aruco_detector")

        # ---------- params ----------
<<<<<<< HEAD
<<<<<<< HEAD
        self.DEBUG_MODE = True # set to True to enable debug overlay
        
        # which camera topics to listen to
        self.declare_parameter("image_topic", "/oak/rgb/image_raw/compressed")
=======
=======
        self.DEBUG_MODE = True # set to True to enable debug overlay
        
>>>>>>> 926bf52 (feat: impleent task controller to receive aruco detector and creating marker map)
        # which camera topics to listen to
<<<<<<< HEAD
        self.declare_parameter("image_topic", "/oak/rgb/image_raw")
>>>>>>> 4fac40f (feat: implement aruco detect)
=======
        self.declare_parameter("image_topic", "/oak/rgb/image_raw/compressed")
>>>>>>> ae945db (bug)
        self.declare_parameter("info_topic", "/oak/rgb/camera_info")
        # aruco marker parameters
        self.declare_parameter("marker_size_m", 0.06) # 6 cm
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("publish_tf", True)
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> ae945db (bug)
        
        # **immediately read them back and log them**
        image_topic   = self.get_parameter("image_topic").value
        info_topic    = self.get_parameter("info_topic").value
        marker_size   = self.get_parameter("marker_size_m").value
        aruco_dict    = self.get_parameter("aruco_dict").value
        publish_tf    = self.get_parameter("publish_tf").value

        self.get_logger().info(f"→ image_topic:   {image_topic}")
        self.get_logger().info(f"→ info_topic:    {info_topic}")
        self.get_logger().info(f"→ marker_size_m: {marker_size}")
        self.get_logger().info(f"→ aruco_dict:    {aruco_dict}")
        self.get_logger().info(f"→ publish_tf:    {publish_tf}")
<<<<<<< HEAD
<<<<<<< HEAD
        self.get_logger().info(f"→ DEBUG_MODE:" + (" ON" if self.DEBUG_MODE else " OFF"))
=======
>>>>>>> 4fac40f (feat: implement aruco detect)
=======
>>>>>>> ae945db (bug)
=======
        self.get_logger().info(f"→ DEBUG_MODE:" + (" ON" if self.DEBUG_MODE else " OFF"))
>>>>>>> 926bf52 (feat: impleent task controller to receive aruco detector and creating marker map)

        # ---------- internal ----------
        self._dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self.get_parameter("aruco_dict").value))
        self._bridge = CvBridge() # to convert ROS images to OpenCV
        self._camera_matrix = None
        self._dist_coeffs = None

        # pubs/subs
        self._tfbr = TransformBroadcaster(self)
<<<<<<< HEAD
<<<<<<< HEAD
        self._pub = self.create_publisher(Detection2DArray, "aruco_detections", 10)
        self._overlay_pub = self.create_publisher(Image,  "aruco/overlay", 10)

        self.create_subscription(CameraInfo,
                                self.get_parameter("info_topic").value,
                                self.info_cb, 10)

        self.create_subscription(CompressedImage,
                                self.get_parameter("image_topic").value,
                                self.image_cb, qos_profile_sensor_data)
        
        # Add marker position publisher
        self.marker_pos_pub = self.create_publisher(
            PointStamped, 
            "marker_position", 
            10)
        
=======
        self._pub = self.create_publisher(Detection2DArray,
                                           "aruco_detections", 10)
=======
        self._pub = self.create_publisher(Detection2DArray, "aruco_detections", 10)
<<<<<<< HEAD
        self._overlay_pub = self.create_publisher(CompressedImage,  "aruco/overlay", 10)
>>>>>>> ae945db (bug)
=======
        self._overlay_pub = self.create_publisher(Image,  "aruco/overlay", 10)
>>>>>>> 926bf52 (feat: impleent task controller to receive aruco detector and creating marker map)

        self.create_subscription(CameraInfo,
                                self.get_parameter("info_topic").value,
                                self.info_cb, 10)

        self.create_subscription(CompressedImage,
<<<<<<< HEAD
                                 self.get_parameter("image_topic").value,
                                 self.image_cb, qos_profile_sensor_data)

>>>>>>> 4fac40f (feat: implement aruco detect)
=======
                                self.get_parameter("image_topic").value,
                                self.image_cb, qos_profile_sensor_data)
        
>>>>>>> 926bf52 (feat: impleent task controller to receive aruco detector and creating marker map)
        self.get_logger().info("\n 👻 💀 ☠️ 👽 👾 🤖 🦾 🦿 ArucoDetector ready to roll! 🤮 🤢 🤧 🤒 🤕 😭 😤 😵")


    # ---------- callbacks ----------
    def info_cb(self, msg: CameraInfo):
        if self._camera_matrix is None:
            k = np.array(msg.k).reshape(3, 3)
            d = np.array(msg.d)
            self._camera_matrix, self._dist_coeffs = k, d
            self.get_logger().info("Camera received")
<<<<<<< HEAD
<<<<<<< HEAD
            
    def image_cb(self, msg: CompressedImage):
        if self._camera_matrix is None:
            return  # wait for calibration

        # --- decompress JPEG/PNG bytes ---
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)   # BGR image
            
        if img is None:
            self.get_logger().info("Failed to decode compressed image")
            return
        
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(img, self._dict)

        n = 0 if ids is None else len(ids)
        if n > 0:
            self.get_logger().info(f"detectMarkers() -> {n} markers")
        
        if n and (self.get_clock().now().nanoseconds // 1e9) % 1 == 0:
            self.get_logger().info(f"IDs: {ids.flatten()[:4]} ...")
    
        if ids is None:
            # self.get_logger().info("\n 👻 no overlay published")
=======

    def image_cb(self, msg: Image):
=======
            
    def image_cb(self, msg: CompressedImage):
>>>>>>> ae945db (bug)
        if self._camera_matrix is None:
            return  # wait for calibration

        # --- decompress JPEG/PNG bytes ---
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)   # BGR image
            
        if img is None:
            self.get_logger().info("Failed to decode compressed image")
            return
        
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(img, self._dict)

        n = 0 if ids is None else len(ids)
        if n > 0:
            self.get_logger().info(f"detectMarkers() -> {n} markers")
        
        if n and (self.get_clock().now().nanoseconds // 1e9) % 1 == 0:
            self.get_logger().info(f"IDs: {ids.flatten()[:4]} ...")
    
        if ids is None:
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 4fac40f (feat: implement aruco detect)
=======
            self.get_logger().info("\n 👻 no overlay published")
>>>>>>> ae945db (bug)
=======
            # self.get_logger().info("\n 👻 no overlay published")
>>>>>>> 926bf52 (feat: impleent task controller to receive aruco detector and creating marker map)
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
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 926bf52 (feat: impleent task controller to receive aruco detector and creating marker map)
            self.get_logger().info(f"Markers ID -> {marker_id} detected")
            
            oh = ObjectHypothesisWithPose()
            oh.hypothesis.class_id = str(marker_id)
            oh.hypothesis.score = 1.0 # dummy score
            
            det.results.append(oh)
            det.bbox.center.position.x = float(np.mean(corners[i][0][:, 0]))
            det.bbox.center.position.y = float(np.mean(corners[i][0][:, 1]))
            # theta is still top‐level on Pose2D
<<<<<<< HEAD
            det.bbox.center.theta = 0.0
            det.bbox.size_x = det.bbox.size_y = 1.0   # pixels, not used
            det_array.detections.append(det)
            print(f"Detected marker {marker_id} at {det.bbox.center.position.x:.2f}, {det.bbox.center.position.y:.2f}")

            ## 2. TF
=======
            det.results.append(ObjectHypothesisWithPose(
                id=marker_id, score=1.0))           # score dummy =1
            det.bbox.center.x = np.mean(corners[i][0][:, 0])
            det.bbox.center.y = np.mean(corners[i][0][:, 1])
=======
>>>>>>> 926bf52 (feat: impleent task controller to receive aruco detector and creating marker map)
            det.bbox.center.theta = 0.0
            det.bbox.size_x = det.bbox.size_y = 1.0   # pixels, not used
            det_array.detections.append(det)
            print(f"Detected marker {marker_id} at {det.bbox.center.position.x:.2f}, {det.bbox.center.position.y:.2f}")

<<<<<<< HEAD
            ## 2. TF (optional)
>>>>>>> 4fac40f (feat: implement aruco detect)
=======
            ## 2. TF
>>>>>>> 926bf52 (feat: impleent task controller to receive aruco detector and creating marker map)
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
        
        # ─── draw debug overlay ──────────────────────────────────────────────
<<<<<<< HEAD
        debug_img = img.copy()
        # 1. draw marker borders + ids
        cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)

        # 2. draw axes for each pose (length = marker_size / 2)
        axis_len = self.get_parameter("marker_size_m").value * 0.5
        for rvec, tvec in zip(rvecs, tvecs):
            cv2.aruco.drawAxis(
                debug_img,
                self._camera_matrix,
                self._dist_coeffs,
                rvec, tvec, axis_len)

        # 3. convert back to ROS Image and publish
        overlay_msg = self._bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
        overlay_msg.header = msg.header          # keep the same stamp/frame
        self._overlay_pub.publish(overlay_msg)

<<<<<<< HEAD

            # Publish marker position
            marker_pos = PointStamped()
            marker_pos.header = msg.header
            marker_pos.point.x = float(np.mean(corners[i][0][:, 0]))
            marker_pos.point.y = float(np.mean(corners[i][0][:, 1]))
            self.marker_pos_pub.publish(marker_pos)
        
        # ─── draw debug overlay ──────────────────────────────────────────────
=======
>>>>>>> 926bf52 (feat: impleent task controller to receive aruco detector and creating marker map)
        if self.DEBUG_MODE:
            debug_img = img.copy()
            # 1. draw marker borders + ids
            cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)

            # 2. draw axes for each pose (length = marker_size / 2)
            axis_len = self.get_parameter("marker_size_m").value * 0.5
            for rvec, tvec in zip(rvecs, tvecs):
                cv2.aruco.drawAxis(
                    debug_img,
                    self._camera_matrix,
                    self._dist_coeffs,
                    rvec, tvec, axis_len)
            
            # 3. Annotate each marker ID in text
            for idx, marker_id in enumerate(ids.flatten()):
                # compute a nice spot (we’ll use the center of the marker)
                c = corners[idx][0]
                cX = int(c[:, 0].mean())
                cY = int(c[:, 1].mean())
                cv2.putText(
                    debug_img,
                    str(marker_id),
                    (cX - 10, cY + 10),                      # offset so the text sits inside the square
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,                                     # font scale
                    (0, 0, 255),                             # red color in BGR
                    2                                        # line thickness
                )

            # 4. convert back to ROS Image and publish
            overlay_msg = self._bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            overlay_msg.header = msg.header # keep the same stamp/frame
            self._overlay_pub.publish(overlay_msg)
        # ─── end draw debug overlay ──────────────────────────────────────────────

        # publish the detection array
<<<<<<< HEAD
=======
>>>>>>> 4fac40f (feat: implement aruco detect)
=======
>>>>>>> 926bf52 (feat: impleent task controller to receive aruco detector and creating marker map)
        self._pub.publish(det_array)

def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
<<<<<<< HEAD
<<<<<<< HEAD
    main()
=======
    main()
>>>>>>> 4fac40f (feat: implement aruco detect)
=======
    main()
>>>>>>> 42bb00957eec8eb231846315678a69ca5002b316
