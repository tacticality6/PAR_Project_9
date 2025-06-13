import rclpy
from rclpy import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from collections import OrderedDict
import numpy as np


class ExplorationNode(Node):
    def __init__(self):
        super().__init__("exploration_node")

        #Parameters
        self.map_topic = self.declare_parameter("map_topic", "/map").get_parameter_value().string_value
        self.nav_action = self.declare_parameter("nav_action", "navigate_to_pose").get_parameter_value().string_value
        self.trigger_topic = self.declare_parameter("trigger_topic", "").get_parameter_value().string_value
        self.memory_duration = self.declare_parameter("memory_duration", 60.0).get_parameter_value().double_value
        self.explore_decision_freq = self.declare_parameter("explore_decision_freq", 2.0).get_parameter_value().double_value
        self.fov = self.declare_parameter("fov_radians", 1.0).get_parameter_value().double_value
        self.vision_range = self.declare_parameter("vision_range", 2.0).get_parameter_value().double_value

        #ROS Listeners & Publishers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.trigger_sub = self.create_subscription(
            Bool,
            self.trigger_topic,
            self.trigger_callback,
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.explore_timer = self.create_timer(
            self.explore_decision_freq,
            self.explore_tick
        )

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)

        #Local Variables
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        
        self.active = False
        self.map = None
        self.viewed_cells = OrderedDict()

    def map_callback(self, msg):
        self.map = msg
    
    def trigger_callback(self, msg):
        self.active = msg.data

    
    def explore_tick(self):
        if self.map is None or not self.nav_client.wait_for_server(timeout_sec=1.0):
            return
        
        self.forget_old_walls()

        #get robot pose
        try:
            tf = self.tf_buffer.lookup_transform(self.map.header.frame_id, 'base_link', self.get_clock().now().seconds_nanoseconds()[0])
            robot_x = tf.transform.translation.x
            robot_y = tf.transform.translation.y
            q = tf.transform.rotation
            _,_, yaw = euler_from_quaternion(q.x,q.y,q.z,q.w)
        except Exception as e:
            self.get_logger.warn(f"TF lookup failed: {e}")
            return
    
        self.update_memory(robot_x, robot_y, yaw)

        if self.active:
            self.choose_next_goal()

    
    def forget_old_walls(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]

        keys_to_delete = []

        for key, ts in self.viewed_cells.items():
            if now - ts > self.memory_duration:
                keys_to_delete.append(key)

        for key in keys_to_delete:
            del self.viewed_cells[key]

    def update_memory(self, robot_x, robot_y, robot_yaw):
        pass

    def can_see(self):
        pass

    def choose_next_goal(self):
        pass




def euler_from_quaternion(x,y,z,w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw




def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()