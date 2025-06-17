import rclpy
from rclpy import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from collections import OrderedDict
import numpy as np
import math


class ExplorationNode(Node):
    def __init__(self):
        super().__init__("exploration_node")

        #Parameters
        self.map_topic = self.declare_parameter("map_topic", "/map").get_parameter_value().string_value
        self.nav_action = self.declare_parameter("nav_action", "navigate_to_pose").get_parameter_value().string_value
        self.pose_topic = self.declare_parameter("pose_topic", "/odom").get_parameter_value().string_value
        self.trigger_topic = self.declare_parameter("trigger_topic", "/explore_trigger").get_parameter_value().string_value
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
        
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            self.pose_topic, 
            self.pose_callback, 
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
        self.current_pose = None
        self.viewed_cells = OrderedDict()

    def map_callback(self, msg):
        self.map = msg
    
    def pose_callback(self, msg):
        self.current_pose = msg

    def trigger_callback(self, msg):
        self.active = msg.data
    

    
    def explore_tick(self):
        if self.map is None or self.current_pose is None or not self.nav_client.wait_for_server(timeout_sec=1.0):
            return
        
        self.forget_old_walls()
        self.update_memory()

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

    def update_memory(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]

        _, _, yaw = euler_from_quaternion(
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        )

        for angle in np.linspace(-self.fov/2, self.fov/2, num=20):
            theta = yaw + angle
            for r in np.linspace(0, self.vision_range, num=20):
                x = self.current_pose.pose.position.x + r * math.cos(theta)
                y = self.current_pose.pose.position.y + r * math.sin(theta)
                mx, my = self.world_to_map(x, y)

                if not self.valid_map_index(mx, my):
                    break

                index = my * self.map.info.width + mx
                if self.map.data[index] > 50:
                    self.viewed_cells[(mx, my)] = now
                    break
                elif self.map.data[index] == -1:
                    break

    def valid_map_index(self,x,y):
        return 0 <= x < self.map.info.width and 0 <= y < self.map.info.height

    def world_to_map(self,x,y):
        mx = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((y - self.map.info.origin.position.y) / self.map.info.resolution)
        return mx, my

    def choose_next_goal(self):
        if self.map is None or self.current_pose is None:
            return

        res = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y

        # Identify frontier cells (adjacent to unknown but near free space)
        frontier_cells = []
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if idx >= len(self.map.data):
                    continue
                val = self.map.data[idx]
                if val == 0:
                    for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            nidx = ny * width + nx
                            if self.map.data[nidx] == -1 or (self.map.data[nidx] > 50 and (nx, ny) not in self.viewed_cells):
                                frontier_cells.append((x, y))
                                break

        if not frontier_cells:
            self.get_logger().info("No frontier cells found.")
            return

        # Convert current pose to map coords
        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y
        rx = int((robot_x - origin_x) / res)
        ry = int((robot_y - origin_y) / res)

        # Choose closest frontier cell
        frontier_cells.sort(key=lambda c: math.hypot(c[0] - rx, c[1] - ry))
        chosen_cell = frontier_cells[0]

        goal_pose = self.compute_goal_pose(chosen_cell)

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.nav_client.send_goal_async(goal)

    def compute_goal_pose(self, goal_cell):
        mx, my = goal_cell
        angle_sum = 0.0
        count = 0
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = mx + dx, my + dy
            if 0 <= nx < self.map.info.width and 0 <= ny < self.map.info.height:
                idx = ny * self.map.info.width + nx
                if idx >= 0 and idx < len(self.map.data):
                    if self.map.data[idx] == 0:
                        angle_sum += math.atan2(dy, dx)
                        count += 1
        if count == 0:
            yaw = 0.0
        else:
            yaw = angle_sum / count

        distance = 0.8
        gx = (mx + 0.5) * self.map.info.resolution + self.map.info.origin.position.x
        gy = (my + 0.5) * self.map.info.resolution + self.map.info.origin.position.y
        px = gx - distance * math.cos(yaw)
        py = gy - distance * math.sin(yaw)

        quat = quaternion_from_yaw(yaw)
        pose = PoseStamped()
        pose.header.frame_id = self.map.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose



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

def quaternion_from_yaw(yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (qx, qy, qz, qw)


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