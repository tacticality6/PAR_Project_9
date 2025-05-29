#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class TaskPlanningNode(Node):
    def __init__(self, node_name):
        self._node_name = node_name
        super().__init__(node_name)



def main(args=None):
    rclpy.init(args=args)

    task_planning_node = TaskPlanningNode("task_planning_node")

    try:
        rclpy.spin(task_planning_node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    task_planning_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()