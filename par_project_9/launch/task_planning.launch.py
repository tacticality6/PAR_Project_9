#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Task Planning Node
    task_planning_node = Node(
        package="par_project_9",
        executable="task_planning_node",
        name="task_planning_node",
        parameters=[{
            "navigation_timeout": 30.0,
            "pickup_timeout": 15.0,
            "delivery_timeout": 15.0,
            "decision_frequency": 2.0,
            "marker_timeout": 10.0,
            "use_tf_for_positions": True,
            "map_frame": "map",
            "base_frame": "base_link"
        }],
        output='screen',
    )

    # Assemble and return the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(task_planning_node)
    return ld
