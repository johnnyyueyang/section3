#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the constant_control node
    constant_control_node = Node(
        package="s3_basic",                  
        executable="constant_control.py",         
        name="constant_control_node",         
        output="screen",                     
        parameters=[{"use_sim_time": True}]  
    )

    # Path to the RViz configuration file
    rviz_config = PathJoinSubstitution([
        FindPackageShare("s3_basic"),
        "rviz",
        "section3.rviz"
    ])

    # Define the RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    # Return the LaunchDescription with both nodes
    return LaunchDescription([
        constant_control_node,
        rviz_node
    ])
