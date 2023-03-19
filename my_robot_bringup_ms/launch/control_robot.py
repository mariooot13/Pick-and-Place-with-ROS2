import sys
import os

from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def generate_launch_description():

    """config_parameters = PathJoinSubstitution(
        [FindPackageShare("my_robot_bringup_ms"), "config", "param_bringup.yaml"]
    )"""
    
    moveit2_UR = Node(
        package="my_moveit2_py",
        executable="executable_node_moveit", #dentro de moveit2_py
    )
    
    """control_robot = Node(
        package="my_func_nodes",
        executable="control_robot_exec",
        name="control_robot_master",
        parameters=[config_parameters],
        output={"stdout": "screen", "stderr": "screen"},
    )"""

    nodes_to_start = [
    	moveit2_UR,
    	#control_robot,
    ]   

    return LaunchDescription(nodes_to_start)
