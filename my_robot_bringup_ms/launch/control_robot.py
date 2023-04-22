import sys
import os

from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():


    get_package_share_directory("my_robot_bringup_ms")
    
    config_parameters = PathJoinSubstitution(
        [FindPackageShare("my_robot_bringup_ms"), "/home/mario/workspace/ros_ur_driver/src/my_robot_bringup_ms/config", "param_bringup.yaml"]
    )
    
    try:
        get_package_share_directory("my_func_nodes")
    except PackageNotFoundError:
        print(
            "ERROR:"
            "Could not find the package" 
        )
        sys.exit(1)

    get_package_share_directory("ur_bringup")
    
    a = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_bringup'),
                    "/home/mario/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/launch",'ur_control.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': 'ur3e',
                'robot_ip': '192.168.20.35',
                'use_fake_hardware':'false',
                'launch_rviz':'true'#,
                #'initial_joint_controller':'joint_trajectory_controller'
            }.items()
        )

    moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_bringup'),
                    "/home/mario/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/launch",'ur_moveit.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': 'ur3e',
                'robot_ip': '192.168.20.35',
                'launch_rviz':'true'
            }.items()
        )
    
    
    
    control_robot_node = Node(
                package="my_func_nodes",
                executable="control_robot_exec",
                name="control_robot_master",
                parameters=[config_parameters],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )

    nodes_to_start = [
    	control_robot_node,
        a,
        moveit,
    	
    ]   

    return LaunchDescription(nodes_to_start)
