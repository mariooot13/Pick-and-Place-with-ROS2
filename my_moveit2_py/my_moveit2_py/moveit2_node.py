"""
Example of moving to a joint configuration.
`ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from my_moveit2_py.Moveit2_resources import MoveIt2
from my_moveit2_py import ur3e_model

from trajectory_msgs.msg import JointTrajectory

from geometry_msgs.msg import Pose

from functools import partial
from ur_msgs.srv import SetIO

import time

def main():

    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [-0.252, -0.118, 0.195]) #x (-0.500,-0.200) y (-0.350,0.000) z (0.050,0.200)
    node.declare_parameter("quat_xyzw",[0.570803357577419, 0.8205298265175397, -0.00518912143252199, 0.029789323459918908]
) #[0.164, 0.986, -0.029, -0.008] PINZA HACIA ABAJO NO CAMBIAR
    node.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur3e_model.joint_names(),
        base_link_name=ur3e_model.base_link_name(),
        end_effector_name=ur3e_model.end_effector_name(),
        group_name=ur3e_model.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    publisher_ = node.create_publisher(JointTrajectory, "trajectory_from_moveit2", 1)

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    
    a = moveit2.plan(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)

    node.get_logger().info(str(a))

    publisher_.publish(a)


    #moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
    #moveit2.wait_until_executed()   

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()

