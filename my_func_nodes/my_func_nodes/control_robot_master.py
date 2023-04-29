import asyncio
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

import time

from threading import Thread

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from functools import partial
from ur_msgs.srv import SetIO

from my_moveit2_py.Moveit2_resources import MoveIt2 #own resources
from my_moveit2_py import ur3e_model #own resources

from rclpy.callback_groups import ReentrantCallbackGroup

class control_robot_master(Node):
    def __init__(self):
        super().__init__("control_robot_master")
        
        # Declare all parameters
        self.declare_parameter("controller_name", "joint_trajectory_position_controller")
        self.declare_parameter("joints")
        self.declare_parameter("check_starting_point", False)
        self.declare_parameter("starting_point_limits")

        # Read parameters
        self.pos_wanted = JointTrajectory()
        controller_name = self.get_parameter("controller_name").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}
        self.get_logger().info(
            'LETSGO')
        self.inp = None    
        #self.callback_group = ReentrantCallbackGroup()
	
	#Subscribers & Publishers
        self.pose_required = Pose()
        self.subscriber_camera = self.create_subscription(Pose, "posicion_pedida",self.callback_recibo_pos_pedida, 10)
        
   
	#Safety joints
        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        # starting point stuff
        if self.check_starting_point:
            # declare nested params
            for name in self.joints:
                param_name_tmp = "starting_point_limits" + "." + name
                self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param_name_tmp).value

            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise Exception('"starting_point" parameter is not set correctly!')
            self.joint_state_sub = self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 10
            )

            
        # Initialize starting point status
        if not self.check_starting_point:
            self.starting_point_ok = True
        else:
            self.starting_point_ok = False

        self.joint_state_msg_received = False


        publish_topic = "/" + controller_name + "/" + "joint_trajectory"
	
        #self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        #self.timer = self.create_timer(4, self.timer_callback)


    #PINZA:
    
    #prueba de servicio para la pinza.
    def call_gripper(self, fun, pin, state):
            client = self.create_client(SetIO, "/io_and_status_controller/set_io") #servicio de la pinza ya creado.
        
            while not client.wait_for_service(1.0):
	            self.get_logger().warn("Waiting for Server SetIO for the gripper to be opened...")
        
            request = SetIO.Request()
            request.fun = fun
            request.pin = pin
            request.state = state
        
            future = client.call_async(request)
            future.add_done_callback(
		        partial(self.callback_gripper, fun=fun, pin=pin, state=state))
		
    def callback_gripper(self, future, fun, pin, state):
    	try:
      	    response = future.result()
            #self.get_logger().info('The response of the gripper is:' + str(response.success) + '.')

    	except Exception as e:
	        self.get_logger().error("Service reset call failed %r" % (e,))

    #The pin can varies with the robot

    def close_gripper(self):
        self.call_gripper(1, 17, 24.0)
        time.sleep(0.5)
        self.call_gripper(1,17,0.0) #Hay que descargar la pinza.
    
    def open_gripper(self):
        self.call_gripper(1, 16, 24.0)
        time.sleep(0.5)
        self.call_gripper(1,16,0.0) #Hay que descargar la pinza.
        
    
    #POSICION DE LA CAMARA

    def pose_required_print(self):

        if self.pose_required is not None:
            return self.pose_required
    
    def callback_recibo_pos_pedida(self, msg):
        self.pose_required.position.x = msg.position.x
        self.pose_required.position.y = msg.position.y
        self.pose_required.position.z = msg.position.z
   
        self.pose_required.orientation.x = msg.orientation.x
        self.pose_required.orientation.y = msg.orientation.y
        self.pose_required.orientation.z = msg.orientation.z
        self.pose_required.orientation.w = msg.orientation.w

        self.get_logger().info(f"Position received: [{self.pose_required.position.x},{self.pose_required.position.y},{self.pose_required.position.z}], quat_xyzw: [{self.pose_required.orientation.x},{self.pose_required.orientation.y},{self.pose_required.orientation.z},{self.pose_required.orientation.w}]")


    def joint_state_callback(self, msg): #Joint state supervisor

        if not self.joint_state_msg_received:

            # check start state
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):
                    self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
                    limit_exceeded[idx] = True

            if any(limit_exceeded):
                self.starting_point_ok = False
            else:
                self.starting_point_ok = True

            self.joint_state_msg_received = True
        else:
            return


def main(args=None):
    rclpy.init(args=args)
    
    control_node = control_robot_master() #Ajuste de parámteros de las articulaciones del robot
    
    moveit2 = MoveIt2(
        node=control_node,
        joint_names=ur3e_model.joint_names(),
        base_link_name=ur3e_model.base_link_name(),
        end_effector_name=ur3e_model.end_effector_name(),
        group_name=ur3e_model.MOVE_GROUP_ARM,
        #callback_group=self.callback_group,
        )
        
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(control_node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()


    while(1):

        while control_node.pose_required_print().position.x != 0.0:
            position_r = control_node.pose_required_print()

            moveit2.move_to_pose(position=[position_r.position.x,position_r.position.y,position_r.position.z], quat_xyzw=position_r.orientation, cartesian=False) #moveit mueve el robot
            moveit2.wait_until_executed()

            time.sleep(3)

            control_node.close_gripper() 


            moveit2.move_to_pose(position=[position_r.position.x,position_r.position.y,position_r.position.z + 0.1], quat_xyzw=position_r.orientation, cartesian=True) #moveit mueve el robot
            moveit2.wait_until_executed()
            
            #time.sleep(3)
            
            #moveit2.move_to_pose(position=[-0.126,-0.27,0.297], quat_xyzw=position_r.orientation, cartesian=True) #moveit mueve el robot
            #moveit2.wait_until_executed()
            
            #[0.17,-0.347,0.246]
            
            control_node.open_gripper() 

            time.sleep(3)

        
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
