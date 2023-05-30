import asyncio
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

import time

from threading import Thread

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Int8

from functools import partial
from ur_msgs.srv import SetIO

from my_moveit2_py.Moveit2_resources import MoveIt2 #own resources
from my_moveit2_py import ur3e_model #own resources

from rclpy.callback_groups import ReentrantCallbackGroup

import tkinter as tk
from PIL import Image,ImageTk
from tkinter import font

class control_robot_master(Node):
    def __init__(self):
        super().__init__("control_robot_master")
        
        # Declare all parameters
        self.declare_parameter("controller_name", "joint_trajectory_position_controller")
        self.declare_parameter("joints",["shoulder_pan_joint", "shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]) #Obligado para lanzar inputs
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
        
        self.respuesta = 0 #NEW

        self.sec_color = "" #NEW
	
	#Subscribers & Publishers
        self.pose_required = Pose()
        self.subscriber_camera = self.create_subscription(Pose, "object_position",self.callback_recibo_pos_pedida, 10)

        #subsrciber respuesta de la interfaz
        self.subscriber_respuesta_menu = self.create_subscription(Int8, "resp_aplicacion",self.actualizar_respuesta, 10)
        
   
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

    def actualizar_respuesta(self, msg):
        self.respuesta = msg.data
        #self.get_logger().info(f"{self.respuesta}")

    def resp_required(self):
        if self.respuesta is not None:
            return self.respuesta
            


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

        if msg.position.z < 0.3:
            self.pose_required.position.z = msg.position.z
        else:
            self.pose_required.position.z = 0.139
   
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

"""
def callback(msg, respuesta):
    respuesta = msg.data
"""
def main(args=None):
    rclpy.init(args=args)
    contador = 0
    respuesta = 0
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


    control_node.get_logger().info("Buenas, bienvenido!\nA continuacion va a poder elegir entre diferentes aplicaciones de Pick and Place \nOpcion 1: Coger las piezas por orden y depositarlas en casa \nOpcion 2: Ordenar las piezas por colores y depositarlas \nOpcion 3: Apilar piezas del mismo color \n")

   
    respuesta = control_node.resp_required()
    
    while respuesta == 0:
        
        respuesta = control_node.resp_required()
    
    
    while(respuesta == 1 or respuesta == 2 or respuesta == 3):
        
        if(respuesta == 1):
            control_node.get_logger().info(f"respuesta: {respuesta}")
            
            time.sleep(3)
            #input("Introduce tres colores seguidos en orden: green/orange/red\n"))

            while control_node.pose_required_print().position.x != 0.0 and control_node.pose_required_print().position.y != 0.0 and control_node.pose_required_print().position.z < 0.3:
                #contador = contador + 1
                control_node.get_logger().info("HEY Aplicacion 1")
                position_r = control_node.pose_required_print()

                moveit2.move_to_pose(position=[position_r.position.x,position_r.position.y,position_r.position.z + 0.1], quat_xyzw=position_r.orientation, cartesian=True) #moveit mueve el robot
                moveit2.wait_until_executed()

                time.sleep(2)

                moveit2.move_to_pose(position=[position_r.position.x,position_r.position.y,position_r.position.z], quat_xyzw=position_r.orientation, cartesian=True) #moveit mueve el robot
                moveit2.wait_until_executed()

                time.sleep(3)

                control_node.close_gripper() 

                time.sleep(1)

                moveit2.move_to_pose(position=[position_r.position.x,position_r.position.y,position_r.position.z + 0.1], quat_xyzw=position_r.orientation, cartesian=True) 
                moveit2.wait_until_executed()

                time.sleep(1)

                moveit2.move_to_pose(position=[-0.251,-0.129,0.236], quat_xyzw=position_r.orientation, cartesian=False)
                moveit2.wait_until_executed()

                control_node.open_gripper() 

                moveit2.move_to_pose(position=[-0.251,-0.129,0.35], quat_xyzw=position_r.orientation, cartesian=True)
                moveit2.wait_until_executed()

                time.sleep(2)

                moveit2.move_to_pose(position=[-0.155,-0.267,0.28], quat_xyzw=position_r.orientation, cartesian=True)
                moveit2.wait_until_executed()

                time.sleep(1)

        if(respuesta == 2):
            control_node.get_logger().info(f"respuesta robot: {respuesta}")

            
            time.sleep(3)
            #input("Introduce tres colores seguidos en orden: green/orange/red\n"))

            while control_node.pose_required_print().position.x != 0.0 and control_node.pose_required_print().position.z < 0.3:
                #contador = contador + 1
                control_node.get_logger().info("HEY: Aplicacion 2")
                position_r = control_node.pose_required_print()

                moveit2.move_to_pose(position=[position_r.position.x,position_r.position.y,position_r.position.z + 0.1], quat_xyzw=position_r.orientation, cartesian=True) #moveit mueve el robot
                moveit2.wait_until_executed()

                time.sleep(2)

                moveit2.move_to_pose(position=[position_r.position.x,position_r.position.y,position_r.position.z], quat_xyzw=position_r.orientation, cartesian=True) #moveit mueve el robot
                moveit2.wait_until_executed()

                time.sleep(1)

                control_node.close_gripper() 

                time.sleep(1)

                moveit2.move_to_pose(position=[-0.155,-0.267,0.28], quat_xyzw=position_r.orientation, cartesian=True) 
                moveit2.wait_until_executed()

                time.sleep(1)

                control_node.get_logger().info(f"{contador}")

                if contador == 0:
                    moveit2.move_to_pose(position=[-0.011,-0.408,0.239], quat_xyzw=position_r.orientation, cartesian=True)
                    moveit2.wait_until_executed()

                    time.sleep(1)

                    moveit2.move_to_pose(position=[-0.011,-0.408,0.144], quat_xyzw=position_r.orientation, cartesian=True)
                    moveit2.wait_until_executed()

                elif contador == 1:

                    moveit2.move_to_pose(position=[0.04,-0.410,0.239], quat_xyzw=position_r.orientation, cartesian=True)
                    moveit2.wait_until_executed()

                    time.sleep(1)

                    moveit2.move_to_pose(position=[0.04,-0.410,0.144], quat_xyzw=position_r.orientation, cartesian=True)
                    moveit2.wait_until_executed()

                    

                elif contador == 2:

                    moveit2.move_to_pose(position=[0.015,-0.39,0.239], quat_xyzw=[0.18474743606783836, -0.9827670174247706, 0.004786203006938818, 0.003803496964226814], cartesian=True)
                    moveit2.wait_until_executed()

                    time.sleep(1)

                    moveit2.move_to_pose(position=[0.015,-0.39,0.155], quat_xyzw=[0.18474743606783836, -0.9827670174247706, 0.004786203006938818, 0.003803496964226814], cartesian=True)
                    moveit2.wait_until_executed()

                    

                elif contador == 3:

                    moveit2.move_to_pose(position=[0.015,-0.442,0.239], quat_xyzw=[0.18474743606783836, -0.9827670174247706, 0.004786203006938818, 0.003803496964226814], cartesian=True)
                    moveit2.wait_until_executed()

                    time.sleep(1)

                    moveit2.move_to_pose(position=[0.015,-0.442,0.155], quat_xyzw=[0.18474743606783836, -0.9827670174247706, 0.004786203006938818, 0.003803496964226814], cartesian=True)
                    moveit2.wait_until_executed()

        


                time.sleep(2)

                control_node.open_gripper()

                time.sleep(1)

                moveit2.move_to_pose(position=[0.001,-0.413,0.282], quat_xyzw=position_r.orientation, cartesian=True)
                moveit2.wait_until_executed()

                time.sleep(1)

                moveit2.move_to_pose(position=[-0.155,-0.267,0.28], quat_xyzw=position_r.orientation, cartesian=True)
                moveit2.wait_until_executed()

                time.sleep(1)

                contador = contador + 1

                if contador == 4:
                    contador = 0

        if(respuesta == 3):
            control_node.get_logger().info(f"respuesta robot: {respuesta}")

            
            time.sleep(3)
            #input("Introduce tres colores seguidos en orden: green/orange/red\n"))

            while control_node.pose_required_print().position.x != 0.0 and control_node.pose_required_print().position.z < 0.3:
                #contador = contador + 1
                control_node.get_logger().info("HEY: Aplicacion 2")
                position_r = control_node.pose_required_print()

                if contador == 0 or contador == 1:

                    moveit2.move_to_pose(position=[position_r.position.x,position_r.position.y,position_r.position.z + 0.1], quat_xyzw=[0.18474743606783836, -0.9827670174247706, 0.004786203006938818, 0.003803496964226814], cartesian=True) #moveit mueve el robot
                    moveit2.wait_until_executed()

                    time.sleep(2)

                    moveit2.move_to_pose(position=[position_r.position.x,position_r.position.y,position_r.position.z], quat_xyzw=[0.18474743606783836, -0.9827670174247706, 0.004786203006938818, 0.003803496964226814], cartesian=True) #moveit mueve el robot
                    moveit2.wait_until_executed()

                    time.sleep(1)

                    control_node.close_gripper() 

                    time.sleep(2)

                    moveit2.move_to_pose(position=[position_r.position.x,position_r.position.y,position_r.position.z + 0.1], quat_xyzw=[0.18474743606783836, -0.9827670174247706, 0.004786203006938818, 0.003803496964226814], cartesian=True) 
                    moveit2.wait_until_executed()

                    time.sleep(2)

                    moveit2.move_to_pose(position=[-0.155,-0.267,0.28], quat_xyzw=position_r.orientation, cartesian=True)
                    moveit2.wait_until_executed()

                    time.sleep(3)

                    control_node.get_logger().info(f"{contador}")

                else:

                    moveit2.move_to_pose(position=[position_r.position.x,position_r.position.y,position_r.position.z + 0.1], quat_xyzw=position_r.orientation, cartesian=True) #moveit mueve el robot
                    moveit2.wait_until_executed()

                    time.sleep(2)

                    moveit2.move_to_pose(position=[position_r.position.x,position_r.position.y,position_r.position.z], quat_xyzw=position_r.orientation, cartesian=True) #moveit mueve el robot
                    moveit2.wait_until_executed()

                    time.sleep(1)

                    control_node.close_gripper() 

                    time.sleep(1)

                    moveit2.move_to_pose(position=[position_r.position.x,position_r.position.y,position_r.position.z + 0.1], quat_xyzw=position_r.orientation, cartesian=True) 
                    moveit2.wait_until_executed()

                    time.sleep(1)

                    moveit2.move_to_pose(position=[-0.155,-0.267,0.28], quat_xyzw=position_r.orientation, cartesian=True)
                    moveit2.wait_until_executed()

                    time.sleep(1)

                    control_node.get_logger().info(f"{contador}")

                if contador == 4:
                    contador = 0

        
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
