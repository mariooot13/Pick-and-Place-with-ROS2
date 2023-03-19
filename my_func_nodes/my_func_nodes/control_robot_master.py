import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from functools import partial
from ur_msgs.srv import SetIO


class control_robot_master(Node):
    def __init__(self):
        super().__init__("control_robot_master")
        
        # Declare all parameters
        self.declare_parameter("controller_name", "joint_trajectory_position_controller")
        self.declare_parameter("joints")
        self.declare_parameter("check_starting_point", False)
        self.declare_parameter("starting_point_limits")

       
        self.pos_wanted = JointTrajectory() #Trajectory 
        
         # Read parameters
        controller_name = self.get_parameter("controller_name").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}
        self.get_logger().info(
            'LETSGO')
	
        #self.subscriber_ = self.create_subscription(Float64MultiArray, "posicion_pedida",self.callback_recibo_pos_pedida, 10)
        
        self.subscriber_ = self.create_subscription(JointTrajectory, "trajectory_from_moveit2",self.callback_recibo_pos_pedida, 10)
   
	
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
            
            
            
        # initialize starting point status
        if not self.check_starting_point:
            self.starting_point_ok = True
        else:
            self.starting_point_ok = False

        self.joint_state_msg_received = False


        publish_topic = "/" + controller_name + "/" + "joint_trajectory"
	
        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(10, self.timer_callback)
        
    
    #Servicio GRIPPER
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
        



    """def callback_recibo_pos_pedida(self, msg):
        if self.pos_wanted is None:
            self.pos_wanted = [-1.60, -1.72, -1.35, -3.22, -1.72, 5.20]
            self.get_logger().info('No position introduced, a new review is required')
        else:
            self.pos_wanted = msg.data 
               
        self.get_logger().info('Moving the robot to the position {}.'.format(self.pos_wanted))"""
    
    def callback_recibo_pos_pedida(self, msg):
        if self.pos_wanted is None:
            self.get_logger().info('No position introduced, a new review is required')
        else:
            self.pos_wanted.points = msg.points 
               
        
    
    
    def timer_callback(self):

        if self.starting_point_ok:
        
            traj = JointTrajectory()
            traj.joint_names = self.joints
            traj.points = self.pos_wanted.points 
            #point = JointTrajectoryPoint()
            #point.positions = self.pos_wanted.points 
            #point.time_from_start = Duration(sec=4)
            #traj.points.append(point)
            self.publisher_.publish(traj)
            
            #if self.pos_wanted[3] < 1.15 and self.pos_wanted[3] > -2 :
            #	self.call_gripper(1, 17, 24.0)
            """         
            int8 fun
            int8 pin
            float32 state
            ---
            bool success
            
           
	        """

        elif self.check_starting_point and not self.joint_state_msg_received:
            self.get_logger().warn(
                'Start configuration could not be checked! Check "joint_state" topic!'
            )
        else:
            self.get_logger().warn("Start configuration is not within configured limits!")

    def joint_state_callback(self, msg): #investigar a fondo

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

    control_robot_joint_trajectory = control_robot_master()

    rclpy.spin(control_robot_joint_trajectory)
    control_robot_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
