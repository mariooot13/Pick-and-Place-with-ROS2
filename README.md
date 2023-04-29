

<p align="center">
Pick-and-Place-with-ROS2
  <h2 align="center">TFG Pick and place UR3e using ROS2 and computer vision</h2>

  <p align="center">
  Mario Sanchez Garcia UPM
  </p>
</p>
<br>



The motivation of the project is to implement an application of Pick and Place with an UR3e robot with the help of ROS2 on python.
The main idea is to recognize some objects with computer vision using a camera, and be capable of picking and placing those objetcs with the planner moveit, rviz2 for its visualization and the use of some ROS tools. In this case we are using an UR3e and a gripper, but you can adapt it for every model of UR and other tools.

## DESCRIPTION

`my_func_nodes`  own nodes for the properly and fully control of the robot 

      ├── my_func_nodes 
      ├── camera_pub_pos.py: It publishes the position of the object manually. This will be the computer vision.
      └── control_robot_master.py: It is the main node related to the movement. It is in charge of the movement with moveit, the use of the gripper and  it also receives the position of the camera by a subscriber.

      ├── resources
      ├── euler_to_quat.py: This is a resource that transforms de rotation vector from de robot's reference to quaternion.
      └── my_func_nodes


`my_moveit2_py` resources and functions based on moveit in order to plan and execute trajectories

      ├── my_moveit2_py: 
      ├── Moveit2_resources.py: library helped by MoveGroup where every function related to moveit are developed thorugh the movement to the configuration of any parameter.
      └── ur3e_model.py: model configurated for UR3e with the name of the links and the gripper.

`my_robot_bringup_ms` launchers and config files

      ├── launch
      ├── control_robot.py
      └── launch_descripition_resources

      ├── config
      ├── argsforlaunching.yaml
      └── param_bringup.yaml

<img src="https://github.com/mariooot13/Pick-and-Place-with-ROS2/blob/tutorial/DIAGNODOS.png">

## GETTING STARTED

0) This tutorial has been made for those users who has Ubuntu distribution. Otherwise you will have to look for some additional requests.

1) ROS2 Foxy and dependencies

First of all, you need to have ROS2 and its dependencies properly installed. We are using Foxy distribution on Ubuntu. 

Here you have ROS2 Documentation: https://docs.ros.org/en/foxy/Installation.html

Install python3: sudo apt install python3-colcon-common-extensions

2) UR ROS2 Driver

The next step is to clone the drivers and configuration from the github of Universal Robots. 

a) export COLCON_WS=~/workspace/ros_ur_driver
   mkdir -p $COLCON_WS/src
   
b) cd $COLCON_WS
  git clone -b foxy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
  rosdep install --ignore-src --from-paths src -y -r
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  source install/setup.bash -> You can configure the file instead.
  
- You can also follow the getting started section of the UR github. Make sure you look for Foxy branch:

  https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy 
  
3) Once you have UR drivers cloned, you will clone this repository. Branch called tutorial!

a) Go into /workspace/ros_ur_driver/src

b) 1 Option: download a zip and paste it inside src directory.

   2 Option: git clone https://github.com/mariooot13/Pick-and-Place-with-ROS2/tree/tutorial
  
  
### RECOMMENDATIONS

- Install terminator for a better visualization of the behaviour. 

sudo apt install terminator

  
## USAGE

In parallel:

- You need to write this command on each console of change de configuration of the project: 

      source install/setup.bash 

- Here we are launching the master control node for the robot and the controllers of Universal Robots. If you want to change the IP of the robot, you will have to configure the arguments from de launch file. In addition, you need to change the paths of your directories inside the launchers.

      ros2 launch my_robot_bringup_ms control_robot.py

- In this step, you will launch moveit with the same parameters of the previous launcher

      ros2 launch ur_bringup ur_moveit.launch.py ur_type:=ur3e robot_ip:="192.168.20.35" launch_rviz:=true

- Finally, run the camera's node: 

      ros2 run my_func_nodes camera_exec
      
      Note: you will have to introduce a position requested on the terminal. In a future this will be the camera itself.
      
      Example of positions: 
      
      self.position = [-0.2,-0.1, 0.1]
      self.quaternion = [0.5, 0.5, 0.5, 0.5]

