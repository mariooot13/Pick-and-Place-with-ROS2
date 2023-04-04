import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion

class Camera_Publisher(Node):

 def __init__(self):
   super().__init__("camera_pub_pos")
   
   self.publisher_ = self.create_publisher(Pose, "posicion_pedida", 10)
   self.timer_ = self.create_timer(5, self.publish_news)
   
   self.get_logger().info("Camera's publisher publishing on topic called posicion_pedida...")

   #Posteriormente serán los datos de la camara, datos por defecto
   self.position = [-0.252,-0.118, 0.195]
   self.quaternion = [0.570803357577419, 0.8205298265175397, -0.00518912143252199, 0.029789323459918908]
   
 def publish_news(self):
 

   if (input("Do you want to move the robot to pick an object? Y/N: \n")=="Y"):

      self.get_logger().info("Introduce the point where it is (this will be tha camera's role)")
      for i in range(3):
   	   self.position[i] = float(input(f'Which value for the position[{i}]?'))
   	
      for i in range(4):
   	   self.quaternion[i] = float(input(f'Which value for the orientation[{i}]?'))

      msg = Pose()
   
      msg.position.x = self.position[0]
      msg.position.y = self.position[1]
      msg.position.z = self.position[2]
   
      msg.orientation.x = self.quaternion[0]
      msg.orientation.y = self.quaternion[1]
      msg.orientation.z = self.quaternion[2]
      msg.orientation.w = self.quaternion[3]

   else:

      self.get_logger().info("Well done. See you")
      exit(0)

   self.publisher_.publish(msg)
   
   self.get_logger().info("Mandando posicion {} y orientacion en quaternion {}".format(self.position,self.quaternion))
  
def main(args=None):
   rclpy.init(args=args)
   
   node = Camera_Publisher()
   
   rclpy.spin(node)
   rclpy.shutdown()
   
 
if __name__ == "__main__":
   main()
