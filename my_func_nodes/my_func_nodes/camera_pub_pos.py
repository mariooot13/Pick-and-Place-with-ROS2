import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion

from ur_msgs.srv import SetIO #servicio pinza

class Camera_Publisher(Node):

 def __init__(self):
   super().__init__("camera_pub_pos")
   
   self.publisher_ = self.create_publisher(Pose, "posicion_pedida", 10)
   self.timer_ = self.create_timer(5, self.publish_news)
   
   self.get_logger().info("Camera's publisher publishing on topic called posicion_pedida...")

   #Posteriormente serán los datos de la camara
   self.position = [-0.252,-0.118, 0.195]
   self.quaternion = [0.570803357577419, 0.8205298265175397, -0.00518912143252199, 0.029789323459918908]
   
 def publish_news(self):

   
   
   #for i in range(len(self.number)):
   #	self.number[i] = float(input(f'Which value for the position[{i}]?'))

   msg = Pose()
   
   msg.position.x = self.position[0]
   msg.position.y = self.position[1]
   msg.position.z = self.position[2]
   
   msg.orientation.x = self.quaternion[0]
   msg.orientation.y = self.quaternion[1]
   msg.orientation.z = self.quaternion[2]
   msg.orientation.w = self.quaternion[3]
  
   
   self.publisher_.publish(msg)
   
   self.get_logger().info("Mandando posicion {} y orientacion en quaternion {}".format(self.position,self.quaternion))
  
def main(args=None):
   rclpy.init(args=args)
   
   node = Camera_Publisher()
   
   rclpy.spin(node)
   rclpy.shutdown()
   
 
if __name__ == "__main__":
   main()
