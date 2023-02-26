import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Publisher(Node):

 def __init__(self):
   super().__init__("pedir_posicion")
   self.publisher_ = self.create_publisher(Float32MultiArray, "posicion_pedida", 10)
   self.timer_ = self.create_timer(0.5, self.publish_news)
   self.get_logger().info("Publisher publishing on topic called pedir_posicion")
   
   
 def publish_news(self):
   self.number = [-1.57, -3.14, 2.4, -4.2, 0.01, 4.72]
   #self.number= input('Which position?')
   msg = Float32MultiArray()
   msg.data = self.number
   self.publisher_.publish(msg)
  
def main(args=None):
   rclpy.init(args=args)
   node = Publisher()
   rclpy.spin(node)
   rclpy.shutdown()
 
if __name__ == "__main__":
   main()
   
