import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from ur_msgs.srv import SetIO #servicio pinza

class Publisher(Node):

 def __init__(self):
   super().__init__("pedir_posicion")
   self.publisher_ = self.create_publisher(Float64MultiArray, "posicion_pedida", 10)
   self.timer_ = self.create_timer(5, self.publish_news)
   self.get_logger().info("Publisher publishing on topic called pedir_posicion")
   
   #servicio pinza:
  # self.server_pinza = sel.create_service(SetIO,"Pinza",self.callback_pinza_SetIO)
   
 #def callback_pinza_SetIO(self, request, response)
   
   
 def publish_news(self):
   self.number = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00] #inicializado
   for i in range(len(self.number)):
   	self.number[i] = float(input(f'Which value for the position[{i}]?'))
   
   file_positions = open("Positions.txt","w")
   file_positions.write(f"Position published to the robot: {self.number}")
   file_positions.write("\n")
   file_positions.close()
   
   msg = Float64MultiArray()
   msg.data = self.number
   self.publisher_.publish(msg)
   self.get_logger().info("Mandando posicion {}".format(msg.data))
  
def main(args=None):
   rclpy.init(args=args)
   node = Publisher()
   rclpy.spin(node)
   rclpy.shutdown()
   
 
if __name__ == "__main__":
   main()
   
