import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import numpy as np
import time

import depthai as dai

from geometry_msgs.msg import Pose, Point, Quaternion

def get_distance_from_disparity(disparity_image, x, y):
    #Parámetros de la cámara -> importante calibracion
    baseline = 7.5 # en cm
    focal_length =  451.144 #calibrada en pixels
    
    # Calcula la distancia a partir de la disparidad
    disparity_value = disparity_image[y,x]
    distance = (baseline * focal_length) / disparity_value

    return distance


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.detect_object,
            10)

        self.subs_disparity = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.def_depth,
            10)

        self.publisher_ = self.create_publisher(Pose, "object_position", 10)
        self.timer_ = self.create_timer(7, self.publish_news)

        self.subscriber_sec_colores = self.create_subscription(String, "sec_color", self.actualizar_sec_colores, 10) #subsrciber interfaz sec_colores

        
        self.quaternion = [0.570803357577419, 0.8205298265175397, -0.00518912143252199, 0.029789323459918908]

        self.timer = self.create_timer(4, self.actualizar_distance_depth) 

        self.timer = self.create_timer(25, self.change_color) ##22

        self.depth_distance_obj = 0
        self.medidas = []

        #centro destino imagen de 8 bits
        self.dst_x = 0
        self.dst_y = 0

        self.bridge = CvBridge()
        
        self.centro_x_24bits = 0
        self.centro_y_24bits = 0

        self.position_x_to_robot = 0.00
        self.position_y_to_robot = 0.00
        self.position_z_to_robot = 0.00

        # Distancia focal de la cámara RGB en metros
        self.focalLength = 0.00337

        # Altura real del objeto en cm
        self.realHeight = 0.074

        self.mask_high = (30, 50, 50)
        self.mask_low = (70, 255, 255)

        self.counter = 0

        #self.sec_color = String()
        self.sec_color = ""

        #antes: self. = input("") = "avnm"
        

    def actualizar_sec_colores(self, msg):
        self.sec_color = msg.data
        self.get_logger().info(f"funcion {self.sec_color}")
        

    def change_color(self):
        self.counter = self.counter + 1

    def publish_news(self):
 
        msg = Pose()
        
        msg.position.x = self.position_x_to_robot * 0.001
        msg.position.y = self.position_y_to_robot * 0.001
        #msg.position.z = 0.139 
        if self.position_z_to_robot > 0:
            if self.position_z_to_robot > 61.5:
                msg.position.z = 0.139 
            else:
                msg.position.z = float(self.position_z_to_robot) * 0.01
        else:
            msg.position.z = 0.139

        if msg.position.z < 0.139:
            msg.position.z = 0.139 
        
        msg.orientation.x = self.quaternion[0]
        msg.orientation.y = self.quaternion[1]
        msg.orientation.z = self.quaternion[2]
        msg.orientation.w = self.quaternion[3]


        self.publisher_.publish(msg)

        #self.counter = self.counter + 1
        
        self.get_logger().info("Mandando posicion {},{},{}".format(msg.position.x, msg.position.y, msg.position.z))

        #hasta aqui
        
   
    def actualizar_distance_depth(self):

        #media = sum(self.medidas) / len(self.medidas)
        self.get_logger().info(f"EL objeto se encuentra a {self.depth_distance_obj} cm de distancia de la camara")
        #Ajustar
        self.position_z_to_robot =  13 + (62.5  - (self.depth_distance_obj + 0.8))  #61.51 al suelo, -13 pinza. habia 63  0.176
        self.medidas = []

    def def_depth(self, msg):
        #x, y = self.center.x, self.center.y
        if self.dst_x != 0 and self.dst_y != 0:
            x,y = self.dst_x, self.dst_y
        else:
            x, y = 320,200

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        depth_val = get_distance_from_disparity(cv_image,x,y)
        cv2.circle(cv_image, (x, y), 3, (255, 255, 255), 2)
        #cv2.circle(cv_image, (0, 0), 20, (255, 255, 255), 2)
        cv2.imshow('Depth measurement', cv_image)
        cv2.waitKey(3)

        #height, width = cv_image.shape
        #self.get_logger().info(f"imagen de 8 bits: {width} x {height}")

        if depth_val is not None:
            self.medidas.append(depth_val)

        self.depth_distance_obj = depth_val

        #libero la posicion para el siguiente objeto a coger
        self.dst_x = 0
        self.dst_y = 0
        
        

    def detect_object(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #Calibracion:
        """
        x2, y2 = 589, 367
        x1,y1= 1210, 367
                #height, width, channels = cv_image.shape
        cv2.circle(cv_image, (x2, y2), 4, (255, 255, 255), 2)
        cv2.circle(cv_image, (x1, y1), 4, (255, 255, 255), 2)
        distancia = abs(x2 - x1) + abs(y2 - y1)
        self.get_logger().info(f"distancia : {distancia}")
        """
        
    
        
        # Definir el rango de valores HSV para el color verde

        green_lower = (30, 100, 100)
        green_upper = (70, 255, 255)

        black_lower = (0, 0, 0)
        black_upper = (180, 255, 30)

        white_lower = (0, 0, 231)
        white_upper = (180, 30, 255)

        orange_lower = (10, 100, 20)
        orange_upper = (25, 255, 255)

        dark_blue_lower = (90, 50, 50)
        dark_blue_upper = (120, 255, 255)

        morado_lower = (135, 50, 50)
        morado_upper = (160, 255, 255)

        yellow_lower = (25, 100, 50)
        yellow_upper = (35, 255, 255)

        blue_lower = (90, 100, 90)
        blue_upper = (130, 255, 255)

        orange_lower = (0, 100, 100)
        orange_upper = (20, 255, 255)
        
        #self.mask_high = green_upper
        #self.mask_low = green_lower
        #Corregir
        self.get_logger().info(f"algoritmo {self.sec_color}")
        self.get_logger().info(f"algoritmo {self.counter}")
        
        if len(self.sec_color) == 4 and self.counter < 5:
            if self.sec_color[self.counter] == "a":
                self.mask_high = blue_upper
                self.mask_low = blue_lower

            elif self.sec_color[self.counter] == "v":
                self.mask_high = green_upper
                self.mask_low = green_lower

            elif self.sec_color[self.counter] == "n":
                self.mask_high = orange_upper
                self.mask_low = orange_lower

            elif self.sec_color[self.counter] == "m":
                self.mask_high = morado_upper
                self.mask_low = morado_lower
            

        if(self.counter == len(self.sec_color)):
            self.counter = 0
        
        #self.mask_high = green_upper
        #self.mask_low = green_lower
        
        # Aplicar una máscara binaria a la imagen HSV
        mask = cv2.inRange(hsv_image, self.mask_low , self.mask_high)
        filtered_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 60 and h > 100 and w < 100 and h < 250:  # Adjust these values according to your object size 50,50
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                #Coordenadas del objeto en la imgen a color de 24 bits
                self.centro_x_24bits = x - w/2 - w/4#importante
                self.centro_y_24bits = y + h/2
                #self.get_logger().info(f"Coordenadas centro.x = {self.centro_x_24bits} y centro.y = {self.centro_y_24bits}")
                #self.get_logger().info(f"ancho del objeto: {w} y altura del objeto: {h}") #ancho 77 que son 24 mm y altura 216 que son 74 mm

                #factor_escala = 2.40 / 77 #centimetros por pixeles con 74 funciona
                factor_escala = 25 / 621 #calibrado

                # Tamaño de la imagen de origen
                src_width = 1920
                src_height = 1080

                # Tamaño de la imagen de destino
                dst_width = 640
                dst_height = 400

                # Posición en la imagen de origen
                src_x = self.centro_x_24bits
                src_y = self.centro_y_24bits

                centro_cm_x = factor_escala * src_x
                centro_cm_y = factor_escala * src_y

                

                #self.get_logger().info(f"{centro_cm_x} , {centro_cm_y}")

                

                centro_robot_cm_x = - 534 + centro_cm_x * 10
                centro_robot_cm_y = - 141 - centro_cm_y * 10 

                #self.get_logger().info(f"centro de la camara en x{centro_robot_cm_x}, centro de la camara en y {centro_robot_cm_y}")
                    
                # Calcular la relación de escala
                scale_x = dst_width / src_width
                scale_y = dst_height / src_height

                # Aplicar la relación de escala a la posición en la imagen de origen
                self.dst_x = int(src_x * scale_x)
                self.dst_y = int(src_y * scale_y)

                #self.get_logger().info(f"{self.dst_x},{self.dst_y}")
                
                self.position_x_to_robot = float(centro_robot_cm_x)
                self.position_y_to_robot = float(centro_robot_cm_y)
            
        cv2.imshow('Object RGB Detector', cv_image)
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
