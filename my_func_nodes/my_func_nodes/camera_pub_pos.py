import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

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
        self.timer_ = self.create_timer(8, self.publish_news)

        
        self.quaternion = [0.570803357577419, 0.8205298265175397, -0.00518912143252199, 0.029789323459918908]

        self.timer = self.create_timer(5, self.actualizar_distance_depth) 

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

    def publish_news(self):
 
        msg = Pose()
        
        msg.position.x = self.position_x_to_robot * 0.01
        msg.position.y = self.position_y_to_robot * 0.01
        #msg.position.z = 0.139 
        msg.position.z = float(self.position_z_to_robot) * 0.001
        
        msg.orientation.x = self.quaternion[0]
        msg.orientation.y = self.quaternion[1]
        msg.orientation.z = self.quaternion[2]
        msg.orientation.w = self.quaternion[3]


        self.publisher_.publish(msg)
        
        self.get_logger().info("Mandando posicion {},{},{}".format(msg.position.x, msg.position.y, msg.position.z))

        #hasta aqui
        
   
    def actualizar_distance_depth(self):

        media = sum(self.medidas) / len(self.medidas)
        self.get_logger().info(f"EL objeto se encuentra a {self.depth_distance_obj} cm de distancia de la camara")
        #Ajustar
        self.position_z_to_robot = 54.4 - self.depth_distance_obj #54 seria la distancia al suelo - lo que mide ejempl 52 -> 2 cm ejemplo desde el suelo
        self.medidas = []

    def def_depth(self, msg):
        #x, y = self.center.x, self.center.y
        if self.dst_x != 0 and self.dst_y != 0:
            x,y = self.dst_x, self.dst_y
        else:
            x, y = 320,200

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        depth_val = get_distance_from_disparity(cv_image,x,y)
        cv2.circle(cv_image, (x, y), 20, (255, 255, 255), 2)
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
        """
        try:
            cv2.imshow('Object RGB Detector', cv_image)
        except:
    # Manejo de excepciones si ocurre alguna
            cv2.imshow('Object RGB Detector', cv_image)
        """
        
        # Definir el rango de valores HSV para el color verde
        green_lower = (30, 50, 50)
        green_upper = (70, 255, 255)

        black_lower = (0, 0, 0)
        black_upper = (180, 255, 30)

        white_lower = (0, 0, 231)
        white_upper = (180, 30, 255)

        orange_lower = (10, 100, 20)
        orange_upper = (25, 255, 255)

        dark_blue_lower = (90, 50, 50)
        dark_blue_upper = (120, 255, 255)

        red_lower = (0, 50, 50)
        red_upper = (10, 255, 255)

        yellow_lower = (25, 100, 50)
        yellow_upper = (35, 255, 255)
        
        a = input("dime que color quieres detectar: rojo, verde, amarillo")

        #Corregir

        if a == "rojo":
            self.mask_high = red_upper
            self.mask_low = red_lower

        elif a == "verde":
            self.mask_high = green_upper
            self.mask_low = green_lower

        elif a == "amarillo":
            self.mask_high = yellow_upper
            self.mask_low = yellow_lower
        
        # Aplicar una máscara binaria a la imagen HSV
        mask = cv2.inRange(hsv_image, self.mask_low , self.mask_high)
        filtered_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 70 and h > 50:  # Adjust these values according to your object size 50,50
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                #Coordenadas del objeto en la imgen a color de 24 bits
                self.centro_x_24bits = x + w/2
                self.centro_y_24bits = y + h/2
                #self.get_logger().info(f"Coordenadas centro.x = {self.centro_x_24bits} y centro.y = {self.centro_y_24bits}")
               
                #height, width, channels = cv_image.shape
                

                #self.get_logger().info(f"ancho del objeto: {w} y altura del objeto: {h}") #ancho 77 que son 24 mm y altura 216 que son 74 mm

                factor_escala = 2.40 / w #centimetros por pixeles

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

                #self.get_logger().info(f"centro de la camara en x{centro_cm_x}, centro de la camara en y {centro_cm_y}")

                centro_robot_cm_x = - 49 + centro_cm_x
                centro_robot_cm_y = - 20 - centro_cm_y

                #self.get_logger().info(f"centro de la camara en x{centro_robot_cm_x}, centro de la camara en y {centro_robot_cm_y}")
                    
                # Calcular la relación de escala
                scale_x = dst_width / src_width
                scale_y = dst_height / src_height

                # Aplicar la relación de escala a la posición en la imagen de origen
                self.dst_x = int(src_x * scale_x)
                self.dst_y = int(src_y * scale_y)

                
                self.position_x_to_robot = float(centro_robot_cm_x)
                self.position_y_to_robot = float(centro_robot_cm_y)
                
                


        #cv2.imshow('Object RGB Detector', cv_image)
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
