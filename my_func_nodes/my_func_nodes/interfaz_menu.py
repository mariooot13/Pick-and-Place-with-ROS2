import tkinter as tk
from PIL import Image,ImageTk
from std_msgs.msg import String, Int8
import rclpy
from rclpy.node import Node
import threading
from threading import Thread

class interfaz_menu(Node):
    def __init__(self):
        super().__init__("interfaz_menu")

        self.publisher_resp = self.create_publisher(Int8, "resp_aplicacion", 10)
        self.publisher_color = self.create_publisher(String, "sec_color", 10)

        self.respuesta = 0
        self.sec_color = String()

        # Crear una instancia de Tkinter
        self.ventana = tk.Tk()
        self.ventana.title("Pick and Place con UR3e y visión") 
        self.ventana.configure(bg="#FFFFFF") 

        self.imagen_v = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/verde.png")  # Reemplaza "imagen.jpg" con la ruta y nombre de tu imagen en formato JPEG
        self.imagen_v = self.imagen_v.resize((10000, 4000)) 
        self.imagen_tk_v = ImageTk.PhotoImage(self.imagen_v)

        self.imagen_a = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/azul.jpeg")  # Reemplaza "imagen.jpg" con la ruta y nombre de tu imagen en formato JPEG
        self.imagen_a = self.imagen_a.resize((10000, 4000)) 
        self.imagen_tk_a = ImageTk.PhotoImage(self.imagen_a)

        self.imagen_n = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/naranja.png")  # Reemplaza "imagen.jpg" con la ruta y nombre de tu imagen en formato JPEG
        self.imagen_n = self.imagen_n.resize((10000, 4000)) 
        self.imagen_tk_n = ImageTk.PhotoImage(self.imagen_n)

        self.imagen_m = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/morado.png")  # Reemplaza "imagen.jpg" con la ruta y nombre de tu imagen en formato JPEG
        self.imagen_m = self.imagen_m.resize((10000, 4000)) 
        self.imagen_tk_m = ImageTk.PhotoImage(self.imagen_m)

        self.imagen_ap1 = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/orden.jpeg")  # Reemplaza "imagen.jpg" con la ruta y nombre de tu imagen en formato JPEG
        self.imagen_ap1 = self.imagen_ap1.resize((200, 140)) 
        self.imagen_tk_ap1 = ImageTk.PhotoImage(self.imagen_ap1)

        self.imagen_ap2 = Image.open("/home/mario/workspace/ros_ur_driver/src/my_func_nodes/resource/paletizado.jpeg")  # Reemplaza "imagen.jpg" con la ruta y nombre de tu imagen en formato JPEG
        self.imagen_ap2 = self.imagen_ap2.resize((200, 140)) 
        self.imagen_tk_ap2 = ImageTk.PhotoImage(self.imagen_ap2)

        #imagen_ap3 = Image.open("despaletizado.jpeg")  HACER LUNES
        #imagen_ap3 = imagen_ap3.resize((10000, 4000)) 
        #imagen_tk_ap3 = ImageTk.PhotoImage(imagen_ap3)

        # Establecer el tamaño de la ventana
        self.ventana.geometry("1000x500")  # Ancho x Alto

        # Agregar widgets a la ventana
        self.etiqueta1 = tk.Label(self.ventana, text="Buenas, bienvenido!\nA continuacion va a poder elegir entre diferentes aplicaciones de Pick and Place ",bg="#FFFDD0")
        self.etiqueta1.place(x=230, y=6)

        self.etiqueta_color = tk.Label(self.ventana, text="Seleccione los colores por orden de las piezas a recoger. ¡Un solo click!",bg="#FFFDD0")
        self.etiqueta_color.place(x=260, y=60)

        self.etiqueta_ap1 = tk.Label(self.ventana, text="Aplicacion 1: Ordenado de piezas",bg="#FFFDD0")
        self.etiqueta_ap1.place(x=133, y=250)

        self.etiqueta_ap2= tk.Label(self.ventana, text="Aplicacion  2: Paletizado",bg="#FFFDD0")
        self.etiqueta_ap2.place(x=415, y=250)

        self.etiqueta_ap3= tk.Label(self.ventana, text="Aplicacion  3: Despaletizado",bg="#FFFDD0")
        self.etiqueta_ap3.place(x=650, y=250)

        self.etiqueta_op= tk.Label(self.ventana, text="Seleccione una opción por favor:",bg="#FFFDD0")
        self.etiqueta_op.place(x=390, y=215)

        self.boton1 = tk.Button(self.ventana, image=self.imagen_tk_ap1, width=200, height=140, command=self.respuesta_1)
        self.boton2 = tk.Button(self.ventana, image=self.imagen_tk_ap2, width=200, height=140, command=self.respuesta_2)
        self.boton3 = tk.Button(self.ventana, width=20, height=8, command=self.respuesta_3)
        self.boton4 = tk.Button(self.ventana, image=self.imagen_tk_n, width=100, height=70, command=self.color_naranja)
        self.boton5 = tk.Button(self.ventana, image=self.imagen_tk_m, width=100, height=70,command=self.color_morado)
        self.boton6 = tk.Button(self.ventana, image=self.imagen_tk_v, width=100, height=70,command=self.color_verde)
        self.boton7 = tk.Button(self.ventana,  image=self.imagen_tk_a, width=100, height=70,command=self.color_azul)
        # Posicionar los botones usando el método pack()
        self.boton1.place(x=140, y=280)  # Arriba
        self.boton2.place(x=395, y=280) # Izquierda
        self.boton3.place(x=650, y=280)  # Derecha
        self.boton4.place(x=140, y=100)  # Abajo
        self.boton5.place(x=340, y=100)  # Por defecto, se posiciona en orden
        self.boton6.place(x=540, y=100) 
        self.boton7.place(x=740, y=100) 
        self.get_logger().info(f"jejeejejej{self.respuesta}")

        # Iniciar el bucle principal de Tkinter
        self.ventana.mainloop()

        self.get_logger().info("holaaaaaaaaaaaaaa")
        self.get_logger().info(f"{self.respuesta}")

    #Funciones para cada boton
    def respuesta_1(self):
        msg = Int8()
        msg.data = 1
        self.publisher_resp.publish(msg)

    def respuesta_2(self):
        msg = Int8()
        msg.data = 2
        self.publisher_resp.publish(msg)

    def respuesta_3(self):
        msg = Int8()
        msg.data = 3
        self.publisher_resp.publish(msg)

    def color_naranja(self):
        self.sec_color.data += "n"

    def color_morado(self):
        self.sec_color.data += "m"

    def color_verde(self):
        self.sec_color.data += "v"

    def color_azul(self):
        self.sec_color.data += "a"

def publish_sec_color(publisher, string_data):
    #node.get_logger().info(f"{string_data}")
    if len(string_data.data) == 4:
        publisher.publish(string_data)


def main(args=None):
    rclpy.init(args=args)
    interfaz = interfaz_menu()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(interfaz)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    def spin_once():
        while rclpy.ok():
            rclpy.spin_once(interfaz)

    spin_thread = Thread(target=spin_once, daemon=True)
    spin_thread.start()

    color_thread = Thread(target=publish_sec_color,args=(interfaz.publisher_color, interfaz.sec_color))
    color_thread.start()

    spin_thread.join()  # Esperar a que el hilo de spin_once termine
    color_thread.join()  # Esperar a que el hilo de publicación termine

    #rclpy.spin(interfaz)
    interfaz.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)