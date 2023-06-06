

<p align="center">
Pick-and-Place-with-ROS2
  <h2 align="center">TFG Pick and place UR3e using ROS2 and computer vision</h2>

  <p align="center">
  Mario Sanchez Garcia UPM
  </p>
</p>
<br>

La motivación de este proyecto consiste en implementar diferentes aplicaciones de Pick and Place con el robot colaborativo UR3e y visión artificial. El objetivo es realizarlo con la ayuda de ROS2 en Python. 
La principal idea es reconocer varios objetos rectangulares de diferentes colores con ayuda de la cámara, convertir sus posiciones y que el robot sea capaz de realizar aplicaciones con estas en función de la petición del usuario en una interfaz gráfica. Además, se va a visualizar en tiempo real el comportamiento tanto de la cámara como del robot con Rviz.

# MATERIALES NECESARIOS

En cuánto a software, se necesitará tener instalada correctamente Python, ROS2, Opencv, DepthAI y Tkinter.
En cuánto a hardware, en este proyecto se ha utilizado: 
- Robot colaborativo UR3e.
- Cámara 3D con modelo OAK-D Lite AF.
- Herramienta Gripper HRC-03
- Estructura con soporte para la cámara, de forma que quede vertical al espacio de trabajo.

## DESCRIPCION DE PAQUETES

`my_func_nodes`  Nodos creados propios para el correcto y completo funcionamiento del proyecto. 

      ├── my_func_nodes 
      ├── camera.py: Nodo encargado de lanzar las imágenes que visualiza la cámara.
      ├── camera_pub_pos.py: Nodo encargado de analizar dichas imágenes y obetener la posición de los objetos. 
      ├── interfaz_menu.py: Nodo encargado de crear y lanzar la interfaz gráfica de usuario la cual se comunica con los demás nodos.
      └── control_robot_master.py: Nodo que controla el movimiento del robot mediante Moveit, así como su herramienta. Además, gestiona todas las comunicaciones internasy estado actual del robot.

      ├── resources
      ├── euler_to_quat.py: Calculadora que transforma vector de rotación a cuaternio para ajustar la orientación del gripper.
      └── Imágenes para la interfaz gráfica.

`my_moveit2_py` Recursos en relación a planificación de trayectoriads con Moveit.

      ├── my_moveit2_py
      ├── Moveit2_resources.py: Librería basada en Moveit con funciones para planificar y ejecutar rayectorias del robot.
      └── ur3e_model.py: Modelo confgiruado del robot UR3e para el correcto funcionamiento de la librería.

`my_robot_bringup_ms` Paquete de lanzamiento.

      ├── launch
      ├── control_robot.py: Archivo de lanzamiento principal del proyecto.
      └── launch_descripition_resources: Información complementaria.

      ├── config: Parámetros caracterísitcos para el funcionamiento correcto de los controladores.
      ├── argsforlaunching.yaml
      └── param_bringup.yaml

## DIAGRAMA DE NODOS

<img src="https://github.com/mariooot13/Pick-and-Place-with-ROS2/blob/tutorial/DIAGNODOS.png">

## GETTING STARTED

0) Este manual de usuario ha sido creado para aquellos usuarios que tienen una distribución de Ubuntu. En caso contrario, serán necesarias acciones adicionales. 

1) ROS2 Foxy y depedencias
En primer lugar, se necesita instalar ROS2 y todas sus dependencias de forma coorecta. En neste caso, se ha usado la distribución Foxy.

- Podrás encontrar la documentación de ROS2 Foxy para su instalación en el siguiente enlace: https://docs.ros.org/en/foxy/Installation.html

A continuación, se explicará como se llevar a cabo la instalación por terminal de todas las librerías mencionadas en requisitos.

- Python3 & Tkinter: 
sudo apt install python3-colcon-common-extensions

- OpenCV y dependencias de ROS2: 
Se recomienda clonar su repositorio: git clone https://github.com/opencv/opencv.git y, posteriormente configurarlo según la documentación de instalación.
Se puede consultar su documentación en https://opencv.org/

- DepthAI: Se debe clonar el repositorio de DepthAI para ROS2 y construirlo de forma apropiada.
git clone https://github.com/luxonis/depthai_ros.git

2) UR ROS2 Drivers

El siguiente paso es clonar los controladores de UR y configurar sus paquetes. Se puede realizar de la siguiente manera:

a) export COLCON_WS=~/workspace/ros_ur_driver
   mkdir -p $COLCON_WS/src
   
b) cd $COLCON_WS
  git clone -b foxy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
  rosdep install --ignore-src --from-paths src -y -r
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  source install/setup.bash 
  
- De forma adicional, puedes seguir las instrucciones del apartado Getting started de UR en el siguiente enlace: 
  https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy 
  
3) IMPORTANTE. Una vez, se tienen clonados los drivers de UR, es el momento de clonar este repositorio en la rama Tutorial.

a) Dirigite hacia /workspace/ros_ur_driver/src

b) Opción 1: download a zip and paste it inside src directory.

   Opción 2: git clone https://github.com/mariooot13/Pick-and-Place-with-ROS2/tree/tutorial
  
  ### RECOMENDACIONES

- Para llevar a cabo de forma correcta toda la visión artificial, es necesario calibrar previamente su sistema de referencia en el nodo de detección. Esto es estrictamente necesario dado que los sistemas de referencia del robot y de la cámara deben alinearse.

- Cualquier duda adicional que pueda surgir, está resuelta en la memoria de este trabajo de fin de grado adjunta al repositorio.

## CONSIDERACIONES PREVIAS

- El primer paso antes de lanzar nada, es llevar a cabo la correcta conexión de los dispositivos.

a) Conexión del robot: 
Dentro del archivo de lanzamiento se ha asignado una dirección IP la cual deberá ser la que tu selecciones en la tablet del robot.
Deberás conectar el cable Ethernet a tu ordenador, y visualizar que dirección IP tiene el ordenador.
En la tablet, necesitarás crear un programa con control externo en el asignes la dirección IP del ordenador.

b) Instalación de la herramienta:
La herramienta debe estar instalada en la tablet del robot.

c) Cámara:
Conecta la cámara medianete un cable con conexión USB y asegurate de su correcto posicionamiento.

  
## USO

0) Abre una terminal en tu ordenador.

2) Necesitarás escribir el siguiente comando por pantalla.

      source install/setup.bash 
      
2) Lanza el archivo de lanzamiento en dicha terminal.

      ros2 launch my_robot_bringup_ms control_robot.py
      
3) Dale al PLAY en el programa del robot en la tablet.
