#
# ROVER LUNARES PARA EL APRENDIZAJE - ENP6-UNAM, 2026
# MOVIMIENTO DEL ROVER CON PWM
#
#El siguieunte código implementa un nodo de ROS2 que permite controlar el movimiento del rover utilizando mensajes de tipo Twist para publicar velocidades lineales y angulares. 
#El nodo incluye funciones para mover el rover hacia adelante y para girar en su propio eje, calculando el tiempo necesario para realizar cada movimiento basado en la velocidad deseada y la distancia o ángulo a recorrer. 
#Además, se incluye una función para detener el rover después de completar cada movimiento.
#Revisar la linea 36 del código y seguir sus instrucciones.


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math


class ObjectiveMovementPWM(Node):
    def __init__(self):
        super().__init__('objective_movement_pwm')
        


        self.publisher_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        self.move_time = 0.0 #Calculo de la cantidad de tiempo que se debe mover el rover
        self.rover_speed = 0.5 # Velocidad de desplazamiento para el rover en m/s o rad/s dependiendo de la función
        self.radius_turn = 3.4 # radio físico del rover para un giro en su propio eje

        self.time_resolution = 0.1
        self.calibration = 1.5 
        
        
        self.get_logger().info("Nodo de distancia objectivo iniciado")

        #Acontinuación implemente la secuencia de funciones correctas para mover el rover con el fin de describir un cuadrado de 1 metro de lado.
        #Las funciones deben estar escritas al terminar estos comentarios con el mismo identado.
        #El cuadraddo se puede implementar con con 4 giros de 90 grados y 4 movimientos hacia adelante. 
        
        #La función move_foward se encarga de mover el rover segun una distancia dada en metros.
        #La función move_turn se encarga de girar el rover en su propio eje segun el un ángulo dado en radianes.

        self.move_forward(1.0)
        self.move_turn(-1.5708)
        self.move_forward(1.0)
        self.move_turn(-1.5708)
        self.move_forward(1.0)
        self.move_turn(-1.5708)
        self.move_forward(1.0)
        self.move_turn(-1.5708)
        

    
    def move_forward (self, distance): #Función para mover el rover hacia adelante o atras.
        msg = Twist()    
        self.move_time = abs(distance / self.rover_speed)
        
        self.pulses= self.move_time/self.time_resolution 
        
        if distance < 0:
            self.rover_speed = -0.5
        else:
            self.rover_speed = 0.5

        msg.linear.x = self.rover_speed
        msg.angular.z = 0.0
        i=0
        for i in range(int(self.pulses)):
            self.publisher_vel.publish(msg)
            time.sleep(self.time_resolution*self.calibration)
        self.robot_stop()
        return
    
    def move_turn (self, angle): #Función para girar el rover en su propio eje.
        msg = Twist() 
        self.move_time = abs(angle / self.rover_speed) 
        self.pulses= self.move_time/self.time_resolution 
        #self.get_logger().info(f"Tiempo de movimiento: {self.move_time}")
        
        
        if angle < 0:
            self.rover_speed = -0.5
        else:
            self.rover_speed = 0.5

        msg.angular.z = self.rover_speed
        msg.linear.x = 0.0
        i=0

        for i in range(int(self.pulses)):
            self.publisher_vel.publish(msg)
            time.sleep(self.time_resolution*1.7)
        self.robot_stop()
        return


    def robot_stop(self): #Función para detener cualquier movimiento del rover.
        msg = Twist() 
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_vel.publish(msg)
        time.sleep(1)
        return

        


def main(args=None):
    rclpy.init(args=args)
    node = ObjectiveMovementPWM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()