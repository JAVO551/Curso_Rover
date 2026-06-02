#
# ROVER LUNARES PARA EL APRENDIZAJE - ENP6-UNAM, 2026
# CONTROL DE SERVOS PARA EL ROVER
#
#El siguieunte código implementa un nodo de ROS2 que permite controlar los servos del rover utilizando mensajes de tipo Twist para publicar velocidades lineales y angulares.
#El código incluye las funciones necesarias para trabajar con los servo motores Dynamixel MX-64, utilizando su SDK para enviar comandos y leer información de posición.
#Se implentan 2 funciones que permiten calcular el ángulo deseado y una segunda para imprimir el ángulo real del servomotor.
#Revisar la linea 56 del código y seguir sus instrucciones.

from dynamixel_sdk import *
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
LEN_GOAL_POSITION = 2 
ADDR_PRESENT_POSITION = 36
PROTOCOL_VERSION = 1.0  
BAUDRATE = 57600  
DEVICE_NAME = '/dev/ttyUSB0'  
TORQUE_ENABLE = 1 
TORQUE_DISABLE = 0  
NUM_SERVOS = 4

class ServosNode(Node):

    def __init__(self):
        super().__init__('servo_node')

        self.goal_position = [2172,2048,2048,2048] 
        self.prev_goal_position = [2048,2048,2048,2048] 
        self.DXL_ID = [1,3,4,2] 

        self.port_handler = PortHandler(DEVICE_NAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.port_handler.openPort()
        self.port_handler.setBaudRate(BAUDRATE)
        self.groupSyncWrite = GroupSyncWrite(self.port_handler,self.packet_handler,ADDR_GOAL_POSITION,LEN_GOAL_POSITION)

        self.diameter = 0.107
        self.radius = self.diameter / 2.0
        self.width = 0.34 
        self.height = 0.31 
        self.ppr = 4450
       
        self.meters_per_tick = (math.pi * self.diameter) / self.ppr 
        
        #Posición y configuración inicial de los servos.
        self.setup_dynamixel(self.DXL_ID)

        time.sleep(2)

        #1- Revise la linea 150 del código, se encuentra en la función move_servos, realice el ejercicio propuesto para completar el código.

        #2- Modifique la llamada a la función self.move_servos para imprimir el ángulo real de cada servo.
        #   La función move_servos recibe como primer argumento la velocidad lineal en m/s y como segundo argumento la velocidad angular en rad/s.
        #   Revise la información impresa en la terminal y compare los ángulos reales con los calculados en la clase, revise si la orientación es correcta.


        self.move_servos(0.0,0.0)



        




    def move_servos(self,linear, angular):
        posicion_real = [0.0,0.0,0.0,0.0]
        if angular > 0.005 or angular < -0.005: 
            wheel_information = self.get_wheel_configuration (linear,angular)   

        else :
            wheel_information = [[linear,linear,linear,linear,linear,linear],[0,0,0,0,0,0]]


        wheel_angles = self.radian_to_dynamixel (wheel_information[1])
            

        self.goal_position[0] = wheel_angles[0]
        self.goal_position[1] = wheel_angles[2]
        self.goal_position[2] = wheel_angles[3]
        self.goal_position[3] = wheel_angles[5]

        for c in range(NUM_SERVOS):
                param = [DXL_LOBYTE(int(round(self.goal_position[c]))),DXL_HIBYTE(int(round(self.goal_position[c])))]
                dxl_addparam_result = self.groupSyncWrite.addParam(self.DXL_ID[c], param)   

        #Commando para mover los servomotores a la posición deseada.  
        try:
            dxl_comm_result = self.groupSyncWrite.txPacket()
        except Exception as e:
            self.get_logger().error(f"Error en Dynamixel: {e}")
            return
        
        self.groupSyncWrite.clearParam()

        for c in range(NUM_SERVOS):
           
            time.sleep(0.5)

            #Se imprime el valor que el servomotor lee como posición actual, se convierte a grados para una mejor interpretación.
            posicion, result, error = self.packet_handler.read2ByteTxRx(self.port_handler,self.DXL_ID[c],ADDR_PRESENT_POSITION)
            if result == COMM_SUCCESS:
                if c == 0:
                    posicion_real[c] = (posicion - 2172) * (360/4096)
                else:
                    posicion_real[c] = (posicion - 2048) * (360/4096)
        self.port_handler.closePort()

        print ("Ángulo llanta izquierda frontal:",posicion_real[0])
        print ("Ángulo llanta izquierda trasera:",posicion_real[1])
        print ("Ángulo llanta derecha frontal:",posicion_real[2])
        print ("Ángulo llanta derecha trasera:",posicion_real[3])




    #Cálculos matemáticos para la el ángulo deseado de cada servo
    def get_wheel_configuration (self,linear,angular): 
        
        #Cálculo del radio de giro del rover, se utiliza la relación entre la velocidad lineal y angular para obtener el radio de giro.
        radius = linear/angular
            
        #Distancia del centro del círculo a cada lateral del rover.
        radius_left_center = radius - (self.width/2)
        radius_right_center = radius + (self.width/2)

        #Los siguientes cálculo se utilizan para corregir errores en la lóica de cálculo al programase en código
        if linear > 0.005 or linear < -0.005: 
            sign = radius/abs(radius)
            sign2 = linear/abs(linear)
            sign3 = 1
            sign4 = -1
            
        else:
            sign = angular/abs(angular)
            sign2 = 1
            sign3 = (angular/abs(angular))
            sign4 = sign3
            
        #Cálculo de la distancia del centro del círculo a cada rueda frontal, se utiliza el teorema de Pitágoras para obtener esta distancia.
        radius_left_frontal = math.sqrt(self.height**2+radius_left_center**2) * (sign)
        radius_right_frontal = math.sqrt(self.height**2+radius_right_center**2) * (sign)

        #Cálculo de los ángulos deseados para cada servo.
        #Descomente las la cuarteta de lineas correctas para calcular los ángulos deaseados.
        #Las opciones a) y b) solo se diferencian en la función trigonométrica utilizada.

        #a)
        # angle_left_front = math.atan2(self.height,  radius_left_center* (sign)) * (sign)
        # angle_left_rear = math.atan2(-self.height,radius_left_center* (sign))* (sign)
        # angle_right_front = math.atan2(self.height,radius_right_center* (sign)) * (sign)
        # angle_right_rear = math.atan2(-self.height,radius_right_center* (sign))* (sign)

        #b)
        # angle_left_front = math.cos(self.height,  radius_left_center* (sign)) * (sign)
        # angle_left_rear = math.cos(-self.height,radius_left_center* (sign))* (sign)
        # angle_right_front = math.cos(self.height,radius_right_center* (sign)) * (sign)
        # angle_right_rear = math.cos(-self.height,radius_right_center* (sign))* (sign)
        

        #Velocidad de cada rueda (no se usa esta información en este código):
        v_lf = abs(radius_left_frontal * angular) * sign2
        v_lc = abs((radius_left_center) * angular) * sign2 * -sign4
        v_lr = abs(radius_left_frontal * angular)* sign2
        v_rf = abs(radius_right_frontal * angular)* sign2
        v_rc = abs((radius_right_center) * angular)* sign2 * sign3
        v_rr = abs(radius_right_frontal * angular)* sign2

        return [[v_lf,v_lc,v_lr,v_rf,v_rc,v_rr],[angle_left_front,0,angle_left_rear,angle_right_front,0,angle_right_rear]]
    

    #Función para convertir los ángulos calculados a valores que el Dynamixel pueda interpretar
    def radian_to_dynamixel (self,angles): 

        angles[0] = 2172 - (4096/(2*math.pi))* angles[0]
        angles[2] = 2048 - (4096/(2*math.pi))* angles[2]
        angles[3] = 2048 - (4096/(2*math.pi))* angles[3]
        angles[5] = 2048 - (4096/(2*math.pi))* angles[5] 

        return angles

    #Función para la configuración del los servos al iniciar el código.
    def setup_dynamixel(self, dxl_id):
        
        for c in range(NUM_SERVOS):
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id[c], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        
        for c in range(NUM_SERVOS):
            
            param = [DXL_LOBYTE(int(round(self.goal_position[c]))),DXL_HIBYTE(int(round(self.goal_position[c])))]
            dxl_addparam_result = self.groupSyncWrite.addParam(self.DXL_ID[c], param)
            if dxl_addparam_result != True:
                self.get_logger().error(f'Failed to addparam for ID {self.DXL_ID[c]}')
                return

        try:
            dxl_comm_result = self.groupSyncWrite.txPacket()
        except Exception as e:
            self.get_logger().error(f"Error en Dynamixel: {e}")
            return
        
        self.groupSyncWrite.clearParam()


    
    def __del__(self):
        for c in range(NUM_SERVOS):
            self.packet_handler.write1ByteTxOnly(self.port_handler,
                                           self.DXL_ID[c],
                                           ADDR_TORQUE_ENABLE,
                                           TORQUE_DISABLE)
        
        self.port_handler.closePort()
        self.get_logger().info('Shutting down read_write_node')


def main(args=None):
    rclpy.init(args=args)
    node = ServosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()