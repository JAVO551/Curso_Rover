from dynamixel_sdk import *
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
LEN_GOAL_POSITION = 2 #Data lenght for position
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

        self.goal_position = [2172,2048,2048,2048] #Initial position for each dynamixel 
        self.prev_goal_position = [2048,2048,2048,2048] #Previous position for each dynamixel, used for odometry calculations
        self.DXL_ID = [1,3,4,2] #ID for all 4 motors #1&3 = left wheels, 4&2 = right wheels

        self.port_handler = PortHandler(DEVICE_NAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.port_handler.openPort()
        self.port_handler.setBaudRate(BAUDRATE)
        self.groupSyncWrite = GroupSyncWrite(self.port_handler,self.packet_handler,ADDR_GOAL_POSITION,LEN_GOAL_POSITION)

        self.diameter = 0.107
        self.radius = self.diameter / 2.0
        self.width = 0.34 #Rover measure of left wheels to right wheels
        self.height = 0.31 #Rover measure of center wheels to front wheels
        self.ppr = 4400
       
        self.meters_per_tick = (math.pi * self.diameter) / self.ppr #For encoders
        
        #Initial configurartion of dynamixels
        self.setup_dynamixel(self.DXL_ID)




        self.move_servos(1.0,1.0)

        




    def move_servos(self,linear, angular):
        if angular > 0.005 or angular < -0.005: #Poner un umbral
            
            wheel_information = self.get_wheel_configuration (linear,angular)   

        else :

            wheel_information = [[linear,linear,linear,linear,linear,linear],[0,0,0,0,0,0]]


        wheel_angles = self.radian_to_dynamixel (wheel_information[1])
            

        self.goal_position[0] = wheel_angles[2]
        self.goal_position[1] = wheel_angles[0]
        self.goal_position[2] = wheel_angles[5]
        self.goal_position[3] = wheel_angles[3]

        for c in range(NUM_SERVOS):
                param = [DXL_LOBYTE(int(round(self.goal_position[c]))),DXL_HIBYTE(int(round(self.goal_position[c])))]
                dxl_addparam_result = self.groupSyncWrite.addParam(self.DXL_ID[c], param)
                
            
        try:
            dxl_comm_result = self.groupSyncWrite.txPacket()
        except Exception as e:
            self.get_logger().error(f"Error en Dynamixel: {e}")
            return
        
        self.groupSyncWrite.clearParam()

        for c in range(NUM_SERVOS):
           
            time.sleep(0.5)
            posicion, result, error = self.packet_handler.read2ByteTxRx(self.port_handler,self.DXL_ID[c],ADDR_PRESENT_POSITION)
            if result == COMM_SUCCESS: #COMM_SUCCES ESTA DEFINIDO DENTRO DEL SDK, SU VALOR ES DE 0
                posicion = (posicion - 2048) * (360/4096)
                self.get_logger().info(f"Posición actual del servo {self.DXL_ID[c]}: {posicion}")



        self.port_handler.closePort()



    def get_wheel_configuration (self,linear,angular): #FUNCTION TO OBTAIN WHEEL INFORMATION
        
        radius = linear/angular
            
        radius_left_center = radius - (self.width/2)
        radius_right_center = radius + (self.width/2)

        if linear > 0.005 or linear < -0.005: #if linear!=0
            sign = radius/abs(radius)
            sign2 = linear/abs(linear)
            sign3 = 1
            sign4 = -1
            
        else:
            sign = angular/abs(angular)
            sign2 = 1
            sign3 = (angular/abs(angular))
            sign4 = sign3
            

        radius_left_frontal = math.sqrt(self.height**2+radius_left_center**2) * (sign)
        radius_right_frontal = math.sqrt(self.height**2+radius_right_center**2) * (sign)

        #Wheel angles
        angle_left_front = math.atan2(self.height,  radius_left_center* (sign)) * (sign)#Radians
        angle_left_rear = math.atan2(-self.height,radius_left_center* (sign))* (sign)
        angle_right_front = math.atan2(self.height,radius_right_center* (sign)) * (sign)
        angle_right_rear = math.atan2(-self.height,radius_right_center* (sign))* (sign)
        

        #Wheel speeds:
        v_lf = abs(radius_left_frontal * angular) * sign2
        v_lc = abs((radius_left_center-0.03) * angular) * sign2 * -sign4
        v_lr = abs(radius_left_frontal * angular)* sign2
        v_rf = abs(radius_right_frontal * angular)* sign2
        v_rc = abs((radius_right_center+0.03) * angular)* sign2 * sign3
        v_rr = abs(radius_right_frontal * angular)* sign2

        return [[v_lf,v_lc,v_lr,v_rf,v_rc,v_rr],[angle_left_front,0,angle_left_rear,angle_right_front,0,angle_right_rear]]

    def radian_to_dynamixel (self,angles): #Angles in radian to angles in bits for each dynamixel

        angles[0] = 2172 - (4096/(2*math.pi))* angles[0]
        angles[2] = 2048 - (4096/(2*math.pi))* angles[2]
        angles[3] = 2048 - (4096/(2*math.pi))* angles[3]
        angles[5] = 2048 - (4096/(2*math.pi))* angles[5] #2048

        return angles

        





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
        
        #Disable movement in dynamixel servos
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