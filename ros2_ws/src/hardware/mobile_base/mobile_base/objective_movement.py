import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped, Twist, Point, PoseStamped, Pose
from nav_msgs.msg import Path
from rclpy.duration import Duration
from std_msgs.msg import Bool
import time
import math


SM_WAITING = 0      
SM_APPROACHING = 1  
SM_ARRIVED = 2 
SM_TURN = 3  

class ObjectiveMovementPWM(Node):
    def __init__(self):
        super().__init__('objective_movement')
        #Movement control using time and speed without feedback
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.read_tf)  #0.1 anteriormente

        #ros2 topic pub --once /objective_point geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 0.0}"


        self.subscription_forward = self.create_subscription(
            Point,
            'objective_forward',
            self.forward_callback,
            10)
        self.subscription_turn = self.create_subscription(
            Point,
            'objective_turn',
            self.turn_callback,
            10)
        self.subscription_start = self.create_subscription(
            Point,
            'objective_start',
            self.start_callback,
            10)
        
        self.subscription_stop = self.create_subscription(
            Bool,
            '/stop_movement',
            self.stop_callback,
            10
        )
            
        
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        
        self.msg_start = Point()
        self.msg_goal = Point()
        
        self.state = SM_WAITING
        self.last_msg_time = time.time()
        self.target_x = 0.0
        self.target_z = 0.0
        self.prev_target_x = 0.0
        self.prev_target_z = 0.0
        self.move_x = False
        self.move_t = False

        #Para el mapa
        self.resolution = 0.1


        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
       

        self.move_time = 0.0
        self.wheel_speed = 1.0 # 1 m/s is 127 pwm in mobile base
        self.time_resolution = 0.1
        self.radius_turn = 3.4 # radius for turning in m
        
        #Parael path



        
        
        
        self.timer = self.create_timer(0.05, self.control_loop)
        

    
    def stop_callback (self,msg):
        self.move_x = False
        self.move_t = False
        self.state = SM_ARRIVED
        self.last_msg_time = time.time()

    def forward_callback(self, msg):
        #Provisionalmente hacer la devolución de ruta con tópicos, actualizar a un servicio o action
        self.target_x = msg.x
        
        self.last_msg_time = time.time()
        #self.get_logger().info(f"Nuevo objetivo recibido: x={self.target_x}, y={self.target_y}")
        self.move_x = True
        self.move_t = False
        #self.control_loop()  # Llamar al control loop para procesar el nuevo path

    def turn_callback(self, msg):
        
        self.target_z = msg.z
        self.move_t = True
        self.move_x = False
        


    # def start_callback (self,msg):
    #     self.target_x = 0.0
    #     self.target_y = 0.0
    #     self.last_msg_time = time.time()
        
    #     self.move = True


        



    # def read_tf(self):
    #     try:
    #         t = self.tf_buffer.lookup_transform(
    #             "odom",       # frame_id
    #             "base_link",  # frame_id child
    #             rclpy.time.Time()
    #         )

    #         self.robot_x = t.transform.translation.x
    #         self.robot_y = t.transform.translation.y

    #         q = t.transform.rotation

    #         # quathernion to euler (yaw)
    #         self.robot_theta = 2 * math.atan2(q.z, q.w)

    #         #print (f"x: {self.robot_x:.2}, y: {self.robot_y:.2}, theta: {self.robot_theta:.2}")

    #         if self.robot_x-self.prev_x > 0.1 or self.robot_y-self.prev_y > 0.1:
                
    #             self.path_traveled.append((self.robot_x, self.robot_y))
    #             self.prev_x = self.robot_x
    #             self.prev_y = self.robot_y



                

        # except Exception as e:
        #     return
        #     #self.get_logger().warn(f"No TF available: {str(e)}")





    def control_loop(self):
            
        
            
            # MÁQUINA DE ESTADOS
            if self.target_x == self.prev_target_x and self.target_z == self.prev_target_z:
                self.state = SM_WAITING

            elif self.move_x == True:
                #Speed profile
                self.state = SM_APPROACHING

            elif self.move_t == True:
                self.state = SM_TURN
            else:
                self.state = SM_ARRIVED

            msg = Twist()            


            
        
            # ACCIONES POR ESTADO
            if self.state == SM_WAITING:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                #self.publisher_vel.publish(msg)
            
            elif self.state == SM_APPROACHING:

                self.move_time = (abs(self.target_x) / self.wheel_speed)/self.time_resolution # 127 pwm in mobile base = 1 m/s

                if self.target_x < 0:
                    self.wheel_speed = -1.0
                else:
                    self.wheel_speed = 1.0


                for i in range(int(self.move_time)):
                    msg.linear.x = self.wheel_speed
                    msg.angular.z = 0.0
                    self.publisher_vel.publish(msg)
                    time.sleep(self.time_resolution)

            elif self.state == SM_TURN:
                self.move_time = (abs(self.target_z) * self.radius_turn / self.wheel_speed)/self.time_resolution # 127 pwm in mobile base = 1 m/s
                
                if self.target_z < 0:
                    self.wheel_speed = -1.0
                else:
                    self.wheel_speed = 1.0

                for i in range(int(self.move_time)):
                    msg.linear.z = self.wheel_speed
                    msg.angular.x = 0.0
                    self.publisher_vel.publish(msg)
                    time.sleep(self.time_resolution)
                

            elif self.state == SM_ARRIVED:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_vel.publish(msg)
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_vel.publish(msg)
                self.move_x = False
                self.move_t = False
                
                self.stop_robot()
               
                
                self.prev_target_x = self.target_x
                self.prev_target_z = self.target_z

          

    def stop_robot(self):
        self.publisher_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ObjectiveMovementPWM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()