import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Bool, Int16
import time
import math


SM_WAITING = 0      
SM_APPROACHING = 1  
SM_ARRIVED = 2      

class PathPlanner(Node):
    def __init__(self):
        super().__init__('goal_follower')
        
        
        self.subscription = self.create_subscription(
            PointStamped,
            '/vision/target_flag',
            self.target_callback,
            10)
        
        #/vision/target_flag
        #/vision/start_flag
        
        self.subscription_move = self.create_subscription(
            Bool,
            '/follow_goal',
            self.move_callback,
            10)
            
        
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.publisher_goal_reached = self.create_publisher(Bool, '/registered_goal', 10)
        
        
        
        self.state = SM_WAITING
        self.last_msg_time = time.time()
        self.target_x = 0.0
        self.target_z = 0.0

        
        self.linear_max = 1.0
        self.angular_max = 1.0
        #Speed profile parameters
        self.des_accel_distance = 1.0
        self.acel = 0.15
        self.current_speed = 0.0
        self.goal_tolerance = 1.5
        self.Kp = 0.6  
        self.move = False
        
        
        self.timer = self.create_timer(0.1, self.control_loop)
        

    def target_callback(self, msg):
        self.target_x = msg.point.x
        self.target_z = msg.point.z
        self.last_msg_time = time.time()
        self.get_logger().info(f"Target FLAG received: x={self.target_x:.3f}, z={self.target_z:.3f}")


    def move_callback(self, msg):
        self.move = True
        self.get_logger().info(f"Follow goal command received: {msg.data}")
          


    def control_loop(self):
        now = time.time()
        msg = Twist()
        msg_goal = Bool ()
        msg_state = Int16 ()

        if (now - self.last_msg_time) > 1.0:
                self.state = SM_WAITING
                self.number_point = 0
                self.move = False
                #self.get_logger().info('Esperando nuevo objetivo...')
        elif self.move == True:
                #Speed profile
                self.state = SM_APPROACHING
                
                if self.target_z < self.des_accel_distance:
                    self.get_logger().info("Entrando en zona de desaceleracion")
                    if self.target_z < self.goal_tolerance:
                        self.get_logger().info("Current speed 0.0")
                        self.current_speed = 0.0
                        self.state = SM_ARRIVED
                    else:
                        self.current_speed = self.Kp * self.target_z 
                else:

                    if self.current_speed < self.linear_max:
                        self.get_logger().info("Acelerando de a poco")
                        self.current_speed = self.current_speed + self.acel
                        
                    else:
                         self.current_speed = self.linear_max
                         self.get_logger().info("Tope de vel")

        else: 
              self.state = SM_WAITING

        if self.state == SM_WAITING:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                #self.publisher_vel.publish(msg)
            
        elif self.state == SM_APPROACHING:
                msg.linear.x = self.current_speed * math.exp(-(self.target_x**2)/0.7)
                msg.angular.z = self.angular_max * ((2/(1+math.exp(self.target_x/0.8))) - 1)
                self.publisher_vel.publish(msg)
                #self.get_logger().info(f"APPROACHING FLAG: x={self.target_x:.3f}, z={self.target_z:.3f}, v={msg.linear.x:.3f}, w={msg.angular.z:.3f}")

        elif self.state == SM_ARRIVED:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_vel.publish(msg)
                self.move = False
                #self.get_logger().info('FIN', throttle_duration_sec=2.0)
                self.stop_robot()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_vel.publish(msg)
                msg_goal.data = True
                self.publisher_goal_reached.publish(msg_goal)
                #self.get_logger().info(f"Ruta completa: {self.path_traveled}")
        
                

    def stop_robot(self):
        self.publisher_vel.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()