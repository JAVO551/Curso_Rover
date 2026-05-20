import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from std_msgs.msg import Bool
import time
import math


class ObjectiveMovementPWM(Node):
    def __init__(self):
        super().__init__('objective_movement_pwm')
        


        self.publisher_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move_time = 0.0
        self.rover_speed = 0.5 # 1 m/s is 127 pwm in mobile base
        self.time_resolution = 0.1
        self.calibration = 1.5 #1.7
        self.radius_turn = 3.4 # radius for turning in m
        
        self.get_logger().info("Nodo de distancia objectivo iniciado")

        self.move_forward(1.0)
        

    def move_forward (self, distance):
            msg = Twist()    
            self.move_time = (abs(distance) / self.rover_speed)
            
            self.pulses= self.move_time/self.time_resolution 
            
            if distance < 0:
                self.rover_speed = -self.rover_speed
            else:
                self.rover_speed = self.rover_speed

            msg.linear.x = self.rover_speed
            msg.angular.z = 0.0
            

            for i in range(int(self.pulses)):
                self.publisher_vel.publish(msg)
                time.sleep(self.time_resolution*self.calibration)
            self.robot_stop()

    def move_turn (self, angle): 
                msg = Twist() 
                self.move_time = (abs(angle) * self.radius_turn / self.rover_speed) 
                self.pulses= self.move_time/self.time_resolution 
                
                if angle < 0:
                    self.rover_speed = -self.rover_speed
                else:
                    self.rover_speed = self.rover_speed

                msg.linear.z = self.rover_speed
                msg.angular.x = 0.0
                

                for i in range(int(self.move_time)):
                    self.publisher_vel.publish(msg)
                    time.sleep(self.time_resolution)
                self.robot_stop()


    def robot_stop(self):
        msg = Twist() 
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_vel.publish(msg)

        


def main(args=None):
    rclpy.init(args=args)
    node = ObjectiveMovementPWM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()