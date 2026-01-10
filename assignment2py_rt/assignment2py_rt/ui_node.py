#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
#import time

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')
        
        #self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_ui', 10)
        self.get_logger().info('Node Ready')
        
        
        self.user_interface()

    def user_interface(self):
        while rclpy.ok():
            print("\n--- ROBOT CONTROL MENU ---")
            print("1: Set new velocity")
            print("2: STOP robot")
            
            choice = input("Select an option: ")

            if choice == "1":
                self.set_new_velocity()
            elif choice == "2":
                self.stop_robot()
            else:
                print("Invalid option, try again.")

    def set_new_velocity(self):
        try:
            lin_x = float(input("Linear velocity (x): "))
            ang_z = float(input("Angular velocity (z): "))
            
            msg = Twist()
            msg.linear.x = lin_x
            msg.angular.z = ang_z
            
            self.publisher_.publish(msg)
            self.get_logger().info(f'Moving: linear={lin_x}, angular={ang_z}')
        except ValueError:
            print("Error: Please enter valid numbers.")

    def stop_robot(self):
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        self.get_logger().info('STOP')

def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()