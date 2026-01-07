#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Node Ready')
        
        
        self.user_interface()

    def user_interface(self):
        while rclpy.ok():
            try:
                lin_x = float(input("Linear velocity (x): "))
                ang_z = float(input("Angular velocity (z): "))
                
                msg = Twist()
                msg.linear.x = lin_x
                msg.angular.z = ang_z
                
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: lin={lin_x}, ang={ang_z} for 5 seconds')

                time.sleep(5)

                stop_msg = Twist()
                self.publisher_.publish(stop_msg)
                self.get_logger().info('Robot stopped')
            except ValueError:
                print("Enter a valid number")

def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()