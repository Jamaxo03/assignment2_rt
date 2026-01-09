#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from assignment2_msgs.msg import RobotStatus
from assignment2_msgs.srv import SetThreshold
import time

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        
        # Default
        self.threshold = 1.0 
        self.is_escaping = False
        
        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        self.pub_status = self.create_publisher(RobotStatus, '/robot_status', 10)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.srv_threshold = self.create_service(SetThreshold, 'set_threshold', self.change_threshold_callback)

    def change_threshold_callback(self, request, response):
        self.threshold = request.new_threshold
        self.get_logger().info(f'New threshold set: {self.threshold}')
        response.success = True
        return response

    def laser_callback(self, msg):

        # flag
        if self.is_escaping:
            return
        
        # filter inf values
        ranges = [r if r < msg.range_max else msg.range_max for r in msg.ranges]
        size = len(ranges)
    
        # laser start from back of robot and goes anti-clockwise
        # [0:1/8] + [7/8:1]
        back_part = ranges[0:size//8] + ranges[7*size//8:size]
        min_back = min(back_part)
    
        # [1/8:3/8]
        min_right = min(ranges[size//8 : 3*size//8])
    
        # [3/8:5/8]
        min_front = min(ranges[3*size//8 : 5*size//8])

        # [5/8:7/8]
        min_left = min(ranges[5*size//8 : 7*size//8])

        dist_min = min(min_front, min_right, min_left, min_back)
    
        if dist_min == min_front:
            direction = "front"
        elif dist_min == min_left:
            direction = "left"
        elif dist_min == min_right:
            direction = "right"
        else:
            direction = "back"

        status_msg = RobotStatus()
        status_msg.distance = float(dist_min)
        status_msg.direction = direction
        status_msg.threshold = self.threshold
        self.pub_status.publish(status_msg)

        if dist_min < self.threshold:
            self.is_escaping = True
            self.execute_escape_maneuver(direction)
            self.is_escaping = False
    
    def execute_escape_maneuver(self, direction):
        self.get_logger().warn(f"DANGER -> {direction.upper()}! Escape maneuver initiated.")
        
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub_vel.publish(msg)

        rotation_speed = 3.14 
        escape_speed = 0.5     
        
        # Rotation
        if direction == "front":
            
            msg.angular.z = rotation_speed
            self.pub_vel.publish(msg)
            time.sleep(3.14/rotation_speed)
            
        elif direction == "right":
           
            msg.angular.z = rotation_speed
            self.pub_vel.publish(msg)
            time.sleep(1.57/rotation_speed)

        elif direction == "left":
            
            msg.angular.z = -rotation_speed
            self.pub_vel.publish(msg)
            time.sleep(1.57/rotation_speed)

        # Escape
        msg.angular.z = 0.0
        msg.linear.x = escape_speed
        self.pub_vel.publish(msg)
        time.sleep(1.0)

        # Stop
        self.pub_vel.publish(Twist())
        self.get_logger().info("Robot is safe now.")

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()