#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from assignment2_msgs.msg import RobotStatus
from assignment2_msgs.srv import SetThreshold

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        
        # self values
        self.threshold = 1.0 
        self.state = "IDLE"      
        self.state_timer = 0.0   
        self.last_ui_cmd = Twist()
        self.escape_msg = Twist()
        self.current_direction = None

        # pub/sub/srv
        self.sub_ui = self.create_subscription(Twist, '/cmd_vel_ui', self.ui_cmd_callback, 10)
        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.pub_status = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.srv_threshold = self.create_service(SetThreshold, 'set_threshold', self.change_threshold_callback)
        
        # timer 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Safety Node (State Machine) Ready.")

    def ui_cmd_callback(self, msg):
        
        self.last_ui_cmd = msg

    def laser_callback(self, msg):
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
    
        # get direction
        if dist_min == min_front:
            self.current_direction = "front"
        elif dist_min == min_left:
            self.current_direction = "left"
        elif dist_min == min_right:
            self.current_direction = "right"
        else:
            self.current_direction = "back"

        status_msg = RobotStatus()
        status_msg.distance = float(dist_min)
        status_msg.direction = self.current_direction
        status_msg.threshold = self.threshold
        self.pub_status.publish(status_msg)

        # change state
        if dist_min < self.threshold and self.state == "IDLE":
            self.last_ui_cmd = Twist()
            self.initiate_escape()

    def control_loop(self):

        self.get_logger().info(f"STATE={self.state} | UI=({self.last_ui_cmd.linear.x:.2f}, {self.last_ui_cmd.angular.z:.2f})")

        if self.state == "IDLE":
            
            self.pub_vel.publish(self.last_ui_cmd)

        elif self.state == "ROTATING":
            
            self.pub_vel.publish(self.escape_msg)

            # when direction is back -> start escaping
            if self.current_direction == "back":
                
                self.state = "ESCAPING"
                self.state_timer = 1.0   

        elif self.state == "ESCAPING":
            
            msg = Twist()
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.pub_vel.publish(msg)
            self.state_timer -= 0.1
            if self.state_timer <= 0:
                # safe
                self.last_ui_cmd = Twist() 
                self.state = "IDLE"
                self.get_logger().info("Robot is safe now.")

    def initiate_escape(self):
        self.get_logger().warn(f"DANGER -> {self.current_direction.upper()}! Switching to escape state.")
        self.state = "ROTATING"
        self.last_ui_cmd = Twist()
        self.escape_msg = Twist()
        
        # start rotating
        if self.current_direction == "front":
            self.escape_msg.angular.z = 3.14 
        elif self.current_direction == "right":
            self.escape_msg.angular.z = 3.14
        elif self.current_direction == "left":
            self.escape_msg.angular.z = -3.14
        elif self.current_direction == "back":
            self.state = "ESCAPING"
            self.state_timer = 1.0

    def change_threshold_callback(self, request, response):
        self.threshold = request.new_threshold
        self.get_logger().info(f'New threshold set: {self.threshold}')
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()