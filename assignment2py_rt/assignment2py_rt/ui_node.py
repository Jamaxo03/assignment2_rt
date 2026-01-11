#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from assignment2_msgs.srv import SetThreshold
from assignment2_msgs.srv import GetAvgSpeed

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')

        # self values
        self.velocity_history = []
        
        # pub/srv/cli
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_ui', 10)
        self.cli_threshold = self.create_client(SetThreshold, 'set_threshold')
        self.cli_average = self.create_client(GetAvgSpeed, 'get_avg_speed')
        self.srv_average = self.create_service(GetAvgSpeed, 'get_avg_speed', self.get_avg_callback)

        #while not self.cli_threshold.wait_for_service(timeout_sec=1.0):
            #self.get_logger().info('Waiting for set_threshold service')


        self.get_logger().info('Node Ready')
        
        self.user_interface()

    def user_interface(self):
        while rclpy.ok():
            print("\n--- ROBOT CONTROL MENU ---")
            print("1: Set new velocity")
            print("2: STOP robot")
            print("3: Change safety threshold")
            print("4: Get average velocity (last 5)")
            
            choice = input("Select an option: ")

            if choice == "1":
                self.set_new_velocity()
            elif choice == "2":
                self.stop_robot()
            elif choice == "3":
                self.change_threshold()
            elif choice == "4":
                self.get_average()
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

            # store in history
            self.velocity_history.append((lin_x, ang_z))
            if len(self.velocity_history) > 5:
                self.velocity_history.pop(0)

        except ValueError:
            print("Error: Please enter valid numbers.")

    def stop_robot(self):
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        self.get_logger().info('STOP')

    def change_threshold(self):
        try:
            new_threshold = float(input("Insert new threshold value: "))

            request = SetThreshold.Request()
            request.new_threshold = new_threshold

            future = self.cli_threshold.call_async(request)

            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None and future.result().success:
                self.get_logger().info(f"Threshold set to {new_threshold}")
            else:
                self.get_logger().error("Failed to set threshold")

        except ValueError:
            print("Error: Please enter a valid number.")

    def get_average(self):
        
        request = GetAvgSpeed.Request()
        future = self.cli_average.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            res = future.result()
            print(f"Linear (Avg):  {res.avg_linear:.2f} m/s")
            print(f"Angular (Avg): {res.avg_angular:.2f} rad/s")
            print(f"Based on: {len(self.velocity_history)} inputs")
        else:
            self.get_logger().error("Service call failed")

    def get_avg_callback(self, request, response):
        # if no history, return zeros
        if not self.velocity_history:
            response.avg_linear = 0.0
            response.avg_angular = 0.0
            return response

        total_lin = sum(v[0] for v in self.velocity_history)
        total_ang = sum(v[1] for v in self.velocity_history)
        count = len(self.velocity_history)

        response.avg_linear = total_lin / count
        response.avg_angular = total_ang / count
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()