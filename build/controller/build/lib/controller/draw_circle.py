#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCirclenode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtlebot_cmdVel",10)
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Start")

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DrawCirclenode()
    rclpy.spin(node)
    rclpy.shutdown()

"""
if __name__ == "__main__":
    main()
"""