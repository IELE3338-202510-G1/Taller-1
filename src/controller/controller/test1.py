#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class test1(Node):

    def __init__(self):
        super().__init__("test1")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtlebot_cmdVel",10)
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Start")

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 2.0
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = test1()
    rclpy.spin(node)
    rclpy.shutdown()