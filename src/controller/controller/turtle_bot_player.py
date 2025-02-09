#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pcl_msgs.srv import UpdateFilename
import time

class TurtleBotPlayer(Node):
    def __init__(self):
        super().__init__('turtle_bot_player')
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/turtlebot_cmdVel', 10)
        self.srv = self.create_service(UpdateFilename, 'update_filename', self.filename_callback)
        self.get_logger().info("TurtleBotPlayer iniciado y esperando archivos...")

    def filename_callback(self, request, response):
        file_path = request.filename
        self.get_logger().info(f"Recibido archivo: {file_path}")
        
        try:
            with open(file_path, 'r') as file:
                lines = file.readlines()[1:]  # Omitir encabezado
                
                for line in lines:
                    vel_x, vel_z = map(float, line.strip().split(','))
                    twist = Twist()
                    twist.linear.x = vel_x
                    twist.angular.z = vel_z
                    self.cmd_vel_pub_.publish(twist)
                    time.sleep(0.0275)  # Simula el envío pausado de comandos
        except Exception as e:
            self.get_logger().error(f"Error al leer el archivo: {e}")
            
        return response


def main():
    rclpy.init()
    node = TurtleBotPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()