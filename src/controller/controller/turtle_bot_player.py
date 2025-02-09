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

        # Variables para almacenar los comandos de velocidad
        self.velocities = []
        self.current_index = 0

        # Temporizador para publicar los comandos de velocidad
        self.timer = self.create_timer(0.03, self.publish_velocity)  # 0.03 segundos = 30 ms

    def filename_callback(self, request, response):
        file_path = request.filename
        self.get_logger().info(f"Recibido archivo: {file_path}")
        
        try:
            with open(file_path, 'r') as file:
                lines = file.readlines()[1:]  # Omitir encabezado
                self.velocities = [tuple(map(float, line.strip().split(','))) for line in lines]
                self.current_index = 0
                self.get_logger().info(f"Archivo cargado con {len(self.velocities)} comandos de velocidad.")
        except Exception as e:
            self.get_logger().error(f"Error al leer el archivo: {e}")
            
        return response

    def publish_velocity(self):
        """ Publica el siguiente comando de velocidad en la lista """
        if self.current_index < len(self.velocities):
            vel_x, vel_z = self.velocities[self.current_index]
            twist = Twist()
            twist.linear.x = vel_x
            twist.angular.z = vel_z
            self.cmd_vel_pub_.publish(twist)
            self.current_index += 1
        else:
            # Si ya se publicaron todos los comandos, detener el robot
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub_.publish(twist)

def main():
    rclpy.init()
    node = TurtleBotPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()