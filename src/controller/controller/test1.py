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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Configuración del teclado
settings = termios.tcgetattr(sys.stdin)

# Mensaje de instrucciones
msg = """
Control del TurtleBot con el teclado
-------------------------------------
W: Avanzar    S: Retroceder
A: Girar izq  D: Girar der
ESPACIO: Detener

Q/E: Aumentar/Disminuir velocidad lineal
Z/C: Aumentar/Disminuir velocidad angular

CTRL+C para salir
"""

# Diccionario de controles de movimiento
moveBindings = {
    'w': (1, 0),  # Adelante
    's': (-1, 0),  # Atrás
    'a': (0, 1),  # Girar izquierda
    'd': (0, -1),  # Girar derecha
}

# Diccionario de ajuste de velocidades
speedBindings = {
    'q': (1.1, 1.0),  # Aumentar velocidad lineal
    'e': (0.9, 1.0),  # Disminuir velocidad lineal
    'z': (1.0, 1.1),  # Aumentar velocidad angular
    'c': (1.0, 0.9)   # Disminuir velocidad angular
}

def getKey():
    """ Captura la tecla presionada sin necesidad de presionar Enter """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0.1)  # Espera por una tecla durante 0.1s
    key = sys.stdin.read(1) if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0] else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopKeyboard(Node):
    
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtlebot_cmdVel", 10)

        self.speed = 0.2  # Velocidad lineal inicial
        self.turn = 0.5   # Velocidad angular inicial
        self.timer_ = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        key = getKey()

        if key in moveBindings:
            x, th = moveBindings[key]
        else:
            x, th = 0, 0  # Si no hay tecla, el robot se detiene

        if key in speedBindings:
            self.speed *= speedBindings[key][0]
            self.turn *= speedBindings[key][1]
            print(f"Velocidad lineal: {self.speed:.2f}, Velocidad angular: {self.turn:.2f}")

        # Publicar velocidad
        twist = Twist()
        twist.linear.x = x * self.speed
        twist.angular.z = th * self.turn
        self.cmd_vel_pub_.publish(twist)

def main():
    print(msg)
    rclpy.init()
    node = TeleopKeyboard()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
