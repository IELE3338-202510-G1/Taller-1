#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import filedialog
import threading
import os

class TurtleBotInterface(Node):
    def __init__(self):
        super().__init__("turtle_bot_interface")

        # Suscripción a la posición del TurtleBot2
        self.subscription = self.create_subscription(Twist, "/turtlebot_position", self.position_callback, 10)

        # Almacenar la trayectoria
        self.x_data = []
        self.y_data = []

        # Crear la ventana principal con Tkinter
        self.root = tk.Tk()
        self.root.title("Visualización de la Trayectoria")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)  # Manejo de cierre de ventana

        # Crear la figura de Matplotlib
        self.fig, self.ax = plt.subplots(figsize=(5, 5))
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-2.5, 2.5)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_title("Trayectoria TurtleBot")
        self.ax.grid(True)
        self.line, = self.ax.plot([], [], marker="o", linestyle="-", color="b")

        # Agregar la figura a Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack()

        # Botón para guardar la gráfica
        self.save_button = tk.Button(self.root, text="Guardar Gráfica", command=self.save_plot)
        self.save_button.pack()

        # Iniciar ROS2 en un hilo separado
        self.ros_thread = threading.Thread(target=self.run_ros, daemon=True)
        self.ros_thread.start()

        # Iniciar actualización periódica de la gráfica
        self.root.after(500, self.update_plot)
        
        # Iniciar el loop de Tkinter
        self.root.mainloop()

    def position_callback(self, msg: Twist):
        """ Callback para recibir la posición del TurtleBot2 y actualizar los datos """
        self.x_data.append(msg.linear.x)
        self.y_data.append(msg.linear.y)

    def update_plot(self):
        """ Actualiza la gráfica de manera eficiente """
        self.line.set_xdata(self.x_data)
        self.line.set_ydata(self.y_data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw()
        self.root.after(100, self.update_plot)  # Llamar nuevamente después de 500ms

    def save_plot(self):
        """ Abre una ventana para elegir la ubicación y el nombre del archivo y guarda la gráfica """
        file_path = filedialog.asksaveasfilename(defaultextension=".png", filetypes=[("PNG files", "*.png"), ("All Files", "*.*")])
        if file_path:
            self.fig.savefig(file_path)
            print(f"Gráfica guardada como {file_path}")
        else:
            print("Guardado cancelado.")

    def run_ros(self):
        """ Función para ejecutar rclpy.spin en un hilo separado """
        rclpy.spin(self)

    def on_closing(self):
        """ Cierra la interfaz de Tkinter y detiene ROS2 """
        print("Cerrando la interfaz y ROS2...")
        self.destroy_node()
        rclpy.shutdown()
        self.root.quit()
        os._exit(0)


def main(args=None):
    rclpy.init(args=args)
    TurtleBotInterface()

if __name__ == '__main__':
    main()