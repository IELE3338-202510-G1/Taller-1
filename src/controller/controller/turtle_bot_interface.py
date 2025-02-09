#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pcl_msgs.srv import UpdateFilename
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter.messagebox as messagebox
import tkinter as tk
from tkinter import filedialog
import threading
import os

class TurtleBotInterface(Node):
    def __init__(self):
        super().__init__("turtle_bot_interface")

        # Suscripción a la posición del TurtleBot2
        self.subscription = self.create_subscription(Twist, "/turtlebot_position", self.position_callback, 10)

        # Cliente para el servicio UpdateFilename
        self.cli = self.create_client(UpdateFilename, 'update_filename')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio update_filename...')
        
        # Almacenar la trayectoria
        self.x_data = []
        self.y_data = []

        self.x_vel = []
        self.z_vel = []

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

        # Preguntar si se desea guardar el recorrido al inicio
        self.save_trajectory = messagebox.askyesno("Guardar Recorrido", "¿Desea guardar el recorrido?")

        # Suscripción a la posición del TurtleBot2
        self.subscription_vel = self.create_subscription(Twist, "/turtlebot_cmdVel", self.velocity_callback, 10)

        # Botón para guardar la gráfica
        self.save_image = tk.Button(self.root, text="Guardar Gráfica", command=self.save_plot)
        self.save_image.pack(side=tk.LEFT, padx=5)

        # Botón para terminar el recorrido y cerrar el programa
        self.finish_button = tk.Button(self.root, text="Terminar Recorrido", command=self.finish_and_save)
        self.finish_button.pack(side=tk.LEFT, padx=5)

        # Botón para terminar el recorrido y cerrar el programa
        self.seleccionar_archivo = tk.Button(self.root, text="Recorrer Archivo", command=self.select_file)
        self.seleccionar_archivo.pack(side=tk.LEFT, padx=5)

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


    def velocity_callback(self, msg: Twist):
        """ Callback para recibir la posición del TurtleBot2 y actualizar los datos """
        self.x_vel.append(msg.linear.x)
        self.z_vel.append(msg.angular.z)
        
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
        file_path = filedialog.asksaveasfilename(defaultextension=".png", filetypes=[("PNG files", ".png"), ("All Files", ".*")])
        if file_path:
            self.fig.savefig(file_path)
            print(f"Gráfica guardada como {file_path}")
        else:
            print("Guardado cancelado.")

    def finish_and_save(self):
        """ Guarda el recorrido si el usuario eligió hacerlo y cierra el programa """
        if self.save_trajectory:  # Si el usuario eligió guardar al inicio
            file_path = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text files", ".txt"), ("All Files", ".*")])
            if file_path:
                with open(file_path, "w") as f:
                    f.write("vel_x, vel_z\n")  # Encabezado
                    for vel_x, vel_z in zip(self.x_vel, self.z_vel):
                        f.write(f"{vel_x}, {vel_z}\n")
                print(f"Recorrido guardado en {file_path}")

        print("Cerrando el programa...")
        self.on_closing()  # Cierra la interfaz y apaga ROS

    def select_file(self):
        file_path = filedialog.askopenfilename(title="Seleccionar archivo", filetypes=[("Text Files", "*.txt")])
        if file_path:
            self.get_logger().info(f"Enviando archivo: {file_path}")
            req = UpdateFilename.Request()
            req.filename = file_path
            future = self.cli.call_async(req)
            future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Archivo enviado con éxito.")
            #else:
                #self.get_logger().info("Hubo un problema con la actualización del archivo.")
        except Exception as e:
            self.get_logger().error(f"Error al recibir respuesta: {e}")

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