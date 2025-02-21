# Controller - ROS2 Package

## Descripción
Este paquete ROS2 proporciona varios nodos para controlar y teleoperar un TurtleBot, además de interfaces y herramientas para interactuar con él.

## Requisitos
Antes de instalar y ejecutar este paquete, asegúrate de tener instaladas las siguientes dependencias:

### Software necesario
- ROS2 (Humble o compatible)
- Python3
- `colcon` (para compilar paquetes en ROS2)

### Dependencias de Python
El paquete usa las siguientes librerías:
- `rclpy`
- `tkinter`
- `matplotlib`

Si alguna de estas dependencias no está instalada, puedes instalarlas con:
```bash
pip install tkinter matplotlib
```

## Instalación
1. Clona este repositorio en tu workspace de ROS2:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/IELE3338-202510-G1/Taller-1.git
   ```
2. Compila el paquete con `colcon`:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```
3. Fuente el setup para que ROS2 reconozca el paquete:
   ```bash
   source install/setup.bash
   ```

## Ejecución de los nodos
El paquete incluye los siguientes nodos:

- **turtle_bot_teleop**: Control del TurtleBot con teclado  
  Ejecutar con:  
  ```bash
  ros2 run controller turtle_bot_teleop
  ```

- **turtle_bot_interface**: Interfaz gráfica para control  
  Ejecutar con:  
  ```bash
  ros2 run controller turtle_bot_interface
  ```

- **turtle_bot_player**: Reproducción de movimientos  
  Ejecutar con:  
  ```bash
  ros2 run controller turtle_bot_player
  ```
## Nota
   Es importante ejecutar los tres nodos al tiempo


