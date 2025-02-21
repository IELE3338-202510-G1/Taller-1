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
- `geometry_msgs`
- `pynput`
- `tkinter`
- `matplotlib`
- `pcl_msgs`

Si alguna de estas dependencias no está instalada, puedes instalarlas con:
```bash
pip install pynput matplotlib
```

## Instalación
1. Clona este repositorio en tu workspace de ROS2:
   ```bash
   cd ~/ros2_ws/src
   git clone <URL_DEL_REPOSITORIO>
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

- **test1**: Nodo de prueba inicial  
  Ejecutar con:  
  ```bash
  ros2 run controller test1
  ```

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

## Ejemplo de uso
Para teleoperar el TurtleBot con el teclado:
```bash
ros2 run controller turtle_bot_teleop
```

Para iniciar la interfaz gráfica:
```bash
ros2 run controller turtle_bot_interface
```

## Notas adicionales
- Asegúrate de tener una sesión de ROS2 corriendo antes de ejecutar los nodos (`ros2 daemon start`).
- Puedes agregar más nodos o modificar los existentes según sea necesario.

## Autores
- **Grupo X - Robótica**



