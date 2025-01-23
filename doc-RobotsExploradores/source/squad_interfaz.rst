Interfaz de Gestión de Estados del TurtleBot
============================================

Este script proporciona una interfaz gráfica de usuario (GUI) para interactuar con el TurtleBot y gestionar su máquina de estados, visualización de mapas y cámaras, así como su exploración.

Introducción
------------

**Descripción General:**

El nodo `interfaz` combina interacción manual y visualización en tiempo real del estado del robot. Permite:
- Cambiar entre modos de operación (reposo, exploración, navegación).
- Visualizar el mapa y las estaciones detectadas.
- Visualizar imágenes de la cámara en tiempo real.
- Enviar comandos de navegación a estaciones específicas.

**Dependencias:**
- `rospy`, `smach`, `smach_ros`
- `actionlib`
- `std_msgs.msg`, `geometry_msgs.msg`
- `sensor_msgs.msg`, `cv_bridge`
- `tf`, `PIL (Pillow)`
- `cv2 (OpenCV)`
- `move_base_msgs.msg`
- `tkinter`

Estructura del Nodo
--------------------

**Clases y Métodos Principales:**

1. **`InterfazManager`**
   - Configura la GUI utilizando `Tkinter`.
   - Gestiona la interacción con los tópicos de ROS.
   - Proporciona métodos para actualizar la visualización del mapa y de las cámaras.

2. **Métodos Clave:**
   - `setup_gui`: Configura la interfaz gráfica.
   - `update_visualization`: Actualiza en tiempo real el mapa, las estaciones y la posición del robot.
   - `stations_callback`: Actualiza la lista de estaciones detectadas.
   - `on_state_button_click`: Envía comandos para cambiar el estado del robot.
   - `subscribe_to_camera`: Gestiona la suscripción a los tópicos de la cámara.
   - `on_closing`: Cierra la GUI de forma segura.

Parámetros Configurables
------------------------

- **`MODO_EJECUCION`**: Especifica el entorno de ejecución (REAL, GAZEBO, STAGE).
- **Tópicos de ROS Configurados:**
  - `camera/rgb/image_raw`: Imagen de la cámara RGB.
  - `map`: Mapa generado por el TurtleBot.
  - `stations_info`: Información de las estaciones detectadas.

Tópicos Utilizados
------------------

**Suscriptores:**
- `current_state`: Recibe el estado actual del robot.
- `stations_info`: Recibe las estaciones detectadas.
- `map`: Recibe el mapa del entorno.
- `control_mode`: Actualiza el modo de control del robot.

**Publicadores:**
- `command`: Envía comandos para cambiar el estado de la máquina de estados.
- `control_mode`: Cambia el modo de control (manual/autónomo).
- `cmd_vel`: Envía comandos de movimiento.

Ejemplo de Ejecución
---------------------

Para ejecutar el nodo, usa el siguiente comando:

.. code-block:: bash

   rosrun squad_planificacion interfaz.py

Uso de la Interfaz
------------------

1. **Cambio de Estado:**
   - Botones para cambiar entre REPOSO, EXPLORACIÓN y NAVEGACIÓN.

2. **Visualización del Mapa:**
   - Muestra el mapa generado y las estaciones detectadas.
   - Actualización en tiempo real de la posición del robot.

3. **Selección de Estaciones:**
   - Desplegable para seleccionar una estación.
   - Botón "Ir" para enviar al robot hacia la estación seleccionada.

4. **Control Manual:**
   - Uso de teclas de flecha para mover el robot.
   - `Up`: Avanzar.
   - `Down`: Retroceder.
   - `Left`: Girar a la izquierda.
   - `Right`: Girar a la derecha.

Código Fuente
-------------

.. automodule:: squad_interfaz
    :members:
    :undoc-members:
    :show-inheritance: