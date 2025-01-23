Interfaz de Gestión de Estados del TurtleBot
============================================

Este módulo proporciona una **interfaz gráfica de usuario (GUI)** para interactuar con el **TurtleBot 2**, 
ofreciendo control manual o automático, visualización del mapa en tiempo real, y cambio de estados 
en la **máquina de estados** (reposo, exploración, navegación, etc.). Además, permite seleccionar 
“estaciones” o puntos de interés generados a partir de la detección de objetos y planificar la ruta del robot hacia ellas.

Introducción
------------

**Descripción General:**

El nodo ``squad_interfaz`` combina interacción manual y visualización en tiempo real, facilitando:

- **Cambio de modo de operación** (por ejemplo, REPOSO, EXPLORACIÓN, NAVEGACIÓN).
- **Visualización del mapa y de las estaciones** detectadas (en 2D, con posiciones del robot y de objetos).
- **Visualización de cámara en tiempo real** (RGB o RGB-D, si se habilita).
- **Selección y navegación a estaciones** detectadas (por ejemplo, sillas, personas, sofás, etc. si han sido clasificadas).
- **Control manual** (usando teclas de flecha) o **control autónomo** (comunicando con los nodos de exploración/navegación).

Dependencias
------------

- **ROS:**  
  - ``rospy``, ``smach``, ``smach_ros``, ``actionlib``
  - Mensajes: ``std_msgs.msg``, ``geometry_msgs.msg``, ``sensor_msgs.msg``
  - Transformaciones: ``tf``
- **Visión:**  
  - ``cv_bridge``  
  - ``cv2 (OpenCV)``  
  - ``PIL (Pillow)`` (para procesar y mostrar imágenes)
- **Navegación:**  
  - ``move_base_msgs.msg`` (para enviar objetivos de navegación)
- **Interfaz Gráfica:**  
  - ``tkinter`` (ventana principal y widgets)

Estructura del Nodo
-------------------

**Clase Principal:**
- **``InterfazManager``**  
  Configura y lanza la **GUI** con Tkinter, suscribe a los tópicos ROS necesarios y publica comandos 
  para cambiar el estado del robot. Proporciona métodos para actualizar en tiempo real:
  - El **mapa** (celdas de ocupación o imagen renderizada).
  - Las **estaciones** detectadas (lista de objetos y coordenadas).
  - La **imagen de la cámara** (cuadro a cuadro).

**Métodos Clave:**
- **``setup_gui``**:  
  Construye la ventana de Tkinter, configura botones de estados (Reposo, Exploración, Navegación), 
  contenedores para mapa y cámara, y menús de selección de estaciones.
- **``update_map_display``** y **``update_camera_display``**:  
  Actualizan en tiempo real la imagen del mapa y de la cámara, respectivamente, 
  utilizando datos suscritos a los tópicos de ROS.
- **``stations_callback``**:  
  Escucha el tópico de **estaciones detectadas** (por ejemplo, ``/stations_info``) y actualiza el menú desplegable 
  para que el usuario pueda seleccionar y dirigir al robot hacia esa estación.
- **``on_state_button_click``**:  
  Envía comandos a la máquina de estados del robot para cambiar el estado (reposo, exploración, navegación).
- **``subscribe_to_camera``**:  
  Configura la suscripción a la imagen RGB o RGB-D para actualizar la vista de la cámara.
- **``on_closing``**:  
  Método invocado al cerrar la GUI; permite una finalización segura de hilos y cierre de conexiones ROS.

Parámetros Configurables
------------------------

- **Tópicos de Entrada:**
  - ``/current_state``: Estado actual del robot (reposo, exploración, etc.).
  - ``/stations_info``: Lista de estaciones (objetos) detectadas en el entorno.
  - ``/map`` o ``/map_image``: Mapa de ocupación o imagen del entorno.
  - ``/camera/rgb/image_raw``: Imagen RGB para visualización (opcional).
- **Tópicos de Salida:**
  - ``/command``: Comandos para cambiar el estado en la máquina de estados.
  - ``/control_mode``: Alterna entre control manual y automático.
  - ``/cmd_vel``: Mensajes de velocidad para teleoperación manual.

- **Variables de Entorno** (opcional):
  - ``MODO_EJECUCION``: Indica si el robot está en **REAL** o **GAZEBO**.

Uso de la Interfaz
------------------

1. **Cambio de Estado**  
   - Botones para REPOSO, EXPLORACIÓN y NAVEGACIÓN.  
   - Cada botón envía un comando al nodo de **estado** para que el robot cambie su comportamiento.

2. **Visualización del Mapa**  
   - Muestra el mapa 2D y la ubicación del robot.  
   - Se actualiza constantemente al suscribirse al tópico de mapa.

3. **Selección de Estaciones**  
   - Menú desplegable con estaciones detectadas (por ejemplo, “Silla #1”, “Persona #2”).  
   - Botón “Ir” para enviar al robot a la estación seleccionada.  
   - El sistema se apoya en la pila de navegación para planificar la ruta hasta ese punto.

4. **Control Manual (Teleoperación)**  
   - El usuario puede presionar flechas del teclado para mover el robot:  
     - **Arriba/Abajo**: Avanzar o retroceder.  
     - **Izquierda/Derecha**: Rotar sobre sí mismo.  
   - Se publica en ``/cmd_vel``.

5. **Cierre de la Interfaz**  
   - Al cerrar la ventana, se llama a ``on_closing``, deteniendo subprocesos y hilos de actualización para evitar bloqueos.

Ejemplo de Ejecución
--------------------

Para ejecutar la interfaz en un entorno ya inicializado (por ejemplo, con el robot simulado):

.. code-block:: bash

   rosrun squad_main interfaz.py

(El nombre real del script o nodo puede variar.)

Si prefieres lanzar todo el sistema (navegación, detección y esta interfaz) de golpe, 
usa el archivo ``.launch`` correspondiente, por ejemplo:

.. code-block:: bash

   roslaunch squad_main main_gazebo.launch

Código Fuente
-------------

.. automodule:: squad_interfaz
    :members:
    :undoc-members:
    :show-inheritance:
