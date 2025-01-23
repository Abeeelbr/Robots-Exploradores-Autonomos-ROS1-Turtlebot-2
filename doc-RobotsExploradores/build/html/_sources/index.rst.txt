Robots Móviles - Documentación
==============================

¡Bienvenido a la documentación del proyecto de Robots Móviles!  
En este proyecto se abordan tecnologías de robótica móvil mediante **ROS (Robot Operating System)**, 
incluyendo exploración autónoma, detección de objetos y manejo de estados.  
La plataforma principal es el **TurtleBot 2**, tanto en entornos de simulación como en el robot real.

Este repositorio contiene múltiples paquetes que, en conjunto, permiten:

- **Explorar** un entorno desconocido y generar el mapa correspondiente (utilizando SLAM).
- **Detectar objetos** en tiempo real (por medio de un modelo de visión por computadora, por ejemplo YOLO).
- **Asignar y gestionar dinámicamente “estaciones”** basadas en la detección de objetos.
- **Ofrecer una interfaz de usuario** que coordina todo el sistema y facilita el control manual o autónomo del robot.

.. toctree::
   :maxdepth: 2
   :caption: Contenidos:

   introduccion
   configuracion
   uso
   squad_state_manager
   pkg_exploracion
   squad_object_detection_action
   squad_station_manager_node
   squad_interfaz


Paquetes y Nodos
----------------

El proyecto se compone de varios paquetes ROS, cada uno encargado de funcionalidades específicas:

- **Paquete Principal (`squad_main`)**  
  Contiene la máquina de estados principal (SMACH) y la **interfaz gráfica** (GUI) para el usuario.
  Gestiona los modos de operación (reposo, exploración, navegación, etc.) y coordina la comunicación con otros paquetes.

- **Paquete de Exploración (`squad_exploracion`)**  
  Proporciona la lógica de **exploración autónoma** basada en `explore_lite`, detecta fronteras del mapa 
  y coordina la navegación hacia dichas fronteras. Integra la **detección de objetos** en tiempo real 
  (por medio de un nodo que procesa las imágenes y coordena con YOLO).

- **Paquete de Navegación (`squad_navegacion`)**  
  Configura y gestiona la **navegación** (por ejemplo, usando `move_base`) y el algoritmo de **SLAM** (como `gmapping`). 
  Define parámetros de mapas de coste, sensores y transformación de marcos de referencia (TF).

- **Paquete de Planificación (`squad_planificacion`)**  
  Se encarga de la **gestión y creación dinámica de “estaciones”** (salas, zonas de interés o bases) a partir de 
  la detección de objetos. Ofrece servicios para asignar nuevas metas al robot y coordina la información 
  proveniente de la exploración y de la detección.

- **Paquete de Simulación (`squad_simulacion`)**  
  Centraliza **archivos de lanzamiento** para Gazebo y **modelos 3D** del entorno (mundo simulado, objetos, etc.). 
  Permite probar los algoritmos de exploración y navegación antes de desplegarlos en el robot físico.

Documentación de Referencia
---------------------------

Cada nodo clave de este sistema se describe en detalle en las secciones correspondientes:

- **Máquina de Estados** (`squad_state_manager.rst`):
  Presenta la estructura SMACH, los estados disponibles (reposo, exploración, navegación, etc.) 
  y cómo se realizan las transiciones entre ellos.

- **Exploración** (`pkg_exploracion.rst`):
  Explica la configuración y uso del algoritmo `explore_lite`, así como parámetros de exploración y cómo se integra 
  con la navegación y la detección.

- **Detección de Objetos** (`squad_object_detection_action.rst`):
  Describe el nodo que procesa las imágenes para identificar objetos en el entorno, calculando su posición 
  para crear “estaciones” o puntos de interés.

- **Gestión de Estaciones** (`squad_station_manager_node.rst`):
  Explica la lógica de asignación dinámica de salas/bases a partir de la detección de objetos 
  y su visualización en la interfaz o en RViz.

- **Interfaz Gráfica** (`squad_interfaz.rst`):
  Detalla la ventana principal en Tkinter, los botones de estado y la visualización de cámara y mapa. 
  Describe cómo se comunican los comandos de usuario hacia el robot.

Indices y Tablas
================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
