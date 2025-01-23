Robots Móviles - Documentación
=================================

Bienvenido a la documentación del proyecto de Robots Móviles. Este proyecto explora conceptos clave de programación en robótica móvil utilizando ROS.

Introducción
------------

Este proyecto incluye una máquina de estados para gestión del flujo de trabajo del robot, procesamiento de datos de sensores, y navegación autónoma. 

.. toctree::
   :maxdepth: 2
   :caption: Contenidos:

   introduccion
   configuracion
   uso
   squad_state_manager
   pkg_exploracion
   squad_object_detection_action
   squad_approach_control_action
   squad_autonomous_control_action
   squad_station_manager_node
   squad_interfaz
   

Paquetes y Nodos
----------------

El proyecto se organiza en los siguientes paquetes:

1. **Paquete Principal (main):**
   - Contiene la máquina de estados y la interfaz gráfica.

2. **Paquete de Exploración (exploracion):**
   - Incluye nodos para detección de objetos, aproximación y control autónomo.

Documentación de Referencia
----------------------------

La documentación de cada nodo se detalla en las siguientes secciones:

- **Detección de Objetos:**
  .. include:: squad_object_detection_action.rst

- **Aproximación a Objetos:**
  .. include:: squad_approach_control_action.rst

- **Control Autónomo:**
  .. include:: squad_autonomous_control_action.rst

Indices y Tablas
================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

