Introducción
============

Este proyecto forma parte de la asignatura de **Robots Móviles** en la Universidad de Alicante y aborda el diseño y la implementación de un sistema de robótica móvil basado en **ROS**.  
La plataforma principal es un **TurtleBot 2**, que puede operar tanto en un entorno simulado como en el robot físico real.

El proyecto integra distintos módulos para:
- **Explorar** entornos desconocidos y generar un mapa en tiempo real (SLAM).
- **Detectar** objetos en el entorno (p. ej., con un modelo YOLO) y clasificarlos.
- **Asignar** dinámicamente “estaciones” (zonas de interés) a partir de la detección de objetos.
- **Coordinar** todos los procesos mediante una **máquina de estados** y ofrecer una **interfaz gráfica** para el usuario.

Objetivos principales
---------------------

1. **Diseñar e implementar** una máquina de estados que gestione los diferentes comportamientos (reposo, exploración, navegación, etc.) de forma modular.  
2. **Realizar detección de objetos** en tiempo real, aprovechando datos de cámara RGB-D y un modelo de redes neuronales para visión por computador.  
3. **Implementar un sistema de exploración y SLAM** que permita al robot generar o actualizar un mapa 2D del entorno de manera autónoma, utilizando sensores LIDAR.  
4. **Desarrollar una interfaz gráfica** (GUI) que ofrezca visualización en tiempo real y control del robot, incluyendo funciones de teleoperación manual y estados automáticos.  

Características clave
---------------------

- **Uso de SMACH**: para la orquestación de tareas y la definición de estados del robot (p. ej., exploración, detección, navegación).  
- **Integración de procesamiento de datos RGB-D y LIDAR**: se combinan lecturas de láser (Hokuyo) y cámara de profundidad para la navegación y la detección de objetos.  
- **Detección de objetos con YOLO**: un servidor de acción en ROS procesa las imágenes para identificar personas, sillas, mesas u otros objetos de interés.  
- **Asignación dinámica de “estaciones”**: el sistema crea nuevas “salas” o “bases” según las detecciones, permitiendo planificar rutas y visualizarlas en RViz o la interfaz.  
- **Modularidad y escalabilidad**: se organiza en paquetes ROS específicos (`squad_main`, `squad_exploracion`, `squad_navegacion`, `squad_planificacion`, `squad_simulacion`) para facilitar la ampliación futura, incluyendo la posibilidad de gestionar múltiples robots.  

Este proyecto proporciona una visión general de cómo integrar varios componentes de ROS (exploración, SLAM, detección de objetos, interfaz gráfica) en un único sistema coherente y funcional, con miras a aplicaciones de robótica móvil como la **búsqueda y rescate**, la logística o la monitorización de entornos interiores.  
