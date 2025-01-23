RobotsExploradores Documentation
================================

Bienvenido a la documentación del proyecto RobotsExploradores. Este proyecto está diseñado para trabajar con robótica móvil utilizando ROS y se divide en dos paquetes principales: **main** y **exploracion**.

Esta documentación detalla los componentes, configuraciones y pasos necesarios para poner en marcha este sistema robótico.

Introducción
------------

El objetivo del proyecto es coordinar diversas tareas en un robot móvil. Estas incluyen detección de objetos, aproximación a puntos específicos y navegación autónoma. A continuación, se presentan los principales paquetes y componentes del sistema:

**Paquete Principal (main):**
- Implementa la máquina de estados para gestionar el flujo de trabajo del robot.
- Proporciona una interfaz gráfica para interactuar con el robot.

**Paquete de Exploración (exploracion):**
- Define los servidores de acción para tareas clave como detección de objetos y control autónomo.

.. note::

   Asegúrate de haber instalado todos los paquetes y dependencias necesarios antes de continuar.

.. toctree::
   :maxdepth: 2
   :caption: Contenidos:

   introduccion
   configuracion
   uso
   squad_state_manager
   pkg_exploracion

Indices y tablas
================

* :ref:`genindex`


.. * :ref:`modindex`
.. * :ref:`search`

