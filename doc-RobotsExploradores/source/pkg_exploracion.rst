Paquete de Exploración (`squad_exploracion`)
============================================

El paquete **squad_exploracion** proporciona las herramientas necesarias para que el robot **explore** 
un entorno desconocido y **detecte** objetos en el proceso, integrándose con el resto de módulos del sistema 
(navegación, asignación de estaciones, interfaz gráfica, etc.).

Descripción General
--------------------

La exploración se basa en el algoritmo de **fronteras** provisto por 
[**explore_lite**](http://wiki.ros.org/explore_lite), al cual se le han realizado 
algunas **modificaciones** para adaptarse a los requerimientos del proyecto:

- **Inicio y paro controlados**: Se añade un servicio o tópico para activar/desactivar 
  la exploración desde la **máquina de estados** o la **interfaz gráfica**.

- **Integración con la detección de objetos**: A medida que el robot recorre el mapa, 
  se publica la información de posibles detecciones en un tópico para su posterior 
  análisis y creación dinámica de “estaciones”.

Componentes Principales
-----------------------

1. **Nodo de Exploración (modificado de `explore_lite`)**

   - Identifica **fronteras** (zonas no exploradas) en el mapa.
   - Envía objetivos a la pila de **navegación** (`move_base`).
   - Permite pausar o continuar la exploración bajo demanda.
   - Crea una “lista negra” para fronteras inalcanzables o bloqueadas.

2. **Nodo de Detección de Objetos (`squad_object_detection_action`)**

   - Procesa imágenes RGB-D para **identificar objetos** ( con YOLOv8).
   - Publica la información de los objetos detectados en el tópico ``/detected_objects``.
   - Permite calcular la posición 3D del objeto y transformarla al marco ``map``.

   Para más detalles, consulta: :doc:`squad_object_detection_action`.

Interacción con Otros Paquetes
------------------------------

- **Navegación (`squad_navegacion`)**: El nodo de exploración depende de los parámetros 
  de SLAM y de los mapas de coste para planificar rutas hacia nuevas fronteras.
- **Planificación (`squad_planificacion`)**: El módulo de creación de “estaciones” 
  se alimenta de las detecciones de objetos que recibe desde `squad_exploracion`.
- **Máquina de Estados (`squad_state_manager`)**: Puede habilitar o deshabilitar 
  la exploración en función del modo de operación (reposo, exploración, navegación, etc.).

Ejecución del Paquete
---------------------

Para lanzar la exploración, se recomienda utilizar los archivos **launch** específicos 
(incluidos generalmente en `squad_simulacion` o `squad_main`). Ejemplo:

.. code-block:: bash

   roslaunch squad_main main_gazebo.launch

Si necesitas correr el nodo de exploración modificado de forma independiente, 
puedes invocarlo así (asegurándote de haberlo compilado en tu workspace de ROS):


.. code-block:: bash

   roslaunch squad_exploracion explore.launch

Según la configuración que hayas definido en tus archivos `.launch` y `.yaml`.

Tópicos Relevantes
------------------

- **`/explore/frontiers`**  
  Publica las fronteras detectadas en tiempo real (MarkerArray, útil para visualizar en RViz).

- **`/explore/goal`**  
  Objetivo de exploración activo, enviado a `move_base`.

- **`/detected_objects`**  
  Información sobre los objetos identificados por la cámara, incluyendo tipo y coordenadas.

Archivos de Configuración
-------------------------

- **`explore.launch`**  
  Ajusta parámetros como la **frecuencia** de exploración (`planner_frequency`), 
  el tamaño mínimo de una frontera (`min_frontier_size`), etc.

- **`explore_costmap_params.yaml`**  
  Configuración de los costmaps (global y local) empleados por la pila de navegación 
  durante la exploración.

- **`camera_params.yaml`** (si aplica)  
  Parámetros para el nodo de detección de objetos (umbral de confianza, clases a filtrar, etc.).

Pruebas y Consideraciones
-------------------------

1. **Pruebas en Simulación**  
   Se recomienda inicializar Gazebo con el mapa de un entorno (por ejemplo, el laboratorio 3D). 
   Luego, activar la exploración y verificar que el robot recorra las zonas no exploradas.

2. **Pruebas con un Robot Real**  
   - Ajustar variables de calibración (TF, odometría, etc.).  
   - Verificar que los **sensores** (LIDAR y cámara) estén publicando correctamente.  
   - Despejar o marcar las zonas que el robot no pueda atravesar.

3. **Integración con la Detección**  
   Durante la exploración, habilitar la detección de objetos para comprobar que 
   los eventos de detección se reciban en el topic `/detected_objects` y que 
   `squad_planificacion` los utilice para crear estaciones.

Este paquete constituye la **columna vertebral** de la capacidad de exploración del robot, 
permitiendo cubrir entornos desconocidos y complementarse con la detección de objetos para 
una **búsqueda autónoma** más avanzada. 
