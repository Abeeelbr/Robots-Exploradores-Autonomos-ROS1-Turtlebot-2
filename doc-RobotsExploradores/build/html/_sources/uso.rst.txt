Uso del Sistema
===============

Este sistema utiliza **ROS** junto con una **máquina de estados** basada en **SMACH** 
para gestionar diferentes comportamientos (reposo, exploración, navegación, etc.) 
en el contexto de la **robótica móvil** con TurtleBot 2.  
Además, integra nodos especializados para **detección de objetos** y **exploración**, 
así como un **interfaz gráfico** para el control y la visualización en tiempo real.

Pasos para Lanzar el Sistema
----------------------------

1. **Preparar el Entorno**

   - Verifica que tienes un **workspace de ROS** (por ejemplo, `robots_moviles_ws`) 
     y que el proyecto está dentro de la carpeta `src`.
   - Asegúrate de **instalar** todas las **dependencias** (bibliotecas ROS, paquetes de visión, etc.).
   - **Compila** el proyecto:

   .. code-block:: bash

      cd ~/robots_moviles_ws
      catkin_make
      source devel/setup.bash

2. **Probar los Nodos Individualmente (Opcional)**

   Puedes lanzar los nodos de forma independiente para asegurarte de que funcionan:

   - **Detección de Objetos**  
     
     .. code-block:: bash

        rosrun squad_exploracion squad_object_detection_action.py

     Observa en la consola y en el tópico `/detected_objects` si se están publicando detecciones.

   - **Exploración Autónoma**  
     
     (Si deseas solo poner en marcha la lógica de `explore_lite` o el nodo modificado):

     .. code-block:: bash

        roslaunch squad_exploracion explore.launch

     Verifica en **RViz** que las fronteras se detecten y el robot se mueva para explorar.

3. **Lanzar el Sistema Completo**

   Para iniciar todos los nodos (máquina de estados, interfaz, exploración, detección, 
   SLAM, etc.) de un golpe:

   .. code-block:: bash

      roslaunch squad_main main_gazebo.launch

   (o bien `main_real.launch` si estás usando un TurtleBot 2 real).  

   Esto iniciará:

   - La **máquina de estados** (`squad_state_manager`).
   - La **interfaz gráfica** (`squad_interfaz`).
   - El nodo de **detección de objetos** (`squad_object_detection_action`).
   - La **navegación** (SLAM con `gmapping` y `move_base`).
   - Opcionalmente, la **simulación** en Gazebo (si es `main_gazebo.launch`).

Componentes Principales
-----------------------

- **Máquina de Estados (`squad_state_manager`)**  
  Gestiona las transiciones entre estados del robot: **reposo**, **exploración**, **navegación**, etc.  
  Publica el estado actual en el tópico `/current_state`.

- **Nodos Especializados**  

  - **`squad_object_detection_action`**  Procesa datos de la cámara RGB-D para **detectar objetos** y publicar sus posiciones.  
  - **Exploración** (basado en `explore_lite` modificado)  Envía objetivos de frontera a la pila de navegación (`move_base`).

- **Interfaz Gráfica (`squad_interfaz`)**  

  - Permite control manual o automático.  
  - Muestra el mapa, la cámara en vivo y las estaciones detectadas.  
  - Facilita el cambio de estado (reposo, exploración, navegación) y la selección de estaciones.

Pruebas del Sistema
-------------------

1. **Prueba de Reposo** 

   - Inicia el sistema y comprueba que el robot está en el estado **Reposo** (se puede ver en `/current_state` o en la interfaz gráfica).

2. **Prueba de Exploración**  

   - Envía el comando `"modo_exploracion"` al tópico `/command` o usa la interfaz gráfica para cambiar al estado de exploración.  
   - Verifica que `explore_lite` (su versión modificada) se active y que el robot empiece a explorar.  
   - Asegúrate de que el nodo `squad_object_detection_action` esté funcionando, publicando en `/detected_objects` cuando visualice objetos.

3. **Prueba de Navegación**  

   - Envía un comando para ir a una estación (por ejemplo, `"ir_a_HOME"`) o selecciona una estación en la interfaz gráfica.  
   - El robot pasará al estado **Navegación** (usa `move_base` para desplazarse a esa posición).

Configuraciones Requeridas
--------------------------

Los parámetros del sistema se encuentran en diversos archivos YAML dentro de los 
paquetes (`squad_navegacion`, `squad_exploracion`, etc.):

- **`configGeneral.yaml`**  
  Configuración general del sistema (tópicos, frecuencias, etc.).
- **`gmapping_params.yaml`**  
  Parámetros relacionados con SLAM (GMapping).
- **`config.launch (Exploracion)`**  
  Ajusta la frecuencia de planificación, tamaño mínimo de fronteras, etc.
- **`move_base_params.yaml`**  
  Configuración de la pila de navegación (track_unknown_space).

Tópicos Importantes
-------------------

- **`/current_state`**  
  Publica el estado actual (reposo, exploración, navegación, etc.).
- **`/command`**  
  Recibe comandos como `"start_exploration"` o `"stop"`.
- **`/detected_objects`**  
  Información de los objetos detectados (tipo, confianza, coordenadas).
- **`/stations_info`**  
  Estaciones creadas dinámicamente a partir de la detección de objetos.
- **`/cmd_vel`**  
  Mensajes de velocidad para el robot (teleoperación manual u órdenes de move_base).
- **'/map', '/scan', '/odom'**  
  Tópicos de SLAM y sensores (láser, odometría).

Con este conjunto de herramientas y pasos, tu **TurtleBot 2** (real o simulado) quedará 
listo para realizar **exploración**, **detección de objetos** y **navegación** con 
visualización y control en tiempo real. ¡Disfruta de la experiencia de robótica móvil!  
